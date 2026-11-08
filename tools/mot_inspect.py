#!/usr/bin/env python3
"""Read-only inspector for RE Engine motion containers.

Supports the layouts used by the large-monster data dump:
  * motbank  (magic ``mbnk``, version 4)  - banks of motion lists
  * motlist  (magic ``mlst``, version 992) - motion tables with embedded MOT / MTREE
  * mot      (magic ``mot ``, version 932) - keyframed motion
  * mtree    (magic ``mtre``, version 21)  - motion blend graphs

The field offsets below were verified against the dump and cross-checked with
the public REasy layout tables for Monster Hunter Wilds.  The tool never writes
to the inspected files; it only prints what it reads.
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys

MOTBANK_MAGIC = b"mbnk"
MOTLIST_MAGIC = b"mlst"
MOT_MAGIC = b"mot "
TREE_MAGIC = b"mtre"

# motlist.992
LIST_HEADER_SIZE = 54
LIST_ROW_SIZE = 72

# mot.932
MOT_HEADER_SIZE = 126
MOT_JOINT_SIZE = 80
MOT_CHANNEL_SIZE = 12
MOT_TRACK_SIZE = 20
SOLVERS = {162: "float", 242: "vector3", 274: "quaternion"}
FRAME_TYPES = {2: "u8", 4: "u16", 5: "u32"}
TRANSFORM_NAMES = ("translation", "rotation", "scale")

# mtree.21
TREE_HEADER_SIZE = 88
TREE_NODE_SIZE = 48
TREE_LINK_SIZE = 40
TREE_REMAP_SIZE = 6
PARAM_KINDS = {0: "bool", 6: "i32", 9: "f32", 12: "string16"}

# Known MotTree node parameter keys (asset, bank id, motion id, blend rate).
PARAM_ASSET = 3620651061
PARAM_BANK_ID = 1557423995
PARAM_MOTION_ID = 2673430856
PARAM_BLEND_RATE = 2370884634


class FormatError(ValueError):
    """Raised when a file does not match the expected RE Engine layout."""


class Reader:
    """Bounds-checked little-endian reader over one buffer."""

    def __init__(self, data: bytes, base: int = 0):
        self.data = data
        self.base = base

    def _unpack(self, fmt: str, offset: int):
        try:
            return struct.unpack_from("<" + fmt, self.data, self.base + offset)
        except struct.error as exc:
            raise FormatError(f"read of {fmt} at 0x{offset:X} leaves the file") from exc

    def u8(self, offset: int) -> int:
        return self._unpack("B", offset)[0]

    def u16(self, offset: int) -> int:
        return self._unpack("H", offset)[0]

    def u32(self, offset: int) -> int:
        return self._unpack("I", offset)[0]

    def i32(self, offset: int) -> int:
        return self._unpack("i", offset)[0]

    def u64(self, offset: int) -> int:
        return self._unpack("Q", offset)[0]

    def f32(self, offset: int) -> float:
        return self._unpack("f", offset)[0]

    def utf16z(self, offset: int) -> str:
        start = self.base + offset
        end = start
        while True:
            end = self.data.find(b"\0\0", end)
            if end < 0:
                raise FormatError(f"unterminated UTF-16 string at 0x{offset:X}")
            if (end - start) % 2 == 0:
                break
            end += 1
        return self.data[start:end].decode("utf-16-le", "replace")

    def utf8z(self, offset: int) -> tuple[str, int]:
        start = self.base + offset
        end = self.data.find(b"\0", start)
        if end < 0:
            raise FormatError(f"unterminated UTF-8 string at 0x{offset:X}")
        return self.data[start:end].decode("utf-8", "replace"), end + 1 - self.base

    def check_magic(self, offset: int, magic: bytes) -> None:
        found = self.data[self.base + offset:self.base + offset + len(magic)]
        if found != magic:
            raise FormatError(f"expected magic {magic!r}, found {found!r}")


def read_mot(reader: Reader) -> dict:
    """Read the MOT header plus skeleton/channel/track summaries."""
    reader.check_magic(4, MOT_MAGIC)
    header = {
        "version": reader.u32(0),
        "error_flags": reader.u32(8),
        "master_size": reader.u32(12),
        "name": reader.utf16z(reader.u64(88)),
        "end_frame": reader.f32(96),
        "loop_frame": reader.f32(100),
        "raw_start_frame": reader.f32(104),
        "raw_end_frame": reader.f32(108),
        "joint_count": reader.u16(112),
        "channel_count": reader.u16(114),
        "sequence_count": reader.u8(116),
        "sync_count": reader.u8(117),
        "fps": reader.u16(120),
        "property_count": reader.u16(122),
        "attributes": reader.u16(124),
    }
    header["joints"] = read_skeleton(reader, header) if not header["master_size"] else []
    header["tracks"] = read_tracks(reader, header)
    return header


def read_skeleton(reader: Reader, header: dict) -> list[str]:
    skeleton = reader.u64(16)
    if not skeleton:
        return []
    count = reader.u16(112)
    table = reader.u64(skeleton)
    names = []
    for index in range(count):
        joint = table + index * MOT_JOINT_SIZE
        names.append(reader.utf16z(reader.u64(joint)))
    return names


def read_tracks(reader: Reader, header: dict) -> dict:
    count = header["channel_count"]
    if not count:
        return {"count": 0, "key_frames": [], "channels": 0}
    animation = reader.u64(24)
    total = 0
    keys = []
    for index in range(count):
        channel = animation + index * MOT_CHANNEL_SIZE
        transforms = reader.u8(channel + 2)
        track = reader.u32(channel + 8)
        for bit in range(3):
            if not transforms >> bit & 1:
                continue
            packed = reader.u32(track)
            solver = packed & 0xFFF
            key_count = reader.u32(track + 4)
            frames = reader.u32(track + 8)
            keys.append({
                "channel": index,
                "transform": TRANSFORM_NAMES[bit],
                "solver": SOLVERS.get(solver, f"0x{solver:X}"),
                "encoding": (packed >> 12) & 0xFF,
                "frame_type": FRAME_TYPES.get((packed >> 20) & 0xF, "?"),
                "key_count": key_count,
                "frame_pointer": frames,
            })
            total += key_count
            track += MOT_TRACK_SIZE
    return {"count": len(keys), "keys": total, "channels": count, "tracks": keys}


def read_tree(reader: Reader) -> dict:
    """Read one MTREE graph.  Class names are stored inline after the node table."""
    reader.check_magic(4, TREE_MAGIC)
    node_count = reader.u16(72)
    nodes_offset = reader.u64(32)
    links_offset = reader.u64(40)
    name = reader.utf16z(reader.u64(48))
    cursor = nodes_offset + node_count * TREE_NODE_SIZE
    nodes = []
    for index in range(node_count):
        node = nodes_offset + index * TREE_NODE_SIZE
        class_name, cursor = reader.utf8z(cursor)
        tags = []
        tag_count = reader.u8(node + 44)
        tag_table = reader.u64(node + 8)
        if tag_table and tag_count:
            for tag in range(tag_count):
                tags.append(reader.utf16z(reader.u64(tag_table + tag * 8)))
        params = []
        param_count = reader.u8(node + 45)
        param_table = reader.u64(node + 24)
        for item in range(param_count):
            entry = param_table + item * 16
            kind = reader.u32(entry)
            key = reader.u32(entry + 4)
            value = reader.u64(entry + 8)
            if kind == 12:
                value = reader.utf16z(value & 0xFFFFFFFF)
            elif kind == 9:
                value = struct.unpack("<f", struct.pack("<I", value & 0xFFFFFFFF))[0]
            elif kind == 6:
                raw = value & 0xFFFFFFFF
                value = raw - (1 << 32) if raw & 0x80000000 else raw
            params.append({"type": PARAM_KINDS.get(kind, f"0x{kind:X}"), "key": key, "value": value})
        nodes.append({
            "class": class_name,
            "name": reader.utf16z(reader.u64(node)),
            "tags": tags,
            "params": params,
        })
    links = []
    links_count = reader.u16(74)
    for index in range(links_count):
        link = links_offset + index * TREE_LINK_SIZE
        links.append({
            "parent": reader.u32(link),
            "pin": reader.u32(link + 4),
            "child": reader.u32(link + 8),
            "child_pin": reader.u32(link + 12),
            "type": reader.u32(link + 16),
        })
    return {
        "version": reader.u32(0),
        "name": name,
        "node_count": node_count,
        "link_count": links_count,
        "root": reader.u16(76),
        "nodes": nodes,
        "links": links,
    }


def read_motlist(path: str, with_entries: bool = True) -> dict:
    data = open(path, "rb").read()
    reader = Reader(data)
    reader.check_magic(4, MOTLIST_MAGIC)
    count = reader.u32(48)
    motions = reader.u64(16)
    rows = reader.u64(24)
    result = {
        "path": path,
        "kind": "motlist",
        "size": len(data),
        "version": reader.u32(0),
        "name": reader.utf16z(reader.u64(32)),
        "base_list": reader.utf16z(reader.u64(40)) if reader.u64(40) else None,
        "count": count,
        "joint_mask_mode": reader.u8(52),
        "entries": [],
        "totals": {"mot": 0, "tree": 0, "reference": 0},
    }
    if not with_entries:
        return result
    for index in range(count):
        row = rows + index * LIST_ROW_SIZE
        motion_id = reader.u32(row + 8)
        pointer = reader.u64(motions + index * 8)
        entry = {
            "index": index,
            "motion_id": motion_id,
            "type": reader.u8(row + 20),
            "flags": reader.u8(row + 21),
            "physics": reader.u8(row + 22),
        }
        if not pointer:
            entry["kind"] = "inherited"
        else:
            magic = data[pointer + 4:pointer + 8]
            if magic == MOT_MAGIC:
                entry["kind"] = "mot"
                entry["mot"] = read_mot(Reader(data, pointer))
            elif magic == TREE_MAGIC:
                entry["kind"] = "tree"
                entry["tree"] = read_tree(Reader(data, pointer))
            else:
                entry["kind"] = "reference"
                entry["reference"] = Reader(data, pointer).utf16z(4)
        result["totals"][entry["kind"]] = result["totals"].get(entry["kind"], 0) + 1
        result["entries"].append(entry)
    return result


def read_motbank(path: str) -> dict:
    data = open(path, "rb").read()
    reader = Reader(data)
    reader.check_magic(4, MOTBANK_MAGIC)
    version = reader.i32(0)
    count = reader.i32(40)
    motlists = reader.u64(16)
    result = {
        "path": path,
        "kind": "motbank",
        "size": len(data),
        "version": version,
        "user_variables": reader.utf16z(reader.u64(24)) if reader.u64(24) else None,
        "joint_map": reader.utf16z(reader.u64(32)) if version >= 3 and reader.u64(32) else None,
        "banks": [],
    }
    item_size = 24 if version >= 3 else 16
    for index in range(count):
        item = motlists + index * item_size
        pointer = reader.u64(item)
        if version >= 3:
            bank_id = reader.i32(item + 8)
            bank_type = reader.u32(item + 12)
            mask = reader.u64(item + 16)
        else:
            bank_id = reader.i32(item + 8)
            bank_type = 0
            mask = reader.u64(item + 16)
        result["banks"].append({
            "bank_id": bank_id,
            "bank_type": bank_type,
            "mask_bits": mask,
            "path": reader.utf16z(pointer) if pointer else None,
        })
    return result


def read_mot_file(path: str) -> dict:
    data = open(path, "rb").read()
    return {"path": path, "kind": "mot", "size": len(data), "mot": read_mot(Reader(data))}


def inventory(root: str) -> dict:
    """Aggregate motion statistics below a directory without dumping every entry."""
    summary = {"root": root, "kind": "inventory", "motlists": [], "motbank": [], "standalone_mot": 0}
    for folder, _dirs, files in os.walk(root):
        for name in files:
            path = os.path.join(folder, name)
            lower = name.lower()
            if lower.endswith(".motbank.4"):
                summary["motbank"].append(read_motbank(path))
            elif lower.endswith(".motlist.992"):
                entry = read_motlist(path, with_entries=True)
                motions = [item["mot"] for item in entry["entries"] if item["kind"] == "mot"]
                summary["motlists"].append({
                    "name": entry["name"],
                    "count": entry["count"],
                    "embedded_mot": len(motions),
                    "tree": entry["totals"].get("tree", 0),
                    "reference": entry["totals"].get("reference", 0),
                    "total_seconds": round(sum(item["end_frame"] for item in motions) / 60.0, 2),
                    "fps": sorted({item["fps"] for item in motions}),
                    "joints": sorted({item["joint_count"] for item in motions}),
                })
            elif lower.endswith(".mot.932"):
                summary["standalone_mot"] += 1
    return summary


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Inspect RE Engine motion containers (read-only).")
    parser.add_argument("command", choices=("bank", "list", "mot", "tree", "inventory"))
    parser.add_argument("path", help="file or directory to inspect")
    parser.add_argument("--json", action="store_true", help="print JSON instead of text")
    parser.add_argument("--first", type=int, default=5, help="entries to show per list (text mode)")
    args = parser.parse_args(argv)

    if args.command == "bank":
        result = read_motbank(args.path)
    elif args.command == "list":
        result = read_motlist(args.path)
    elif args.command == "mot":
        result = read_mot_file(args.path)
    elif args.command == "tree":
        result = read_motlist(args.path)
    else:
        result = inventory(args.path)

    if args.json:
        json.dump(result, sys.stdout, indent=2, ensure_ascii=False)
        sys.stdout.write("\n")
        return 0

    if result["kind"] == "motbank":
        print(f"{result['path']}: version {result['version']}, {len(result['banks'])} banks")
        print(f"  user variables: {result['user_variables']}")
        print(f"  joint map:      {result['joint_map']}")
        for bank in result["banks"]:
            print(f"  bank {bank['bank_id']:>3} type={bank['bank_type']} {bank['path']}")
    elif result["kind"] == "motlist":
        counts = result["totals"]
        print(f"{result['path']}: motlist {result['version']}, {result['count']} entries, "
              f"{result['size']} bytes")
        print(f"  name: {result['name']}  base_list: {result['base_list']}")
        print(f"  kinds: {counts}")
        shown = 0
        for entry in result["entries"]:
            if entry["kind"] != "mot" or shown >= args.first:
                continue
            mot = entry["mot"]
            shown += 1
            print(f"  id {entry['motion_id']:>4} {mot['name']}: {mot['end_frame']:.1f} frames, "
                  f"{mot['fps']} fps, joints={mot['joint_count']}, channels={mot['channel_count']}, "
                  f"sequences={mot['sequence_count']}, sync={mot['sync_count']}")
        for entry in result["entries"]:
            if entry["kind"] == "tree":
                tree = entry["tree"]
                print(f"  tree {tree['name']}: {tree['node_count']} nodes, {tree['link_count']} links")
                break
        for entry in result["entries"]:
            if entry["kind"] == "reference":
                print(f"  reference example: {entry['reference']}")
                break
    elif result["kind"] == "mot":
        mot = result["mot"]
        print(f"{result['path']}: mot {mot['version']}, {result['size']} bytes")
        print(f"  name={mot['name']} end={mot['end_frame']} fps={mot['fps']} "
              f"joints={mot['joint_count']} channels={mot['channel_count']} "
              f"sequences={mot['sequence_count']} sync={mot['sync_count']} "
              f"attributes=0x{mot['attributes']:X}")
        print(f"  tracks: {mot['tracks']['count']} tracks, {mot['tracks'].get('keys', 0)} keys")
        if mot["joints"]:
            print(f"  joints: {', '.join(mot['joints'][:12])} ...")
    else:
        print(f"{result['root']}: {len(result['motlists'])} motlists, "
              f"{len(result['motbank'])} motbanks, {result['standalone_mot']} standalone mot files")
        for item in result["motlists"]:
            print(f"  {item['name']}: entries={item['count']} mot={item['embedded_mot']} "
                  f"tree={item['tree']} ref={item['reference']} "
                  f"seconds={item['total_seconds']} fps={item['fps']} joints={item['joints']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
