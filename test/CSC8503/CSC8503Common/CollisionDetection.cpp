#include "CollisionDetection.h"
#include "CollisionVolume.h"
#include "AABBVolume.h"
#include "OBBVolume.h"
#include "SphereVolume.h"
#include "../../Common/Vector2.h"
#include "../../Common/Window.h"
#include "../../Common/Maths.h"
#include "Debug.h"
#include <vector>
#include <algorithm>

#include <list>

using namespace NCL;

bool CollisionDetection::RayPlaneIntersection(const Ray&r, const Plane&p, RayCollision& collisions) {
	float ln = Vector3::Dot(p.GetNormal(), r.GetDirection());

	if (ln == 0.0f) {
		return false; //direction vectors are perpendicular!
	}
	
	Vector3 planePoint = p.GetPointOnPlane();

	Vector3 pointDir = planePoint - r.GetPosition();

	float d = Vector3::Dot(pointDir, p.GetNormal()) / ln;

	collisions.collidedAt = r.GetPosition() + (r.GetDirection() * d);

	return true;
}

bool CollisionDetection::RayIntersection(const Ray& r,GameObject& object, RayCollision& collision) {
	bool hasCollided = false;

	const Transform& worldTransform = object.GetTransform();
	const CollisionVolume* volume	= object.GetBoundingVolume();

	if (!volume) {
		return false;
	}

	switch (volume->type) {
		case VolumeType::AABB:		hasCollided = RayAABBIntersection(r, worldTransform, (const AABBVolume&)*volume	, collision); break;
		case VolumeType::OBB:		hasCollided = RayOBBIntersection(r, worldTransform, (const OBBVolume&)*volume	, collision); break;
		case VolumeType::Sphere:	hasCollided = RaySphereIntersection(r, worldTransform, (const SphereVolume&)*volume	, collision); break;
		case VolumeType::Capsule:	hasCollided = RayCapsuleIntersection(r, worldTransform, (const CapsuleVolume&)*volume, collision); break;
	}

	return hasCollided;
}

bool CollisionDetection::RayBoxIntersection(const Ray&r, const Vector3& boxPos, const Vector3& boxSize, RayCollision& collision) {
	Vector3 boxMin = boxPos - boxSize;
	Vector3 boxMax = boxPos + boxSize;

	Vector3 rayPos = r.GetPosition();
	Vector3 rayDir = r.GetDirection();

	Vector3 tVals(-1, -1, -1);

	for (int i = 0; i < 3; ++i) {//get best 3 intersections
		if (rayDir[i] > 0) {
			tVals[i] = (boxMin[i] - rayPos[i]) / rayDir[i];
		}
		else if (rayDir[i] < 0) {
			tVals[i] = (boxMax[i] - rayPos[i]) / rayDir[i];
		}
	}
	float bestT = tVals.GetMaxElement();
	if (bestT < 0.0f) {
		return false; // no backwards rays!
	}

	Vector3 intersection = rayPos + (rayDir * bestT);
	const float epsilon = 0.0001f; // an amount of leeway in our calcs
	for (int i = 0; i < 3; ++i) {
		if (intersection[i] + epsilon < boxMin[i] || intersection[i] - epsilon > boxMax[i]) {
			return false; //best intersection doesn't touch the box!
		}
	}
	collision.collidedAt = intersection;
	collision.rayDistance = bestT;

	return true;
}

bool CollisionDetection::RayAABBIntersection(const Ray&r, const Transform& worldTransform, const AABBVolume& volume, RayCollision& collision) {
	Vector3 boxPos = worldTransform.GetPosition();
	Vector3 boxSize = volume.GetHalfDimensions();
	return RayBoxIntersection(r,boxPos,boxSize,collision);
}

bool CollisionDetection::RayOBBIntersection(const Ray&r, const Transform& worldTransform, const OBBVolume& volume, RayCollision& collision) {
	Quaternion orientation = worldTransform.GetOrientation();
	Vector3 position = worldTransform.GetPosition();

	Matrix3 transform = Matrix3(orientation);
	Matrix3 invTransform = Matrix3(orientation.Conjugate());

	Vector3 localRayPos = r.GetPosition() - position;

	Ray tempRay(invTransform * localRayPos, invTransform * r.GetDirection());

	bool collided = RayBoxIntersection(tempRay, Vector3(), volume.GetHalfDimensions(), collision);

	if (collided) {
		collision.collidedAt = transform * collision.collidedAt + position;
	}

	return collided;
}

bool CollisionDetection::RayCapsuleIntersection(const Ray& r, const Transform& worldTransform, const CapsuleVolume& volume, RayCollision& collision) {

	Vector3 ray_Dir = r.GetDirection().Normalised();
	Vector3 ray_Pos = r.GetPosition();
	Vector3 Cap_Dir = worldTransform.GetOrientation() * Vector3(0, 1, 0);
	Vector3 Cap_Pos = worldTransform.GetPosition();

	Vector3 normal = Vector3::Cross(ray_Dir, Cap_Dir);

	//LineLineNormalProjection_A
	Vector3 normPlane_ray = Vector3::Cross(ray_Dir, normal);
	Vector3 planePt_ray = Cap_Pos;
	Vector3 dirline_ray = ray_Dir;
	Vector3 linePt_ray = ray_Pos;
	float t_ray = (Vector3::Dot((planePt_ray - linePt_ray), normPlane_ray)) / (Vector3::Dot(dirline_ray, normPlane_ray)); //Check Edge Parallel
	Vector3 ptIntersect_ray = linePt_ray + dirline_ray * t_ray;

	//LineLineNormalProjection_B
	Vector3 normPlane_Cap = Vector3::Cross(Cap_Dir, normal);
	Vector3 planePt_Cap = ray_Pos;
	Vector3 dirline_Cap = Cap_Dir;
	Vector3 linePt_Cap = Cap_Pos;
	float t_Cap = (Vector3::Dot((planePt_Cap - linePt_Cap), normPlane_Cap)) / (Vector3::Dot(dirline_Cap, normPlane_Cap)); //Check Edge Parallel
	Vector3 ptIntersect_cap = linePt_Cap + dirline_Cap * t_Cap;

	// linelinedistance
	float linlinedistance =abs(Vector3:: Dot((Cap_Pos - ptIntersect_ray), normal));

	if (linlinedistance < volume.GetRadius()) {
		collision.rayDistance =0.0f;
		collision.collidedAt = Vector3();
		return true;
	}

	return false;
}

bool CollisionDetection::RaySphereIntersection(const Ray&r, const Transform& worldTransform, const SphereVolume& volume, RayCollision& collision) {
	Vector3 spherePos = worldTransform.GetPosition();
	float sphereRadius = volume.GetRadius();
	
	//GEt the direction between the ray origin and the sphere origin
	Vector3 dir = (spherePos - r.GetPosition());

	//Then project the sphere's origin onto our ray direction vector
	float sphereProj = Vector3::Dot(dir, r.GetDirection());

	if (sphereProj < 0.0f) {
		return false; //point is behind the ray!
	}

	//Get closest point on the ray line to sphere
	Vector3 point = r.GetPosition() + (r.GetDirection() * sphereProj);

	float sphereDist = (point - spherePos).Length();

	if (sphereDist > sphereRadius) {
		return false;
	}

	float offset = sqrt((sphereRadius * sphereRadius) - (sphereDist * sphereDist));
	collision.rayDistance = sphereProj - (offset);
	collision.collidedAt = r.GetPosition() + (r.GetDirection() * collision.rayDistance);

	return true;
}

Matrix4 GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
		Matrix4::Translation(position) *
		Matrix4::Rotation(-yaw, Vector3(0, -1, 0)) *
		Matrix4::Rotation(-pitch, Vector3(-1, 0, 0));

	return iview;
}

Vector3 CollisionDetection::Unproject(const Vector3& screenPos, const Camera& cam) {
	Vector2 screenSize = Window::GetWindow()->GetScreenSize();

	float aspect	= screenSize.x / screenSize.y;
	float fov		= cam.GetFieldOfVision();
	float nearPlane = cam.GetNearPlane();
	float farPlane  = cam.GetFarPlane();

	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(cam) * GenerateInverseProjection(aspect, fov, nearPlane, farPlane);

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(screenPos.x / (float)screenSize.x) * 2.0f - 1.0f,
		(screenPos.y / (float)screenSize.y) * 2.0f - 1.0f,
		(screenPos.z),
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}

Ray CollisionDetection::BuildRayFromMouse(const Camera& cam) {
	Vector2 screenMouse = Window::GetMouse()->GetAbsolutePosition();
	Vector2 screenSize	= Window::GetWindow()->GetScreenSize();

	//We remove the y axis mouse position from height as OpenGL is 'upside down',
	//and thinks the bottom left is the origin, instead of the top left!
	Vector3 nearPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		-0.99999f
	);

	//We also don't use exactly 1.0 (the normalised 'end' of the far plane) as this
	//causes the unproject function to go a bit weird. 
	Vector3 farPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		0.99999f
	);

	Vector3 a = Unproject(nearPos, cam);
	Vector3 b = Unproject(farPos, cam);
	Vector3 c = b - a;

	c.Normalise();

	//std::cout << "Ray Direction:" << c << std::endl;

	return Ray(cam.GetPosition(), c);
}

//http://bookofhook.com/mousepick.pdf
Matrix4 CollisionDetection::GenerateInverseProjection(float aspect, float fov, float nearPlane, float farPlane) {
	Matrix4 m;

	float t = tan(fov*PI_OVER_360);

	float neg_depth = nearPlane - farPlane;

	const float h = 1.0f / t;

	float c = (farPlane + nearPlane) / neg_depth;
	float e = -1.0f;
	float d = 2.0f*(nearPlane*farPlane) / neg_depth;

	m.array[0]  = aspect / h;
	m.array[5]  = tan(fov*PI_OVER_360);

	m.array[10] = 0.0f;
	m.array[11] = 1.0f / d;

	m.array[14] = 1.0f / e;

	m.array[15] = -c / (d*e);

	return m;
}

/*
And here's how we generate an inverse view matrix. It's pretty much
an exact inversion of the BuildViewMatrix function of the Camera class!
*/
Matrix4 CollisionDetection::GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
Matrix4::Translation(position) *
Matrix4::Rotation(yaw, Vector3(0, 1, 0)) *
Matrix4::Rotation(pitch, Vector3(1, 0, 0));

return iview;
}


/*
If you've read through the Deferred Rendering tutorial you should have a pretty
good idea what this function does. It takes a 2D position, such as the mouse
position, and 'unprojects' it, to generate a 3D world space position for it.

Just as we turn a world space position into a clip space position by multiplying
it by the model, view, and projection matrices, we can turn a clip space
position back to a 3D position by multiply it by the INVERSE of the
view projection matrix (the model matrix has already been assumed to have
'transformed' the 2D point). As has been mentioned a few times, inverting a
matrix is not a nice operation, either to understand or code. But! We can cheat
the inversion process again, just like we do when we create a view matrix using
the camera.

So, to form the inverted matrix, we need the aspect and fov used to create the
projection matrix of our scene, and the camera used to form the view matrix.

*/
Vector3	CollisionDetection::UnprojectScreenPosition(Vector3 position, float aspect, float fov, const Camera &c) {
	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(c) * GenerateInverseProjection(aspect, fov, c.GetNearPlane(), c.GetFarPlane());

	Vector2 screenSize = Window::GetWindow()->GetScreenSize();

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(position.x / (float)screenSize.x) * 2.0f - 1.0f,
		(position.y / (float)screenSize.y) * 2.0f - 1.0f,
		(position.z) - 1.0f,
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}

bool CollisionDetection::ObjectIntersection(GameObject* a, GameObject* b, CollisionInfo& collisionInfo) {
	const CollisionVolume* volA = a->GetBoundingVolume();
	const CollisionVolume* volB = b->GetBoundingVolume();

	if (!volA || !volB) {
		return false;
	}

	collisionInfo.a = a;
	collisionInfo.b = b;

	Transform& transformA = a->GetTransform();
	Transform& transformB = b->GetTransform();

	VolumeType pairType = (VolumeType)((int)volA->type | (int)volB->type);

	if (pairType == VolumeType::AABB) {
		return AABBIntersection((AABBVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	}

	if (pairType == VolumeType::Sphere) {
		return SphereIntersection((SphereVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}

	if (pairType == VolumeType::OBB) {
		return OBBIntersection((OBBVolume&)*volA, transformA, (OBBVolume&)*volB, transformB, collisionInfo);
	}

	if (pairType == VolumeType::Capsule) {
		return CapsuleIntersection((CapsuleVolume&)*volA, transformA, (CapsuleVolume&)*volB, transformB, collisionInfo);
	}

	if (volA->type == VolumeType::AABB && volB->type == VolumeType::Sphere) {
		return AABBSphereIntersection((AABBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);//the order of the collisionInfo is important
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBSphereIntersection((AABBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	if (volA->type == VolumeType::AABB && volB->type == VolumeType::Capsule) {
		return AABBCapsuleIntersection((AABBVolume&)*volA, transformA, (CapsuleVolume&)*volB, transformB, collisionInfo);//the order of the collisionInfo is important
	}
	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBCapsuleIntersection((AABBVolume&)*volB, transformB, (CapsuleVolume&)*volA, transformA, collisionInfo);
	}

	if (volA->type == VolumeType::OBB && volB->type == VolumeType::Capsule) {
		return OBBCapsuleIntersection((OBBVolume&)*volA, transformA, (CapsuleVolume&)*volB, transformB, collisionInfo);//the order of the collisionInfo is important
	}
	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::OBB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return OBBCapsuleIntersection((OBBVolume&)*volB, transformB, (CapsuleVolume&)*volA, transformA, collisionInfo);
	}

	if (volA->type == VolumeType::AABB && volB->type == VolumeType::OBB) {
		return AABBOBBIntersection((AABBVolume&)*volA, transformA, (OBBVolume&)*volB, transformB, collisionInfo);//the order of the collisionInfo is important
	}
	if (volA->type == VolumeType::OBB && volB->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBOBBIntersection((AABBVolume&)*volB, transformB, (OBBVolume&)*volA, transformA, collisionInfo);
	}

	if (volA->type == VolumeType::OBB && volB->type == VolumeType::Sphere) {
		return OBBSphereIntersection((OBBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);//the order of the collisionInfo is important
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::OBB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return OBBSphereIntersection((OBBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}


	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::Sphere) {
		return SphereCapsuleIntersection((CapsuleVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::Capsule) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return SphereCapsuleIntersection((CapsuleVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	return false;
}

bool CollisionDetection::AABBTest(const Vector3& posA, const Vector3& posB, const Vector3& halfSizeA, const Vector3& halfSizeB) {

	Vector3 delta = posB - posA;
	Vector3 totalSize = halfSizeA + halfSizeB;

	if (abs(delta.x) < totalSize.x && abs(delta.y) < totalSize.y && abs(delta.z) < totalSize.z) {
		return true;
	}

	return false;
}

//AABB/AABB Collisions
bool CollisionDetection::AABBIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 boxAPos = worldTransformA.GetPosition();
	Vector3 boxBPos = worldTransformB.GetPosition();

	Vector3 boxASize = volumeA.GetHalfDimensions();
	Vector3 boxBSize = volumeB.GetHalfDimensions();

	bool overlap = AABBTest(boxAPos, boxBPos, boxASize, boxBSize);

	if (overlap) {
		static const Vector3 faces[6] =
		{
			Vector3(-1,0,0),Vector3(1,0,0),
			Vector3(0,-1,0),Vector3(0,1,0),
			Vector3(0,0,-1),Vector3(0,0,1),
		};

		Vector3 maxA = boxAPos + boxASize;
		Vector3 minA = boxAPos - boxASize;

		Vector3 maxB = boxBPos + boxBSize;
		Vector3 minB = boxBPos - boxBSize;

		float distances[6] =
		{
			(maxB.x - minA.x),//distance of box 'b' to 'left' of 'a'
			(maxA.x - minB.x),//distance of box 'b' to 'right' of 'a'
			(maxB.y - minA.y),//distance of box 'b' to 'bottom' of 'a'
			(maxA.y - minB.y),//distance of box 'b' to 'top' of 'a'
			(maxB.z - minA.z),//distance of box 'b' to 'far' of 'a'
			(maxA.z - minB.z),//distance of box 'b' to 'near' of 'a'
		};
		float penetration = FLT_MAX;
		Vector3 bestAxis;

		for (int i = 0; i < 6; i++)
		{
			if (distances[i] < penetration) {
				penetration = distances[i];
				bestAxis = faces[i];
			}
		}
		collisionInfo.AddContactPoint(Vector3(), Vector3(), bestAxis, penetration);
		return true;

	}
	return false;
}

//Sphere / Sphere Collision
bool CollisionDetection::SphereIntersection(const SphereVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	float radii   = volumeA.GetRadius() + volumeB.GetRadius();
	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();

	float deltaLength = delta.Length();

	if (deltaLength < radii) {
		float penetration = (radii - deltaLength);
		Vector3 normal    =  delta.Normalised();
		Vector3 localA    =  normal * volumeA.GetRadius();
		Vector3 localB    = -normal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, normal, penetration);
		return true; // we are colliding!
	}

	return false;
}

//AABB - Sphere Collision
bool CollisionDetection::AABBSphereIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 boxSize = volumeA.GetHalfDimensions();

	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();

	Vector3 closestPointOnBox = Maths::Clamp(delta, -boxSize, boxSize);

	Vector3 localPoint = delta - closestPointOnBox;
	float distance     = localPoint.Length();

	if (distance < volumeB.GetRadius()) {//yes,we are colliding！
		Vector3 collisionNormal = localPoint.Normalised();
		float penetration       = (volumeB.GetRadius() - distance);

		Vector3 localA = Vector3();
		Vector3 localB = -collisionNormal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penetration);
		return true;
	}
	return false;
}

bool CollisionDetection::OBBTest(
	const Transform& worldTransformA, const Vector3 A_vertex[], 
	const Transform& worldTransformB, const Vector3 B_vertex[],
	Vector3& min_axis, int& num, Vector3& BestA, Vector3& BestB) {

	Vector3 A_x = worldTransformA.GetOrientation() * Vector3(1, 0, 0);
	Vector3 A_y = worldTransformA.GetOrientation() * Vector3(0, 1, 0);
	Vector3 A_z = worldTransformA.GetOrientation() * Vector3(0, 0, 1);
	Vector3 B_x = worldTransformB.GetOrientation() * Vector3(1, 0, 0);
	Vector3 B_y = worldTransformB.GetOrientation() * Vector3(0, 1, 0);
	Vector3 B_z = worldTransformB.GetOrientation() * Vector3(0, 0, 1);

	Vector3 A_x_B_x = Vector3::Cross(A_x, B_x);
	Vector3 A_x_B_y = Vector3::Cross(A_x, B_y);
	Vector3 A_x_B_z = Vector3::Cross(A_x, B_z);
	Vector3 A_y_B_x = Vector3::Cross(A_y, B_x);
	Vector3 A_y_B_y = Vector3::Cross(A_y, B_y);
	Vector3 A_y_B_z = Vector3::Cross(A_y, B_z);
	Vector3 A_z_B_x = Vector3::Cross(A_z, B_x);
	Vector3 A_z_B_y = Vector3::Cross(A_z, B_y);
	Vector3 A_z_B_z = Vector3::Cross(A_z, B_z);

	Vector3 Axis[15] = { A_x ,A_y,A_z,B_x,B_y,B_z,A_x_B_x,A_x_B_y,A_x_B_z,A_y_B_x,A_y_B_y,A_y_B_z,A_z_B_x,A_z_B_y,A_z_B_z };

	int it = 0;
	float distance = 0.0f;
	float bestDis = FLT_MAX;

	float identity = FLT_MAX;

	float A_max, A_min, B_max, B_min;
	Vector3 AVec_max, AVec_min, BVec_max, BVec_min;

	for (int i = 0; i < 15; ++i) {
		
		A_max = B_max = -FLT_MAX;
		A_min = B_min = FLT_MAX;
		float A_lengthBest;
		float B_lengthBest;

		for (int j = 0; j < 8; ++j) {
			float A_length = Vector3::Dot(Axis[i], A_vertex[j]);
			if (A_length > A_max) {
				A_max = A_length;
				AVec_max = A_vertex[j];
			}
			if (A_length < A_min) {
				A_min = A_length;
				AVec_min = A_vertex[j];
			}
			
			float B_length = Vector3::Dot(Axis[i], B_vertex[j]);
			if (B_length > B_max) {
				B_max = B_length;
				BVec_max = B_vertex[j];
			}
			if (B_length < B_min) {
				B_min = B_length;
				BVec_min = B_vertex[j];
			}
		}
		
		if (A_max < B_min || B_max < A_min) {
			return false;
		}
		else {
			it++;
		}
		
		if (A_max > B_max) {
			distance = B_max - A_min;
		}
		else
		{
			distance = A_max - B_min;
		}

		if (distance < bestDis) { //Update best
			if (A_max > B_max) {
				BestA = AVec_min;
				BestB = BVec_max;
			}
			else {
				BestA = AVec_max;
				BestB = BVec_min;
			}
			bestDis = distance;
			min_axis = Axis[i];
			num = i;
		}
	}
	return true;
}

bool CollisionDetection::OBBIntersection(
	const OBBVolume& volumeA, const Transform& worldTransformA,
	const OBBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 A_x = worldTransformA.GetOrientation() * Vector3(1, 0, 0);
	Vector3 A_y = worldTransformA.GetOrientation() * Vector3(0, 1, 0);
	Vector3 A_z = worldTransformA.GetOrientation() * Vector3(0, 0, 1);
	Vector3 B_x = worldTransformB.GetOrientation() * Vector3(1, 0, 0);
	Vector3 B_y = worldTransformB.GetOrientation() * Vector3(0, 1, 0);
	Vector3 B_z = worldTransformB.GetOrientation() * Vector3(0, 0, 1);

	Vector3 A_Pos = worldTransformA.GetPosition();
	Vector3 B_Pos = worldTransformB.GetPosition();
	Vector3 OBB_A_half = volumeA.GetHalfDimensions();
	Vector3 OBB_B_half = volumeB.GetHalfDimensions();
	

	Vector3 A_vertex1 = A_Pos + worldTransformA.GetOrientation() * Vector3( OBB_A_half.x, OBB_A_half.y, OBB_A_half.z);
	Vector3 A_vertex2 = A_Pos + worldTransformA.GetOrientation() * Vector3(-OBB_A_half.x, OBB_A_half.y, OBB_A_half.z);
	Vector3 A_vertex3 = A_Pos + worldTransformA.GetOrientation() * Vector3( OBB_A_half.x,-OBB_A_half.y, OBB_A_half.z);
	Vector3 A_vertex4 = A_Pos + worldTransformA.GetOrientation() * Vector3(-OBB_A_half.x,-OBB_A_half.y, OBB_A_half.z);
	Vector3 A_vertex5 = A_Pos + worldTransformA.GetOrientation() * Vector3( OBB_A_half.x, OBB_A_half.y,-OBB_A_half.z);
	Vector3 A_vertex6 = A_Pos + worldTransformA.GetOrientation() * Vector3(-OBB_A_half.x, OBB_A_half.y,-OBB_A_half.z);
	Vector3 A_vertex7 = A_Pos + worldTransformA.GetOrientation() * Vector3( OBB_A_half.x,-OBB_A_half.y,-OBB_A_half.z);
	Vector3 A_vertex8 = A_Pos + worldTransformA.GetOrientation() * Vector3(-OBB_A_half.x,-OBB_A_half.y,-OBB_A_half.z);

	Vector3 B_vertex1 = B_Pos + worldTransformB.GetOrientation() * Vector3( OBB_B_half.x, OBB_B_half.y, OBB_B_half.z);
	Vector3 B_vertex2 = B_Pos + worldTransformB.GetOrientation() * Vector3(-OBB_B_half.x, OBB_B_half.y, OBB_B_half.z);
	Vector3 B_vertex3 = B_Pos + worldTransformB.GetOrientation() * Vector3( OBB_B_half.x,-OBB_B_half.y, OBB_B_half.z);
	Vector3 B_vertex4 = B_Pos + worldTransformB.GetOrientation() * Vector3(-OBB_B_half.x,-OBB_B_half.y, OBB_B_half.z);
	Vector3 B_vertex5 = B_Pos + worldTransformB.GetOrientation() * Vector3( OBB_B_half.x, OBB_B_half.y,-OBB_B_half.z);
	Vector3 B_vertex6 = B_Pos + worldTransformB.GetOrientation() * Vector3(-OBB_B_half.x, OBB_B_half.y,-OBB_B_half.z);
	Vector3 B_vertex7 = B_Pos + worldTransformB.GetOrientation() * Vector3( OBB_B_half.x,-OBB_B_half.y,-OBB_B_half.z);
	Vector3 B_vertex8 = B_Pos + worldTransformB.GetOrientation() * Vector3(-OBB_B_half.x,-OBB_B_half.y,-OBB_B_half.z);


	Vector3 A_vertex[8] = { A_vertex1,A_vertex2,A_vertex3,A_vertex4,A_vertex5,A_vertex6,A_vertex7,A_vertex8 };
	Vector3 B_vertex[8] = { B_vertex1,B_vertex2,B_vertex3,B_vertex4,B_vertex5,B_vertex6,B_vertex7,B_vertex8 };


	
	Vector3 A_x_B_x = Vector3::Cross(A_x, B_x);
	Vector3 A_x_B_y = Vector3::Cross(A_x, B_y);
	Vector3 A_x_B_z = Vector3::Cross(A_x, B_z);
	Vector3 A_y_B_x = Vector3::Cross(A_y, B_x);
	Vector3 A_y_B_y = Vector3::Cross(A_y, B_y);
	Vector3 A_y_B_z = Vector3::Cross(A_y, B_z);
	Vector3 A_z_B_x = Vector3::Cross(A_z, B_x);
	Vector3 A_z_B_y = Vector3::Cross(A_z, B_y);
	Vector3 A_z_B_z = Vector3::Cross(A_z, B_z);

	Vector3 Axis[15] = { A_x ,A_y,A_z,B_x,B_y,B_z,A_x_B_x,A_x_B_y,A_x_B_z,A_y_B_x,A_y_B_y,A_y_B_z,A_z_B_x,A_z_B_y,A_z_B_z };


	Vector3 min_axis = Vector3();
	int num = 0;
	Vector3 BestA = Vector3();
	Vector3 BestB = Vector3();

	bool overlap=OBBTest(worldTransformA, A_vertex, worldTransformB, B_vertex,min_axis,num, BestA, BestB);

	if (overlap) {
		std::cout << num << std::endl;
		if (num < 6) {
			float min_depth = FLT_MAX;
			Vector3 normal = Vector3();
			Vector3 localA = Vector3();
			Vector3 localB = Vector3();
			bool a = OBBResolution_PointAndBox(BestA, worldTransformB, volumeB, min_depth, normal, localA, localB);
			if (a) {
				normal = worldTransformB.GetOrientation() * normal;
				localA = worldTransformA.GetOrientation().Conjugate()*(worldTransformB.GetOrientation() * (localA + B_Pos) - A_Pos);
				localB = /*worldTransformB.GetOrientation() **/ localB ;
				collisionInfo.AddContactPoint(localA, localB, normal, min_depth);
			}
			else {
				bool b = OBBResolution_PointAndBox(BestB, worldTransformA, volumeA, min_depth, normal, localB, localA);
				normal = worldTransformA.GetOrientation() * normal;
				localA = /*worldTransformA.GetOrientation() * */localA;
				localB = worldTransformB.GetOrientation().Conjugate()*(worldTransformA.GetOrientation() * (localB + A_Pos) - B_Pos);
				collisionInfo.AddContactPoint(localA, localB, normal, min_depth);
			}
		}
		else if (num >= 6) {
			Vector3 normal = Axis[num];
			Vector3 A_axis[3] = { A_x,A_y,A_z };
			Vector3 A_Dir = A_axis[(int)(num - 6) /3];
			Vector3 B_axis[3] = { B_x,B_y,B_z };
			Vector3 B_Dir = B_axis[(num + 1 - 6) % 3];

			//LineLineNormalProjection_A
			Vector3 normPlane_A = Vector3::Cross(A_Dir, normal).Normalised();
			Vector3 planePt_A = BestB;
			Vector3 dirline_A = A_Dir;
			Vector3 linePt_A = BestA;
			float dotLinePlane_A = (Vector3::Dot(dirline_A, planePt_A));
			if (abs(dotLinePlane_A) < 1e-6) {//parallel
				return false;
			}
			float t_A = (Vector3::Dot((planePt_A - linePt_A), normPlane_A))/ dotLinePlane_A; //Check Edge Parallel
			Vector3 ptIntersect_A = linePt_A + dirline_A * t_A;

			//LineLineNormalProjection_B
			Vector3 normPlane_B = Vector3::Cross(B_Dir, normal);
			Vector3 planePt_B = BestA;
			Vector3 dirline_B = B_Dir;
			Vector3 linePt_B = BestB;
			float dotLinePlane_B = (Vector3::Dot(dirline_B, planePt_B));
			if (abs(dotLinePlane_B) < 1e-6) {//parallel
				return false;
			}
			float t_B = (Vector3::Dot((planePt_B - linePt_B), normPlane_B))/ dotLinePlane_B; //Check Edge Parallel
			Vector3 ptIntersect_B = linePt_B + dirline_B * t_B;

			Vector3 localA = worldTransformA.GetOrientation().Conjugate() * (ptIntersect_A - A_Pos);
			Vector3 localB = worldTransformB.GetOrientation().Conjugate() * (ptIntersect_B - B_Pos);

			if (Vector3::Dot((B_Pos - A_Pos), normal) >= 0) {
				normal = normal;
			}
			else {
				normal = -normal;
			}
			collisionInfo.AddContactPoint(localA, localB, normal, (ptIntersect_A- ptIntersect_B).Length());
		}
		return true;
	 }


	

	return false;
}

//bool CollisionDetection::SphereCapsuleIntersection( // To do blackhole attachment
//	const CapsuleVolume& volumeA, const Transform& worldTransformA,
//	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
//
//	Vector3 sphDir = worldTransformB.GetPosition();
//
//	Vector3 relativePos = worldTransformB.GetPosition() - worldTransformA.GetPosition();
//
//	float halfheight = volumeA.GetHalfHeight();
//	float cap_radius = volumeA.GetRadius();
//	float sph_radius = volumeB.GetRadius();
//
//	Vector3 capDir_y = worldTransformA.GetOrientation() * Vector3(0, 1, 0);
//	Vector3 capDir_x = worldTransformA.GetOrientation() * Vector3(1, 0, 0);
//	Vector3 capDir_z = worldTransformA.GetOrientation() * Vector3(0, 0, 1);
//
//	float distance_y = sqrt((pow(relativePos.Length(), 2)) - pow(Vector3::Dot(relativePos, capDir_y),2));
//	float distance_x = sqrt((pow(relativePos.Length(), 2)) - pow(Vector3::Dot(relativePos, capDir_x),2));
//	float distance_z = sqrt((pow(relativePos.Length(), 2)) - pow(Vector3::Dot(relativePos, capDir_z), 2));
//
//	float delta_y = volumeA.GetRadius() + volumeB.GetRadius();
//	float delta_x = volumeA.GetHalfHeight() + volumeB.GetRadius();
//
//	//colliding
//	if (distance_y < delta_y && distance_x < delta_x && distance_z < delta_x) {
//		float away = Vector3::Dot(capDir_y, (sphDir - worldTransformA.GetPosition()));
//		if (abs(away) < (halfheight-cap_radius)) { // on the cylinder
//			Vector3 cap_center = worldTransformA.GetPosition() - capDir_y * away;
//
//			Vector3 delta = sphDir - cap_center;
//
//			float penetration = (cap_radius + sph_radius) - delta.Length();
//			Vector3 normal = delta.Normalised();
//
//			Vector3 localA = normal * cap_radius ;
//			Vector3 localB = -normal * sph_radius;
//
//			collisionInfo.AddContactPoint(localA, localB, normal, penetration);
//			return true;
//		}
//
//		else if (away >= (halfheight - cap_radius) && away <= halfheight) { // on the side
//		    Vector3 cap_center = worldTransformA.GetPosition() + capDir_y * (halfheight- cap_radius);
//
//		    Vector3 delta = sphDir - cap_center;
//
//		    float penetration = (cap_radius + sph_radius) - delta.Length();
//		    Vector3 normal = delta.Normalised();
//
//	        Vector3 localA = normal * cap_radius;
//		    Vector3 localB = -normal * sph_radius;
//
//			collisionInfo.AddContactPoint(localA, localB, normal, penetration);
//			return true;
//		}
//		else if ((-away) >= (halfheight - cap_radius) && (-away) <= halfheight) {
//			Vector3 cap_center = worldTransformA.GetPosition() - capDir_y * (halfheight - cap_radius);
//
//			Vector3 delta = sphDir - cap_center;
//
//			float penetration = (cap_radius + sph_radius) - delta.Length();
//			Vector3 normal = delta.Normalised();
//
//			Vector3 localA = normal * cap_radius;
//			Vector3 localB = -normal * sph_radius;
//
//			collisionInfo.AddContactPoint(localA, localB, normal, penetration);
//			return true;
//		}
//
//		
//	}
//
//
//
//	return false;
//}

bool CollisionDetection::SphereCapsuleIntersection( // To do
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 sphPos = worldTransformB.GetPosition();
    Vector3 casPos = worldTransformA.GetPosition();
	float halfheight = volumeA.GetHalfHeight();
	float cap_radius = volumeA.GetRadius();

	Vector3 relativePos = sphPos - casPos;

	Vector3 capDir_y = worldTransformA.GetOrientation() * Vector3(0, 1, 0);

	float away = Vector3::Dot(capDir_y, relativePos);
	
	Vector3 Best = Maths::Clamp(casPos + capDir_y * away, casPos - capDir_y * (halfheight - cap_radius), casPos + capDir_y * (halfheight - cap_radius));

	Vector3 delta = sphPos - Best;

	float deltaLength = delta.Length();
	float radii = volumeA.GetRadius() + volumeB.GetRadius();

	if (deltaLength <= radii) {
		float penetration = (radii - deltaLength);
		Vector3 normal = delta.Normalised();

		Vector3 localA = normal * cap_radius + (Best-worldTransformA.GetPosition());//
		Vector3 localB = -normal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, normal, penetration);

		return true;
	}

	return false;

}

//to do OBB with sphere

bool CollisionDetection::OBBResolution_PointAndBox(const Vector3& BestA, const Transform& worldTransformB, const OBBVolume& volumeB, float& min_depth,Vector3& normal,Vector3& localA, Vector3& localB) {//normal是local normal 在外面要乘回来
	Vector3 boxSize = volumeB.GetHalfDimensions();

	Quaternion invoritentation = worldTransformB.GetOrientation().Conjugate();

	Vector3 vertex;

	
	float realtime_depth_x = 0.0f;
	float realtime_depth_y = 0.0f;
	float realtime_depth_z = 0.0f;
	float realtime_min_depth = min_depth;
	Vector3 delta = Vector3();
	Vector3 closestPointOnBox = Vector3();
	

//	for (int i = 0; i < 8; ++i) {
		
		vertex = invoritentation * BestA ;
		

		delta = invoritentation * (BestA - worldTransformB.GetPosition());

		closestPointOnBox = Maths::Clamp(delta, -boxSize, boxSize);

		Vector3 localPoint = delta - closestPointOnBox;

		if (localPoint.Length()==0) {
			realtime_depth_x = boxSize.x - delta.x;
			if (realtime_depth_x < min_depth) {
				min_depth = realtime_depth_x;
				
				normal = Vector3((delta.x < 0 )? -1 : 1, 0, 0);
				localB = Vector3(normal.x * boxSize.x + worldTransformB.GetPosition().x, delta.y, delta.z);
			}

			realtime_depth_y = boxSize.y - delta.y;
			if (realtime_depth_y < min_depth) {
				min_depth = realtime_depth_y;
				normal = Vector3(0, (delta.y < 0) ? -1 : 1, 0);
				localB = Vector3(delta.x, normal.y*boxSize.y+worldTransformB.GetPosition().y, delta.z);
			}

			realtime_depth_z = boxSize.z - delta.z;
			if (realtime_depth_z < min_depth) {
				min_depth = realtime_depth_z;
				normal = Vector3(0, 0, (delta.z < 0) ? -1 : 1);
				localB = Vector3(delta.x, delta.y, normal.z * boxSize.z+worldTransformB.GetPosition().z);
			}

			localA = delta;
			

			return true;

		}
		


		delta = Vector3();
		closestPointOnBox = Vector3();
		

	//}
	return false;
}


bool CollisionDetection::OBBSphereIntersection(const OBBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) 
{
	Vector3 boxSize = volumeA.GetHalfDimensions();

	Quaternion invorientation = worldTransformA.GetOrientation().Conjugate();

	Vector3 delta = invorientation * (worldTransformB.GetPosition() - worldTransformA.GetPosition());

	Vector3 closestPointOnBox = Maths::Clamp(delta, -boxSize, boxSize);

	Vector3 localPoint = delta - closestPointOnBox;
	float distance = localPoint.Length();

	if (distance < volumeB.GetRadius()) {//yes,we are colliding！


		Vector3 collisionNormal = (worldTransformB.GetPosition()- worldTransformA.GetOrientation()*closestPointOnBox- worldTransformA.GetPosition()).Normalised();
		float penetration = (volumeB.GetRadius() - distance);

		Vector3 localA = closestPointOnBox;
		Vector3 localB = -collisionNormal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penetration);
		return true;
	}
	
	
	
	return false;
}


bool CollisionDetection::AABBOBBIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const OBBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	OBBVolume volumeOBBA(volumeA.GetHalfDimensions());
	Transform worldtransformO = worldTransformA;
	worldtransformO.SetOrientation(Quaternion(0,0,0,1));
	bool isCollision = OBBIntersection(volumeOBBA, worldtransformO, volumeB, worldTransformB, collisionInfo);
	if (isCollision) {
	    
		collisionInfo.point.localA = Vector3();
		if (collisionInfo.point.normal==Vector3(0,0,0)) {
			AABBVolume volumeAABBO(volumeB.GetHalfDimensions());
			Transform worldtransformAABB = worldTransformB;
			AABBIntersection(volumeA, worldTransformA, volumeAABBO, worldtransformAABB, collisionInfo);
		}
		
		
		return true;
	}
	//else 
	//{

	//}
	return false;
}		// 
//bunengpingxing

bool CollisionDetection::CapsuleIntersection(const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const CapsuleVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	Vector3 A_Pos = worldTransformA.GetPosition();
	Vector3 A_Dir = worldTransformA.GetOrientation() * Vector3(0, 1, 0);
	Vector3 A_Top = A_Pos + A_Dir * (volumeA.GetHalfHeight() - volumeA.GetRadius());
	Vector3 A_Bot = A_Pos - A_Dir * (volumeA.GetHalfHeight() - volumeA.GetRadius());

	Vector3 B_Pos = worldTransformB.GetPosition();
	Vector3 B_Dir = worldTransformB.GetOrientation() * Vector3(0, 1, 0);
	Vector3 B_Top = B_Pos + B_Dir * (volumeB.GetHalfHeight() - volumeB.GetRadius());
	Vector3 B_Bot = B_Pos - B_Dir * (volumeB.GetHalfHeight() - volumeB.GetRadius());

	Vector3 normal = Vector3::Cross(A_Dir, B_Dir).Normalised();
	Vector3 relative = A_Top - B_Top;
	float distance = Vector3::Dot(relative, normal);

	if (distance < volumeA.GetRadius() + volumeB.GetRadius()) {
		//LineLineNormalProjection_A
		Vector3 normPlane_A = Vector3::Cross(A_Dir, normal).Normalised();
		Vector3 planePt_A = B_Top;
		Vector3 dirline_A = A_Dir;
		Vector3 linePt_A = A_Top;
		float dotLinePlane_A = (Vector3::Dot(dirline_A, planePt_A));
		
		float t_A = (Vector3::Dot((planePt_A - linePt_A), normPlane_A)) / dotLinePlane_A; //Check Edge Parallel
		Vector3 ptIntersect_A = linePt_A + dirline_A * t_A;
		//if ((ptIntersect_A - A_Pos).Length() > (volumeA.GetHalfHeight() - volumeA.GetRadius())) {
			//return false;
		//}

		//LineLineNormalProjection_B
		Vector3 normPlane_B = Vector3::Cross(B_Dir, normal);
		Vector3 planePt_B = A_Top;
		Vector3 dirline_B = B_Dir;
		Vector3 linePt_B = B_Top;
		float dotLinePlane_B = (Vector3::Dot(dirline_B, planePt_B));
		
		float t_B = (Vector3::Dot((planePt_B - linePt_B), normPlane_B)) / dotLinePlane_B; //Check Edge Parallel
		Vector3 ptIntersect_B = linePt_B + dirline_B * t_B;
		//if ((ptIntersect_B - B_Pos).Length() > (volumeB.GetHalfHeight() - volumeB.GetRadius())) {
			//return false;
		//}

		Vector3 collisionNormal = ptIntersect_B - ptIntersect_A;
		Vector3 localA = ptIntersect_A - A_Pos + collisionNormal * volumeA.GetRadius();
		Vector3 localB = ptIntersect_B - B_Pos + collisionNormal * volumeB.GetRadius();
		collisionInfo.AddContactPoint(localA, localB, collisionNormal, volumeA.GetRadius() + volumeB.GetRadius()- distance);
		return true;
	}

	return false;
}

bool CollisionDetection::AABBCapsuleIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const CapsuleVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 A_Pos = worldTransformA.GetPosition();
	Vector3 B_Pos = worldTransformB.GetPosition();
    
	float B_Size = volumeB.GetRadius();

	SphereVolume volumeS(B_Size);
	Transform worldtransformS = worldTransformB;
	bool a = AABBSphereIntersection(volumeA, worldTransformA, volumeS, worldtransformS, collisionInfo);
	if (a) {
		return true;
	}
	else {
		return false;
	}

}
bool CollisionDetection::OBBCapsuleIntersection(const OBBVolume& volumeA, const Transform& worldTransformA,
	const CapsuleVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 A_Pos = worldTransformA.GetPosition();
	Vector3 B_Pos = worldTransformB.GetPosition();

	float B_Size = volumeB.GetRadius();

	SphereVolume volumeS(B_Size);
	Transform worldtransformS = worldTransformB;
	bool a = OBBSphereIntersection(volumeA, worldTransformA, volumeS, worldtransformS, collisionInfo);
	if (a) {
		return true;
	}
	else {
		return false;
	}

}