#pragma once
#include "GameTechRenderer.h"
#include "../CSC8503Common/PhysicsSystem.h"
#include "StateGameObject.h"
#include "PathGameObject.h"


namespace NCL {
	namespace CSC8503 {
		class TutorialGame		{
		public:
			TutorialGame();
			~TutorialGame();

			virtual void UpdateGame(float dt);

			void InitWorld();
			void InitDLC();
			void InitDemo();

			void UpdateKeys(int num);
			
			PhysicsSystem* GetPhysics() {
				return physics;
			}

			GameWorld* GetGameWorld() {
				return world;
			}

			GameObject* selectionObject = nullptr;

			float spendtime;

			int nodeSize;
			int gridWidth;
			int gridHeight;

		protected:
			void InitialiseAssets();

			void InitCamera();
			

			

			void InitGameExamples();

			void InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius);
			void InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing);
			void InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims);
			void InitDefaultFloor();
			void BridgeConstraintTest();
			void ropeConstraintTest();
			
	
			bool SelectObject();
			void MoveSelectedObject();
			void DebugObjectMovement();
			void LockedObjectMovement();

			GameObject* AddFloorToWorld(const Vector3& position);//layer
			GameObject* AddSphereToWorld(const Vector3& position, float radius, float inverseMass = 10.0f, bool is_solid=true);//layer
			GameObject* AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f); //layer
			GameObject* AddJumpPadToWorld(const Vector3& position, Vector3 dimensions,Quaternion orientation = Quaternion(0,0,0,1) ,float inverseMass = 10.0f); //layer
			GameObject* AddBlockToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);
			
			GameObject* AddCapsuleToWorld(const Vector3& position, float halfHeight, float radius, Quaternion orientation = Quaternion(0, 0, 0, 1), float inverseMass = 10.0f); //layers
			GameObject* AddConstraintCubeToWorld(const Vector3& position, Vector3 dimensions, Quaternion orientation = Quaternion(0, 0, 0, 1), float inverseMass = 10.0f);
			GameObject* AddControlAABBCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 10.0f);


			
			

			GameObject* AddPlayerToWorld(const Vector3& position);
			GameObject* AddEnemyToWorld(const Vector3& position);
			GameObject* AddBonusToWorld(const Vector3& position);

			GameObject* AddCoin1ToWorld(const Vector3& position);
			GameObject* AddCoin2ToWorld(const Vector3& position);
			GameObject* AddCoin3ToWorld(const Vector3& position);


			GameObject* AddFreezeFloorToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 0.0f);
			GameObject* AddAccFloorToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 0.0f);
			GameObject* AddJumpFloorToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 0.0f);
			GameObject* AddEmptyFloorToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 0.0f);

			void AddQueneAABBToWorld(const std::string& filename);
			void AddMagicFloorToWorld();


			GameTechRenderer*	renderer;
			PhysicsSystem*		physics;
			GameWorld*			world;

			bool useGravity;
			bool inSelectionMode;

			float		forceMagnitude;

			
			GameObject* forward_selectionObject = nullptr;

			OGLMesh*	capsuleMesh = nullptr;
			OGLMesh*	cubeMesh	= nullptr;
			OGLMesh*	sphereMesh	= nullptr;
			OGLTexture* basicTex	= nullptr;
			OGLShader*	basicShader = nullptr;

			//Coursework Meshes
			OGLMesh*	charMeshA	= nullptr;
			OGLMesh*	charMeshB	= nullptr;
			OGLMesh*	enemyMesh	= nullptr;
			OGLMesh*	bonusMesh	= nullptr;

			//Coursework Additional functionality	
			GameObject* lockedObject	= nullptr;
			Vector3 lockedOffset		= Vector3(0, 100, 20);//选定状态下摄像机位置
			void LockCameraToObject(GameObject* o) {
				lockedObject = o;
			}

			StateGameObject* AddStateObjectToWorld(const Vector3& position);
			StateGameObject* testStateObject;

			PathGameObject* AddPathObjectToWorld(const Vector3& position,GameObject* player);
			PathGameObject* testPathObject;

			GameObject* apple1 = nullptr;

			int scene_num;
		};
	}
}

