#pragma once
#include "../CSC8503Common/GameWorld.h"
#include <set>
#include<ctime>

namespace NCL {
	namespace CSC8503 {
		class PhysicsSystem	{
		public:
			PhysicsSystem(GameWorld& g);
			~PhysicsSystem();

			void Clear();

			void Update(float dt);

			void UseGravity(bool state) {
				applyGravity = state;
			}

			void SetGlobalDamping(float d) {
				globalDamping = d;
			}

			void SetGravity(const Vector3& g);

			bool isPlayerDead = false;
			//bool isPlayerDead2 = false;
			bool isPlayerSuccess = false;
			bool isGameEnd = false;

			bool isProtected = false;
			bool perfectSuccess = false;
			bool TotalselfkillSuccess = false;
			bool NotTotalSuccess = false;
			bool NotTotalSelfKillSuccess = false;

			void SetInitialise() {
				 isPlayerDead = false;
				//bool isPlayerDead2 = false;
				 isPlayerSuccess = false;
				 isGameEnd = false;

				 isProtected = false;
				 perfectSuccess = false;
				 TotalselfkillSuccess = false;
				 NotTotalSuccess = false;
				 NotTotalSelfKillSuccess = false;
			}


			clock_t start_time;
			clock_t end_time;

			int num_protect;
			int num_kill;

		protected:
			void BasicCollisionDetection();
			void BroadPhase();
			void NarrowPhase();

			void ClearForces();

			void IntegrateAccel(float dt);
			void IntegrateVelocity(float dt);

			void UpdateConstraints(float dt);

			void UpdateCollisionList();
			void UpdateObjectAABBs();

			void ImpulseResolveCollision(GameObject& a , GameObject&b, CollisionDetection::ContactPoint& p) /*const*/;
			
			void SimplifiedFrictionImpulseResolveCollision(GameObject& a, GameObject& b, CollisionDetection::ContactPoint& p);

			void ForceResolveCollision(GameObject& a, GameObject& b, CollisionDetection::ContactPoint& p) const;

			GameWorld& gameWorld;

			bool	applyGravity;
			Vector3 gravity;
			float	dTOffset;
			float	globalDamping;

			std::set<CollisionDetection::CollisionInfo> allCollisions;
			std::set<CollisionDetection::CollisionInfo> broadphaseCollisions;

			bool useBroadPhase		= true;
			int numCollisionFrames	= 5;
		};
	}
}

