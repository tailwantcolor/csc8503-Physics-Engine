#pragma once
#include "..\CSC8503Common\GameObject.h"
#include "../CSC8503Common/NavigationGrid.h"
#include "..\CSC8503Common\GameWorld.h"
#include <vector>

namespace NCL {
	namespace CSC8503 {
		class StateMachine;
		class PathGameObject :public GameObject {
		public:
			PathGameObject(GameWorld* world,GameObject* player);
			~PathGameObject();

			//virtual void Update(float dt);


			//func
			void findPath(Vector3 startPos, Vector3 endPos);
			virtual void Update(float dt, GameObject* player);
		protected:
			
			void MoveAstar(float dt);
			void MoveDirect(float dt);
			//NavigationGrid grid;
			StateMachine* stateMachine;
			float counter;
			std::vector<Vector3> testNodes;

			float distance;
			float speed;
			float total_distance;
			float nodesize;
			Vector3 playerPos;
			GameWorld* world1;
			GameObject* player1;


			NavigationGrid* grid;

			NavigationPath outPath;
			//Ray ray;
		};
	}
}