#include "PathGameObject.h"
#include "../CSC8503Common/StateTransition.h"
#include "../CSC8503Common/StateMachine.h"
#include "../CSC8503Common/State.h"
#include "../CSC8503Common/NavigationGrid.h"
#include "../CSC8503Common/CollisionDetection.h"
#include <math.h>

using namespace NCL;
using namespace CSC8503;

#define PI   3.14159265358979323846
#define PI_4 0.78539816339744830962

PathGameObject::PathGameObject(GameWorld* world,GameObject* player) {//传一个gameobject进来当作被抓的对象
	counter = 0.0f;
	stateMachine = new StateMachine();
	//NavigationGrid grid("TestGrid1.txt");
	distance = 0.0f;
	total_distance = 0.0f;
	speed = 2.0f;
	nodesize = 0;
	playerPos=player->GetTransform().GetPosition();

    grid = new NavigationGrid("TestGrid1.txt");

	outPath.Clear();

	//RayCollision closestCollision;
	this->world1 = world;
	this->player1 = player;
	findPath(Vector3(80,0,10),Vector3(80,0,80));

	name = "protecter";


	State* stateA = new State([&](float dt)->void 
		{
			this->MoveAstar (dt);
		}
	);

	State* stateB = new State([&](float dt)->void
		{
			this->MoveDirect(dt);
		}
	);

	stateMachine->AddState(stateA);
	stateMachine->AddState(stateB);


	StateTransition* stateAB = new StateTransition(stateA, stateB, [&]()->bool
		{
			RayCollision closestCollision;
			Ray ray(transform.GetPosition() + ((player1->GetTransform().GetPosition() - transform.GetPosition()).Normalised()*0.01) ,/*Vector3(80, 0, 80)- transform.GetPosition()*/(player1->GetTransform().GetPosition()- transform.GetPosition()).Normalised());
			bool is_look = world1->Raycast(ray, closestCollision,true);
			GameObject* selectionObject = (GameObject*)closestCollision.node;
			bool is_player = true;
			if (is_look) {
				 is_player = (selectionObject ==player1);//可以加渲染颜色看现在最近物体是哪个
				 std::cout << closestCollision.node<< " " << is_player << std::endl;	

				 Debug::DrawLine(transform.GetPosition()+Vector3(0,0,0.001), (player1->GetTransform().GetPosition()), Vector4(0, 1, 0, 0));
			}
			selectionObject = nullptr;
			return is_player;
		}
	);

	StateTransition* stateBA = new StateTransition(stateB, stateA, [&]()->bool
		{
			RayCollision closestCollision1;

			Ray ray(transform.GetPosition() + Vector3(0, 0, 0.0001), /*Vector3(80, 0, 80)- transform.GetPosition()*/playerPos - transform.GetPosition());
			bool is_look = world1->Raycast(ray, closestCollision1, true);
			bool is_player1 = true;
			if (is_look) {
				is_player1 = ((GameObject*)closestCollision1.node == player1);//可以加渲染颜色看现在最近物体是哪个
				if (!is_player1) {
					findPath(Vector3(((int)transform.GetPosition().x/10)*10, ((int)transform.GetPosition().y / 10) * 10, ((int)transform.GetPosition().z / 10) * 10), 
						Vector3(80, ((int)transform.GetPosition().y / 10) * 10 ,80));
				}
			}
			
			
			return !is_player1;
		}
	);

	stateMachine->AddTransition(stateAB);
	stateMachine->AddTransition(stateBA);



	//从追赶切回来的状态机是需要重新寻路的
	//state3 重复巡逻



}

PathGameObject::~PathGameObject() {

}

void PathGameObject::Update(float dt, GameObject* player) {
	//findPath();
	playerPos = player->GetTransform().GetPosition();
	stateMachine->Update(dt);

}

void PathGameObject::findPath(Vector3 startPos, Vector3 endPos) {

	testNodes.clear();
	outPath.Clear();
	distance = 0.0f;
	//NavigationGrid grid("TestGrid1.txt");

	//NavigationPath outPath;

	bool found = grid->FindPath(startPos, endPos, outPath);

	Vector3 pos;

	while (outPath.PopWaypoint(pos)) {
		testNodes.push_back(pos);
	}

	total_distance = testNodes.size() * grid->GetnodeSize();//nodesize
	nodesize = grid->GetnodeSize();

	

	/*for (int i = 0; i < testNodes.size()-1; ++i) {
		Vector3 a = testNodes[i + 1];
		Vector3 b = testNodes[i];
		Debug::DrawLine(b, a, Vector4(1, 0, 0, 1));
	}*/
}

void PathGameObject::MoveAstar(float dt) {
	//Quaternion real_orientation = transform.GetOrientation();
	//Vector3 CharacterDir = real_orientation * Vector3(0, 0, -1);


	distance += speed * dt;
	int i = distance / nodesize;
	Vector3 direction = testNodes[i + 1] - testNodes[i];
	Debug::DrawLine(testNodes[i], testNodes[i + 1], Vector4(1, 0, 0, 1));
	Vector3 step = (direction/nodesize) * (speed * dt);
	transform.SetPosition(transform.GetPosition() + step);//在a里写 100%就回归初始点
	
	/*if (CharacterDir != direction) {
		if(Vector3::Dot(direction, CharacterDir)==0)
		transform.SetOrientation(Quaternion(PI_4, PI_4, PI_4, 1));
		else {

		}
	}*/

}

void PathGameObject::MoveDirect(float dt) {

	//physicsObject->SetLinearVelocity(Vector3());
	Vector3 direction = (playerPos - transform.GetPosition()).Normalised();
	physicsObject->AddForce(direction * speed*5);

}