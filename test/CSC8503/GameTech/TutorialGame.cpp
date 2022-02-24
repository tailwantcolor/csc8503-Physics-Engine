#include "TutorialGame.h"
#include "../CSC8503Common/GameWorld.h"
#include "../../Plugins/OpenGLRendering/OGLMesh.h"
#include "../../Plugins/OpenGLRendering/OGLShader.h"
#include "../../Plugins/OpenGLRendering/OGLTexture.h"
#include "../../Common/TextureLoader.h"
#include "../CSC8503Common/PositionConstraint.h"
#include "StateGameObject.h"
#include <math.h>
#include <fstream>
#include "../../Common/Assets.h"
#include "../CSC8503Common/NavigationGrid.h"

#define PI   3.14159265358979323846
#define PI_4 0.78539816339744830962

using namespace NCL;
using namespace CSC8503;

TutorialGame::TutorialGame()	{
	world		= new GameWorld();
	renderer	= new GameTechRenderer(*world);
	physics		= new PhysicsSystem(*world);

	forceMagnitude	= 10.0f;
	useGravity		= false;
	inSelectionMode = false;

	scene_num = 1;
	nodeSize =10;
	gridWidth=10;
	gridHeight=10;

	spendtime = 0.0f;

	testStateObject = nullptr;

	//testPathObject = nullptr;

	Debug::SetRenderer(renderer);

	InitialiseAssets();

	
	


}

/*

Each of the little demo scenarios used in the game uses the same 2 meshes, 
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
void TutorialGame::InitialiseAssets() {
	auto loadFunc = [](const string& name, OGLMesh** into) {
		*into = new OGLMesh(name);
		(*into)->SetPrimitiveType(GeometryPrimitive::Triangles);
		(*into)->UploadToGPU();
	};

	loadFunc("cube.msh"		 , &cubeMesh);
	loadFunc("sphere.msh"	 , &sphereMesh);
	loadFunc("Male1.msh"	 , &charMeshA);
	loadFunc("courier.msh"	 , &charMeshB);
	loadFunc("security.msh"	 , &enemyMesh);
	loadFunc("coin.msh"		 , &bonusMesh);
	loadFunc("capsule.msh"	 , &capsuleMesh);

	basicTex	= (OGLTexture*)TextureLoader::LoadAPITexture("checkerboard.png");
	basicShader = new OGLShader("GameTechVert.glsl", "GameTechFrag.glsl");

	InitCamera();
	//InitWorld();
}

TutorialGame::~TutorialGame()	{
	delete cubeMesh;
	delete sphereMesh;
	delete charMeshA;
	delete charMeshB;
	delete enemyMesh;
	delete bonusMesh;

	delete basicTex;
	delete basicShader;

	delete physics;
	delete renderer;
	delete world;
}

void TutorialGame::UpdateGame(float dt) {
	if (!inSelectionMode) {
		world->GetMainCamera()->UpdateCamera(dt);
	}

	//UpdateKeys();

	if (useGravity) {
		Debug::Print("(G)ravity on", Vector2(5, 95),Vector4(1,1,1,1));
	}
	else {
		Debug::Print("(G)ravity off", Vector2(5, 95), Vector4(1, 1, 1, 1));
	}

	if (testStateObject) {
		testStateObject->Update(dt);
	}

	if (testPathObject) {
		testPathObject->Update(dt, apple1);
	}

	SelectObject();

	MoveSelectedObject();
	physics->Update(dt);
	//增加自由变量
	

	if (lockedObject != nullptr) {
		Vector3 objPos = lockedObject->GetTransform().GetPosition();
		Vector3 camPos = objPos + lockedOffset;

		Matrix4 temp = Matrix4::BuildViewMatrix(camPos, objPos, Vector3(0,1,0));

		Matrix4 modelMat = temp.Inverse();

		Quaternion q(modelMat);
		Vector3 angles = q.ToEuler(); //nearly there now!

		world->GetMainCamera()->SetPosition(camPos);
		world->GetMainCamera()->SetPitch(angles.x);
		world->GetMainCamera()->SetYaw(angles.y);

		//Debug::DrawAxisLines(lockedObject->GetTransform().GetMatrix(), 2.0f);
	}

	world->UpdateWorld(dt);
	renderer->Update(dt);

	Debug::FlushRenderables(dt);
	renderer->Render();

	
}

void TutorialGame::UpdateKeys(int num) {

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F3)) {//chu shi hua zheng ge shi jie respawn
		if (num == 1) {
			physics->start_time = 0;
			physics->end_time = 0;
			InitWorld();
		}
		else if (num == 2) {
			InitDLC();
		}
		else if (num == 3) {
			InitDemo();
		}
	}


	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F1)) {
		InitWorld(); //We can reset the simulation at any time with F1
		selectionObject = nullptr;
		lockedObject	= nullptr;
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F2)) {
		InitCamera(); //F2 will reset the camera to a specific default place
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::G)) {
		useGravity = !useGravity; //Toggle gravity!
		physics->UseGravity(useGravity);
       // if()
	}
	//Running certain physics updates in a consistent order might cause some
	//bias in the calculations - the same objects might keep 'winning' the constraint
	//allowing the other one to stretch too much etc. Shuffling the order so that it
	//is random every frame can help reduce such bias.
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F9)) {
		world->ShuffleConstraints(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F10)) {
		world->ShuffleConstraints(false);
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F7)) {
		world->ShuffleObjects(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F8)) {
		world->ShuffleObjects(false);
	}

	if (lockedObject) {
		LockedObjectMovement();
	}
	else {
		DebugObjectMovement();
	}
}

void TutorialGame::LockedObjectMovement() {
	//world->GetMainCamera()->SetPitch(-90.0f);
	Matrix4 view		= world->GetMainCamera()->BuildViewMatrix();//initcamera()调整到物体正上面
	Matrix4 camWorld	= view.Inverse();

	//std::cout << world->GetMainCamera()->GetPitch() << world->GetMainCamera()->GetYaw() << std::endl;

	Vector3 rightAxis = Vector3(camWorld.GetColumn(0)); //view is inverse of model!

	//forward is more tricky -  camera forward is 'into' the screen...
	//so we can take a guess, and use the cross of straight up, and
	//the right axis, to hopefully get a vector that's good enough!

	Vector3 fwdAxis =Vector3::Cross(Vector3(0, 1, 0), rightAxis);
	fwdAxis.y = 0.0f;
	fwdAxis.Normalise();

	//Vector3 fwdAxis = Vector3(0, -1, 0);

	Vector3 charForward  = lockedObject->GetTransform().GetOrientation() * Vector3(0, 0, 1);
	Vector3 charForward2 = lockedObject->GetTransform().GetOrientation() * Vector3(0, 0, 1);

	float force = 10.0f;

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
		lockedObject->GetPhysicsObject()->AddForce(-rightAxis * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
		Vector3 worldPos = selectionObject->GetTransform().GetPosition();
		lockedObject->GetPhysicsObject()->AddForce(rightAxis * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
		lockedObject->GetPhysicsObject()->AddForce(fwdAxis * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
		lockedObject->GetPhysicsObject()->AddForce(-fwdAxis * force);
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NEXT)) {
		lockedObject->GetPhysicsObject()->AddForce(Vector3(0,-10,0));
	}
}

void TutorialGame::DebugObjectMovement() {
//If we've selected an object, we can manipulate it with some key presses
	if (inSelectionMode && selectionObject) {
		//Twist the selected object!
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(-10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM7)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, 10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM8)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, -10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM5)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
		}
	}

}

void TutorialGame::InitCamera() {
	world->GetMainCamera()->SetNearPlane(0.1f);
	world->GetMainCamera()->SetFarPlane(500.0f);
	world->GetMainCamera()->SetPitch(-90.0f);
	world->GetMainCamera()->SetYaw(0.0f);
	world->GetMainCamera()->SetPosition(Vector3(20, 100, 20));
	lockedObject = nullptr;
}

void TutorialGame::InitWorld() {
	world->ClearAndErase();
	physics->Clear();
	testPathObject = nullptr;

	InitCamera();

	physics->SetInitialise();

	//useGravity = false; //Toggle gravity!
	//physics->UseGravity(useGravity);

	//AddCubeToWorld(Vector3(-110, -2, 0), Vector3(5, 1, 5), 0.0f);
	AddJumpPadToWorld(Vector3(-110, -1, 0), Vector3(20, 1, 5),Quaternion(0, 0, -PI_4/3,1), 0.0f);
	AddSphereToWorld(Vector3(-110, 10, 0), 3.0f);

	//InitMixedGridWorld(5, 5, 3.5f, 3.5f);
	//InitGameExamples();
	InitDefaultFloor();
	//BridgeConstraintTest();

	apple1 = new GameObject();

	SphereVolume* volume = new SphereVolume(0.25f);
	apple1->SetBoundingVolume((CollisionVolume*)volume);
	apple1->GetTransform()
		.SetScale(Vector3(0.25, 0.25, 0.25))
		.SetPosition(Vector3(20,10,20));

	apple1->SetRenderObject(new RenderObject(&apple1->GetTransform(), bonusMesh, nullptr, basicShader));
	apple1->SetPhysicsObject(new PhysicsObject(&apple1->GetTransform(), apple1->GetBoundingVolume()));

	apple1->GetPhysicsObject()->SetInverseMass(1.0f);
	apple1->GetPhysicsObject()->InitSphereInertia();
	//apple1->GetPhysicsObject()->SetLinearVelocity(Vector3(10,0,0));

	world->AddGameObject(apple1);

	AddCapsuleToWorld(Vector3(-27, 2, 0), 10, 4, Quaternion(PI_4, PI_4, PI_4, 1),0.0);

	AddControlAABBCubeToWorld(Vector3(27, -1, -4), Vector3(3, 10, 1));

	ropeConstraintTest();
	
	

	//testPathObject = AddPathObjectToWorld(Vector3(80, 0, 10));
	//AddSphereToWorld(Vector3(20,10,20),1.5f,10.0f);

	physics->start_time = clock();


	//testStateObject = AddStateObjectToWorld(Vector3(0, 10, 0));
}

void TutorialGame::InitDLC() {

	InitCamera();
	world->ClearAndErase();
	physics->Clear();
	testPathObject = nullptr;
	physics->num_kill = 0;
	physics->num_protect = 0;
	physics->SetInitialise();

	//AddCubeToWorld(Vector3(25, 30, 20), Vector3(1, 1, 1), 10.0f);
	//AddCubeToWorld(Vector3(20, 30, 20), Vector3(1, 1, 1), 10.0f);

	AddQueneAABBToWorld("TestGrid1.txt");

	apple1 = new GameObject("player");

	SphereVolume* volume = new SphereVolume(0.5f);
	apple1->SetBoundingVolume((CollisionVolume*)volume);
	apple1->GetTransform()
		.SetScale(Vector3(0.25, 0.25, 0.25))
		.SetPosition(Vector3(80, 0, 80));

	apple1->SetRenderObject(new RenderObject(&apple1->GetTransform(), bonusMesh, nullptr, basicShader));
	apple1->SetPhysicsObject(new PhysicsObject(&apple1->GetTransform(), apple1->GetBoundingVolume()));

	apple1->GetPhysicsObject()->SetInverseMass(1.0f);
	apple1->GetPhysicsObject()->InitSphereInertia();
	//apple1->GetPhysicsObject()->SetLinearVelocity(Vector3(10,0,0));

	world->AddGameObject(apple1);

	testPathObject = AddPathObjectToWorld(Vector3(80, 0, 10),apple1);

	AddMagicFloorToWorld();

	AddCoin1ToWorld(Vector3(50,0,20));
	AddCoin2ToWorld(Vector3(40,0,40));
	AddCoin3ToWorld(Vector3(60, 0, 80));

	physics->start_time = clock();
}

void TutorialGame::InitDemo() {
	InitCamera();
	world->ClearAndErase();
	physics->Clear();
	testPathObject = nullptr;
	physics->num_kill = 0;
	physics->num_protect = 0;
	physics->SetInitialise();

	AddCubeToWorld(Vector3(20, 10, 20), Vector3(1, 1, 1));
	AddCubeToWorld(Vector3(15, 10, 20), Vector3(1, 1, 1));
	AddCapsuleToWorld(Vector3(20, 0, 20), 12, 2, Quaternion(PI_4, PI_4, PI_4, 1),10.0f);
	AddCapsuleToWorld(Vector3(50, 0, 20), 6, 2, Quaternion(0, 0, 0, 1), 10.0f);

}

void TutorialGame::BridgeConstraintTest() {
	Vector3 cubeSize = Vector3(8, 8, 8);

	float invCubeMass  = 5; //how heavy the middle pieces are
	int   numlinks	   =	10;
	float maxDistance  = 30;//constraint distance
	float cubeDistance = 20;//distance between links

	Vector3 startPos = Vector3(50, 100, 50);

	GameObject* start = AddCubeToWorld(startPos + Vector3(0, 0, 0), cubeSize, 0);
	GameObject* end = AddCubeToWorld(startPos + Vector3((numlinks + 2) * cubeDistance, 0, 0), cubeSize, 0);

	GameObject* previous = start;

	for (int i = 0; i < numlinks; ++i) {
		GameObject* block = AddCubeToWorld(startPos + Vector3((i + 1) * cubeDistance, 0, 0), cubeSize, invCubeMass);
		PositionConstraint* constraint = new PositionConstraint(previous, block, maxDistance);
		world->AddConstraint(constraint);
		previous = block;
	}

	PositionConstraint* constraint = new PositionConstraint(previous, end, maxDistance);
	world->AddConstraint(constraint);
}

void TutorialGame::ropeConstraintTest() {
	Vector3 cubeSize = Vector3(4, 4, 4);

	float invCubeMass = 5; //how heavy the middle pieces are
	int   numlinks = 0;
	float maxDistance = 30;//constraint distance
	float cubeDistance = 20;//distance between links

	Vector3 startPos = Vector3(30, 8, 30);

	GameObject* start = AddConstraintCubeToWorld(startPos + Vector3(0, 0, 0), cubeSize,Quaternion(0,0,0,1),0);
	GameObject* end = AddConstraintCubeToWorld(startPos + Vector3((numlinks + 2) * cubeDistance, -5, 0), cubeSize,Quaternion(PI_4, PI_4, PI_4,1));

	GameObject* previous = start;

	for (int i = 0; i < numlinks; ++i) {
		GameObject* block = AddConstraintCubeToWorld(startPos + Vector3((i + 1) * cubeDistance, 0, 0), cubeSize);
		PositionConstraint* constraint = new PositionConstraint(previous, block, maxDistance);
		world->AddConstraint(constraint);
		previous = block;
	}
	end->GetPhysicsObject()->SetLinearVelocity(Vector3(0,-5,30));

	PositionConstraint* constraint = new PositionConstraint(previous, end, maxDistance);
	world->AddConstraint(constraint);

}



/*

A single function to add a large immoveable cube to the bottom of our world

*/


GameObject* TutorialGame::AddFloorToWorld(const Vector3& position) {
	GameObject* floor = new GameObject("floor");

	Vector3 floorSize	= Vector3(100, 2, 100);
	AABBVolume* volume	= new AABBVolume(floorSize);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform()
		.SetScale(floorSize * 2)
		.SetPosition(position);

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, basicTex, basicShader));
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(0);
	floor->GetPhysicsObject()->InitCubeInertia();

	floor->SetCollisionResolveNum(-0.66f);

	world->AddGameObject(floor);

	return floor;
}

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple' 
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius, float inverseMass,bool is_solid) {
	GameObject* sphere = new GameObject("test");

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));//tie tu
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);

	if(is_solid)
	sphere->GetPhysicsObject()->InitSphereInertia();
	else if(!is_solid)
	sphere->GetPhysicsObject()->InitHollowSphereInertia();

	world->AddGameObject(sphere);
 
	return sphere;
}

GameObject* TutorialGame::AddCapsuleToWorld(const Vector3& position, float halfHeight, float radius, Quaternion orientation , float inverseMass) {
	GameObject* capsule = new GameObject("ForwardCapsule");

	CapsuleVolume* volume = new CapsuleVolume(halfHeight, radius);
	capsule->SetBoundingVolume((CollisionVolume*)volume);

	capsule->GetTransform()
		.SetScale(Vector3(radius, halfHeight, radius))
		.SetPosition(position)
		.SetOrientation(orientation);

	capsule->SetRenderObject(new RenderObject(&capsule->GetTransform(), capsuleMesh, basicTex, basicShader));
	capsule->SetPhysicsObject(new PhysicsObject(&capsule->GetTransform(), capsule->GetBoundingVolume()));

	capsule->GetPhysicsObject()->SetInverseMass(inverseMass);
	capsule->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(capsule);

	return capsule;

}

GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	//AABBVolume* volume = new AABBVolume(dimensions);

	OBBVolume* volume = new OBBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddControlAABBCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass ) {
	GameObject* cube = new GameObject("Control");

	AABBVolume* volume = new AABBVolume(dimensions);

	//OBBVolume* volume = new OBBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	cube->GetPhysicsObject()->SetLinearVelocity(Vector3(5, 0, -5));

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddJumpPadToWorld(const Vector3& position, Vector3 dimensions,  Quaternion orientation, float inverseMass) {
	GameObject* cube = new GameObject("JumpPad");

	//AABBVolume* volume = new AABBVolume(dimensions);

	OBBVolume* volume = new OBBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2)
		.SetOrientation(orientation);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}


GameObject* TutorialGame::AddConstraintCubeToWorld(const Vector3& position, Vector3 dimensions, Quaternion orientation, float inverseMass) {
	GameObject* cube = new GameObject("ConstraintCube");

	//AABBVolume* volume = new AABBVolume(dimensions);

	OBBVolume* volume = new OBBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2)
		.SetOrientation(orientation);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(cube);

	return cube;
}

void TutorialGame::InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius) {
	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddSphereToWorld(position, radius, 1.0f);
		}
	}
	AddFloorToWorld(Vector3(0, -2, 0));
}

void TutorialGame::InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing) {
	float sphereRadius = 1.0f;
	Vector3 cubeDims = Vector3(1, 1, 1);

	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);

			if (rand() % 2) {
				AddCubeToWorld(position, cubeDims);

			}
			else if(rand()%3){
				AddSphereToWorld(position, sphereRadius);
			}
			//else {
				//AddSphereToWorld(position, sphereRadius, 10.0f, false);
			//}
		}
	}
}

void TutorialGame::InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims) {
	for (int x = 1; x < numCols+1; ++x) {
		for (int z = 1; z < numRows+1; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddCubeToWorld(position, cubeDims, 1.0f);
		}
	}
}

void TutorialGame::InitDefaultFloor() {
	AddFloorToWorld(Vector3(0, -2, 0));
}

void TutorialGame::InitGameExamples() {
	AddPlayerToWorld(Vector3(0, 5, 0));
	AddEnemyToWorld(Vector3(5, 5, 0));
	AddBonusToWorld(Vector3(10, 5, 0));
}

GameObject* TutorialGame::AddPlayerToWorld(const Vector3& position) {
	float meshSize = 3.0f;
	float inverseMass = 0.5f;

	GameObject* character = new GameObject();

	//AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.85f, 0.3f) * meshSize);
	CapsuleVolume* volume = new CapsuleVolume(3.3f,0.3f);

	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	if (rand() % 2) {
		character->SetRenderObject(new RenderObject(&character->GetTransform(), charMeshA, nullptr, basicShader));
	}
	else {
		character->SetRenderObject(new RenderObject(&character->GetTransform(), charMeshB, nullptr, basicShader));
	}
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(character);

	//lockedObject = character;

	return character;
}

GameObject* TutorialGame::AddEnemyToWorld(const Vector3& position) {
	float meshSize		= 3.0f;
	float inverseMass	= 0.5f;

	GameObject* character = new GameObject();

	//AABBVolume* volume = new AABBVolume(Vector3(0.3f, 0.9f, 0.3f) * meshSize);

	CapsuleVolume* volume = new CapsuleVolume(3.3f, 0.3f);
	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), enemyMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(character);

	return character;
}

GameObject* TutorialGame::AddBonusToWorld(const Vector3& position) {
	GameObject* apple = new GameObject();

	SphereVolume* volume = new SphereVolume(0.25f);
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(0.25, 0.25, 0.25))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), bonusMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);

	return apple;
}

GameObject* TutorialGame::AddCoin1ToWorld(const Vector3& position) {
	GameObject* apple = new GameObject("Coin1");

	SphereVolume* volume = new SphereVolume(0.25f);// larger
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(0.25, 0.25, 0.25))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), bonusMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);

	return apple;
}

GameObject* TutorialGame::AddCoin2ToWorld(const Vector3& position) {
	GameObject* apple = new GameObject("Coin2");

	SphereVolume* volume = new SphereVolume(0.25f);// larger
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(0.25, 0.25, 0.25))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), bonusMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);

	return apple;
}

GameObject* TutorialGame::AddCoin3ToWorld(const Vector3& position) {
	GameObject* apple = new GameObject("Coin3");

	SphereVolume* volume = new SphereVolume(0.25f);// larger
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(0.25, 0.25, 0.25))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), bonusMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);

	return apple;
}


StateGameObject* TutorialGame::AddStateObjectToWorld(const Vector3& position) {
	StateGameObject* apple = new StateGameObject();

	SphereVolume* volume = new SphereVolume(0.25f);
	
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(0.25, 0.25, 0.25))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), bonusMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);

	return apple;
}

PathGameObject* TutorialGame::AddPathObjectToWorld (const Vector3& position,GameObject* player) {




	PathGameObject* apple = new PathGameObject(world,player);

	SphereVolume* volume = new SphereVolume(0.25f);

	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(1, 1, 1))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), sphereMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);

	return apple;
}


/*

Every frame, this code will let you perform a raycast, to see if there's an object
underneath the cursor, and if so 'select it' into a pointer, so that it can be 
manipulated later. Pressing Q will let you toggle between this behaviour and instead
letting you move the camera around. 

*/
bool TutorialGame::SelectObject() {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::Q)) {
		inSelectionMode = !inSelectionMode;
		if (inSelectionMode) {
			Window::GetWindow()->ShowOSPointer(true);
			Window::GetWindow()->LockMouseToWindow(false);
		}
		else {
			Window::GetWindow()->ShowOSPointer(false);
			Window::GetWindow()->LockMouseToWindow(true);
		}
	}
	if (inSelectionMode) {
		renderer->DrawString("Press Q to change to camera mode!", Vector2(5, 85));

		if (Window::GetMouse()->ButtonDown(NCL::MouseButtons::LEFT)) {
			if (selectionObject) {	//set colour to deselected;
				selectionObject->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
				selectionObject = nullptr;
				lockedObject	= nullptr;
			}

			if (forward_selectionObject) {
				//forward_selectionObject->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
				forward_selectionObject = nullptr;
			}

			Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());
			

			RayCollision closestCollision;
			if (world->Raycast(ray, closestCollision, true)) {//default long long int mask for check layers
				selectionObject = (GameObject*)closestCollision.node;
				selectionObject->GetRenderObject()->SetColour(Vector4(0, 1, 0, 1));
				//renderer->DrawLine(ray.GetPosition()+Vector3(1,0,1),selectionObject->GetTransform().GetPosition(),Vector4(0,0,1,0));
				
				Ray forwardRay(selectionObject->GetTransform().GetPosition()+ Vector3(0, 0, -0.00001),Vector3(0,0,-1));//lightPos and volumePos can't be same for the sphere
				if (world->Raycast(forwardRay, closestCollision, true)) {
					forward_selectionObject = (GameObject*)closestCollision.node;
					//forward_selectionObject->GetRenderObject()->SetColour(Vector4(1, 0, 0, 1));
				}

				//if (selectionObject->GetName() == "floor") {
					//selectionObject->SetCollisionResolveNum(10.0f);
				//}
				

				if (selectionObject->GetName() == "JumpPad") {
					selectionObject->SetCollisionResolveNum(15.0f);
				}

				if (selectionObject->GetName() == "ForwardCapsule") {
					selectionObject->SetCollisionResolveNum(20.0f);
				}

				if (selectionObject->GetName() == "Freeze") {
					selectionObject->GetPhysicsObject()->SetFriction(0.2);
				}

				if (selectionObject->GetName() == "Accelerate") {
					selectionObject->GetPhysicsObject()->SetFriction(2.0);
				}

				if (selectionObject->GetName() == "JumpFloor") {
					selectionObject->SetCollisionResolveNum(5.0f);
				}


				if (selectionObject->GetName() == "Empty") {
					world->RemoveGameObject(selectionObject);
				}


				return true;
			}
			else {
				return false;
			}
		}
	}
	else {
		renderer->DrawString("Press Q to change to select mode!", Vector2(5, 85));
	}

	if (lockedObject) {
		renderer->DrawString("Press L to unlock object!", Vector2(5, 80));
	}

	else if(selectionObject){
		renderer->DrawString("Press L to lock selected object object!", Vector2(5, 80));
	}

	if (Window::GetKeyboard()->KeyPressed(NCL::KeyboardKeys::L)) {
		if (selectionObject) {
			if (lockedObject == selectionObject) {
				lockedObject = nullptr;
			}
			else {
				lockedObject = selectionObject;
			}
		}

	}

	return false;
}

/*
If an object has been clicked, it can be pushed with the right mouse button, by an amount
determined by the scroll wheel. In the first tutorial this won't do anything, as we haven't
added linear motion into our physics system. After the second tutorial, objects will move in a straight
line - after the third, they'll be able to twist under torque aswell.
*/
void TutorialGame::MoveSelectedObject() {
	renderer->DrawString("Click Force:" + std::to_string(forceMagnitude), Vector2(10, 20));

	if (!selectionObject) {
		return; // we haven't selected anything
	}
	//Push the selected object!
	if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::RIGHT)) {
		Ray ray = CollisionDetection::BuildRayFromMouse(*world->GetMainCamera());
		//renderer->DrawString("Click Force:" + std::to_string(ray.GetDirection().x) + std::to_string(ray.GetDirection().y) + std::to_string(ray.GetDirection().z), Vector2(10, 5));
		RayCollision closestCollision;
		if (world->Raycast(ray, closestCollision, true)) {
			//if (closestCollision.node == selectionObject) {
			//	/*Debug::DrawLine()*/
			//	selectionObject->GetPhysicsObject()->AddForce(ray.GetDirection() * forceMagnitude);
			//	//selectionObject->GetPhysicsObject()->AddForce(Vector3(0,0,-10));
			//}
			if (closestCollision.node == selectionObject) {
				selectionObject->GetPhysicsObject()->AddForceAtPosition(ray.GetDirection() * forceMagnitude, closestCollision.collidedAt);
			}
		}
	}
	
}


void TutorialGame::AddQueneAABBToWorld(const std::string& filename) {
	std::ifstream infile(Assets::DATADIR + filename);


	infile >> nodeSize;
	infile >> gridWidth;
	infile >> gridHeight;

	GridNode* allNodes = new GridNode[gridWidth * gridHeight];

	for (int y = 0; y < gridHeight; ++y) {
		for (int x = 0; x < gridWidth; ++x) {
			GridNode& n = allNodes[(gridWidth * y) + x];
			char type = 0;
			infile >> type;
			n.type = type;
			
			n.position = Vector3((float)(x * nodeSize), 0, (float)(y * nodeSize));/*+Vector3(100,0,100);*/// + Vector3(30,0,30);//改初始点的坐标
			
			if (n.type == 'x') {
				AddBlockToWorld(n.position, Vector3(nodeSize / 2, nodeSize / 2, nodeSize / 2), 0.0f);
			}
			
		}
	}

}

GameObject* TutorialGame::AddBlockToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject("Block");

	AABBVolume* volume = new AABBVolume(dimensions);

	//OBBVolume* volume = new OBBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	//
	cube->SetLayers(6);

	//cube->SetCollisionResolveNum(1.0f);
	world->AddGameObject(cube);

	return cube;
}

void TutorialGame::AddMagicFloorToWorld() {

	Vector3 position=Vector3();

	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			position = Vector3((float)(i * nodeSize), 0, (float)(j * nodeSize));
			//AddFreezeFloorToWorld(position-Vector3(0,10,0), Vector3(nodeSize / 2, nodeSize / 2, nodeSize / 2), 0.0f);
			int t = rand();
			if (t % 4 == 1) {
				AddFreezeFloorToWorld(position - Vector3(0, 10, 0), Vector3(nodeSize / 2, nodeSize / 2, nodeSize / 2), 0.0f);
			}
			else if(t %4==2){
				AddAccFloorToWorld(position - Vector3(0, 10, 0), Vector3(nodeSize / 2, nodeSize / 2, nodeSize / 2), 0.0f);
			}
			else if(t %4==3) {
				AddJumpFloorToWorld(position - Vector3(0, 10, 0), Vector3(nodeSize / 2, nodeSize / 2, nodeSize / 2), 0.0f);
			}
			else {
				AddEmptyFloorToWorld(position - Vector3(0, 10, 0), Vector3(nodeSize / 2, nodeSize / 2, nodeSize / 2), 0.0f);
			}
		}
	}
}

GameObject* TutorialGame::AddFreezeFloorToWorld(const Vector3& position, Vector3 dimensions, float inverseMass ) {
	GameObject* cube = new GameObject("Freeze");

	AABBVolume* volume = new AABBVolume(dimensions);

	//OBBVolume* volume = new OBBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	//cube->GetPhysicsObject()->SetFriction(0.2);

	cube->SetLayers(2);

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddAccFloorToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject("Accelerate");

	AABBVolume* volume = new AABBVolume(dimensions);

	//OBBVolume* volume = new OBBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	//cube->GetPhysicsObject()->SetFriction(2);

	cube->SetLayers(2);

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddJumpFloorToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject("JumpFloor");

	AABBVolume* volume = new AABBVolume(dimensions);

	//OBBVolume* volume = new OBBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	//cube->GetPhysicsObject()->SetFriction(2);

	cube->SetLayers(4);

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddEmptyFloorToWorld(const Vector3& position, Vector3 dimensions, float inverseMass ) {
	GameObject* cube = new GameObject("Empty");

	AABBVolume* volume = new AABBVolume(dimensions);

	//OBBVolume* volume = new OBBVolume(dimensions);

	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	//cube->GetPhysicsObject()->SetFriction(2);

	cube->SetLayers(3);

	world->AddGameObject(cube);

	return cube;
}