#pragma once
#include "../CSC8503Common/PushdownState.h"
#include"../CSC8503Common/PushdownMachine.h"
#include "../../Common/Window.h"
#include "PauseMenu.h"
#include "TutorialGame.h"

#include <ctime>

namespace NCL {
	namespace CSC8503 {
		class NormalGameScreen :public PushdownState {
		public:
			NormalGameScreen(TutorialGame* game)
			{
				this->game = game;
				game->InitWorld();
				start = clock();
			}

			PushdownResult OnUpdate(float dt, PushdownState** newState) override {
				if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::P))
				{
					*newState = new PauseMenu(game);
					return PushdownResult::Push;
				}

				if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F1))
				{
					std::cout << "Returning to main menu!\n";
					game->GetGameWorld()->Clear();
					game->GetPhysics()->Clear();
					game->selectionObject = nullptr;
					return PushdownResult::Pop;
				}
				game->UpdateKeys(1);
				
				if (game->GetPhysics()->isPlayerDead) {
					Debug::Print("AH,HAHA,you have dead ", Vector2(10, 30));
					Debug::Print("Press Escape to try again", Vector2(10, 50));
					game->UpdateGame(0.0f);
				}
				else if(game->GetPhysics()->isPlayerSuccess){
					Debug::Print("Success, Congraulation!", Vector2(50, 50));
					//game->UpdateGame(0.0f);
					game->UpdateGame(dt);
				}
				else if (game->GetPhysics()->isGameEnd) {
					Debug::Print("Time is out", Vector2(5, 50));
					Debug::Print("Please Press F3 to try again", Vector2(5, 70));
					//game->UpdateGame(0.0f);
					game->UpdateGame(dt);
				}
				else if (!game->GetPhysics()->isPlayerSuccess && ! game->GetPhysics()->isPlayerDead) {
					//Debug::Print("Please Press ESCAPE to quit and try again", Vector2(5, 70));
					Debug::Print("spend time is " + std::to_string((game->GetPhysics()->end_time - game->GetPhysics()->start_time) / 1000) + "seconds", Vector2(30, 30), Vector4(0, 0, 1, 1));
					Debug::Print("You can press P to pause and U to unpause", Vector2(20, 70));
					game->UpdateGame(dt);
				}

				
				
				
				
				return PushdownResult::NoChange;
			}

			~NormalGameScreen() {}

			void OnAwake() override {}

		protected:
			clock_t start;
			clock_t end;//µ•Œª «∫¡√Î
			TutorialGame* game;

		};



	}
}