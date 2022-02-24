#pragma once
#include "../CSC8503Common/PushdownMachine.h"
#include "../CSC8503Common/PushdownState.h"
#include "../../Common/Window.h"
#include "PauseMenu.h"
#include "TutorialGame.h"
#include <ctime>

namespace NCL{
	namespace CSC8503 {
		class DLCScreen : public PushdownState {
		public:
			DLCScreen(TutorialGame* game) {
				this->game = game;
				game->InitDLC();
				start = clock();
			}

			PushdownResult OnUpdate(float dt, PushdownState** newState) override 
			{
				if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::P)) {
					*newState = new PauseMenu(game);
					return PushdownResult::Push;
				}

				if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F1)) {
					std::cout << "Returning to main menu!\n";
					game->GetGameWorld()->Clear();
					game->GetPhysics()->Clear();
					game->selectionObject = nullptr;
					return PushdownResult::Pop;
				}

				game->UpdateKeys(2);

				//if (!game->GetPhysics()->isGameEnd) {
					//Debug::Print("spend time is " + std::to_string((game->GetPhysics()->end_time - game->GetPhysics()->start_time) / 1000) + "seconds", Vector2(30, 30), Vector4(0, 0, 1, 1));
					
				//}
				//else {
					//game->UpdateGame(0.0f);
				//}
					if (game->GetPhysics()->isProtected) {
						Debug::Print("AH,HAHA,you have dead...Maybe? ", Vector2(10, 50));
						Debug::Print("Press Escape to try again", Vector2(10, 30));
						//std::cout << game->GetPhysics()->num_kill << " " << game->GetPhysics()->num_protect << std::endl;
						Debug::Print("Total Score " + std::to_string(900 - 300 * game->GetPhysics()->num_kill - 150 * game->GetPhysics()->num_protect), Vector2(10,70));
						game->UpdateGame(0.0f);
			    	}
					else if (game->GetPhysics()->perfectSuccess && !game->GetPhysics()->TotalselfkillSuccess && !game->GetPhysics()->NotTotalSuccess && !game->GetPhysics()->NotTotalSelfKillSuccess) {
						Debug::Print("Perfect Success!", Vector2(10, 50));
						Debug::Print("You're so cute to find the perfect end", Vector2(10, 30));
						Debug::Print("There're 4 different ends", Vector2(10, 40));
						game->UpdateGame(dt);
					}
					else if (!game->GetPhysics()->perfectSuccess && game->GetPhysics()->TotalselfkillSuccess && !game->GetPhysics()->NotTotalSuccess && !game->GetPhysics()->NotTotalSelfKillSuccess) {
						Debug::Print("TotalselfkillSuccess!", Vector2(10, 50));
						game->UpdateGame(dt);
					}
					else if (!game->GetPhysics()->perfectSuccess && !game->GetPhysics()->TotalselfkillSuccess && game->GetPhysics()->NotTotalSuccess && !game->GetPhysics()->NotTotalSelfKillSuccess) {
						Debug::Print("Success! But not total", Vector2(10, 30));
						Debug::Print("Your Score is " + std::to_string(900 - 300 * game->GetPhysics()->num_kill - 150 * game->GetPhysics()->num_protect) + ", which is less than 900", Vector2(10, 50));
						Debug::Print("So the reason is why, maybe you should to try in different ways", Vector2(10, 60));
						game->UpdateGame(dt);
					}
					else if (!game->GetPhysics()->perfectSuccess && !game->GetPhysics()->TotalselfkillSuccess && !game->GetPhysics()->NotTotalSuccess && game->GetPhysics()->NotTotalSelfKillSuccess) {
						Debug::Print("NotTotalSelfKillSuccess!", Vector2(10, 50));
						game->UpdateGame(dt);
					}
					else {
						Debug::Print("Total Score " + std::to_string(900 - 300 * game->GetPhysics()->num_kill - 150 * game->GetPhysics()->num_protect), Vector2(10,70));
						Debug::Print("If you need some help", Vector2(10, 30));
						Debug::Print("you can get some information in the help txt", Vector2(10, 40));
						Debug::Print("But maybe lose some fun and Riddle", Vector2(10, 50));
						game->UpdateGame(dt);
					}
				

				return PushdownResult::NoChange;
			}

			void OnAwake() override
			{

			}

		protected:
			TutorialGame* game;
			clock_t start;
			clock_t end;
		};
	}
}