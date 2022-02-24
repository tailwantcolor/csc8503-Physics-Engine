#pragma once
#include "../CSC8503Common/PushdownMachine.h"
#include "../CSC8503Common/PushdownState.h"
#include "../../Common/Window.h"
#include "NormalGameScreen.h"
#include "TutorialGame.h"
#include "DLCScreen.h"
#include "DemoScreen.h"

namespace NCL {
	namespace CSC8503 {
		class MainMenu :public PushdownState {
		public:
			MainMenu(TutorialGame* game)
			{
				this->game = game;
				
			}

			PushdownResult OnUpdate(float dt, PushdownState** newState) override 
			{
				if(Window::GetKeyboard()->KeyPressed(KeyboardKeys::M))
				{
					*newState = new NormalGameScreen(game);
					return PushdownResult::Push;
				}

				if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::Z))
				{
					*newState = new DLCScreen(game);
					return PushdownResult::Push;
				}

				if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::X))
				{
					*newState = new DemoScreen(game);
					return PushdownResult::Push;
				}


				if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::ESCAPE))
				{
					return PushdownResult::Pop;
				}

				Debug::Print("Press M to the game1", Vector2(10, 30));
				Debug::Print("Press Z to the DLC", Vector2(10, 45));
				Debug::Print("Press X to the demo", Vector2(10, 60));

				game->UpdateGame(0.0f);


			}

			void OnAwake() override 
			{
				std::cout << "Welcome to my game!\n";
				//std::cout << "Press Space To Begin or escape to quit!\n";
			}

		protected:
			TutorialGame* game;
		};

        

	}
}