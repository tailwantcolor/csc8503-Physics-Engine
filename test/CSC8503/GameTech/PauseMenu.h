#pragma once
#include "../CSC8503Common/PushdownMachine.h"
#include "../CSC8503Common/PushdownState.h"
#include "../../Common/Window.h"
#include "TutorialGame.h"

namespace NCL {
	namespace CSC8503 {
		class PauseMenu :public PushdownState {
		public:
			PauseMenu(TutorialGame* game) {
				this->game = game;
			}

			PushdownResult OnUpdate(float dt, PushdownState** newState) override {
				if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::U)) 
				{
					return PushdownResult::Pop;
				}

				game->UpdateGame(0.0f);
				return PushdownResult::NoChange;
			}

			void OnAwake() override 
			{
				std::cout << "Press U to unpause game!\n";
			}

		protected:
			TutorialGame* game;
		};
	}
}