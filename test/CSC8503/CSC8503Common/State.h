#pragma once
#include<functional>

namespace NCL {
	namespace CSC8503 {
		typedef std::function<void(float)> StateUpdateFunction;
		class State		{
		public:
			State() {}
			State(StateUpdateFunction someFunc) {
				func = someFunc;
			}
			virtual ~State() {}

			//virtual void Update()=0;
			/*virtual*/ void Update(float dt) {//存疑，调用子类要怎么调用
				if (func != nullptr) {
					func(dt);
				}
			}; //Pure virtual base class
		protected:
			StateUpdateFunction func;
		};

		typedef void(*StateFunc)(void*);//。。。。

		class GenericState : public State		{
		public:
			GenericState(StateFunc someFunc, void* someData) {
				func		= someFunc;
				funcData	= someData;
			}
			virtual void Update() {
				if (funcData != nullptr) {
					func(funcData);
				}
			}
		protected:
			StateFunc func;
			void* funcData;
		};
	}
}