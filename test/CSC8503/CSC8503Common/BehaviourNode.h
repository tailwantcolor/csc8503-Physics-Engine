#pragma once
#include <string>


namespace NCL {
	namespace CSC8503 {
		enum BehaviourState {
			Initialise,
			Failure,
			Success,
			Ongoing
		};

		class BehaviourNode {
		public:
			BehaviourNode(const std::string& nodeName) {
				currentState = Initialise;
				name = nodeName;
			}

			virtual ~BehaviourNode() {}
			virtual BehaviourState Execute(float dt) = 0;//这是什么返回值

			virtual void Reset() { currentState = Initialise; }

		protected:
			BehaviourState currentState;
			std::string name;
		};
	}
}