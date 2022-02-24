#pragma once
#include "BehaviourNodeWithChildren.h"

namespace NCL {
	namespace CSC8503 {
		class BehaviourSequence :public BehaviourNodeWithChildren {
		public:
			BehaviourSequence(const std::string& nodeName):BehaviourNodeWithChildren(nodeName){}
			~BehaviourSequence(){}

			BehaviourState Execute(float dt)override {
				//std::cout << "Executing sequence " << name << "\n";
				for (auto& i : childNodes) {
					BehaviourState nodeState = i->Execute(dt);
					switch (nodeState) 
					{
					case Success:continue;
					case Failure:
					case Ongoing: 
					     {
							currentState = nodeState;
							return nodeState;//return nodestate 和 cuurentstate有区别吗

					     }
					}
				}
				return Success;
			}
		};
	}
}