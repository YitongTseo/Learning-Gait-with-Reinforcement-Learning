#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include "wheeledRobotEnvironment2.h"

using namespace std;

class qLearningAgent {

protected:
	Environment *environment;
	float ALPHA;
	float GAMMA;
	float EPSILON;
	//std::unordered_map<StateAction, float> beliefDict;
	float computeValueFromQValues();
	Action computeActionFromQValues();
	float getQValue(const Action& action);

public:
	//TODO: make this protected again.
	std::unordered_map<StateAction, float> beliefDict;
	void updateBeliefs(const State& state,
			   const Action& action,
			   const State& nextState,
			   const float reward);

	qLearningAgent(): ALPHA(0.9f), GAMMA(0.1f), EPSILON(0.1f){}

	qLearningAgent(Environment& e, 
				   float alpha = 0.9f, 
				   float gamma = 0.1f, 
				   float epsilon = 0.1f) : ALPHA(alpha), GAMMA(gamma), EPSILON(epsilon) {
		//we want to store _the pointer_ to e, not the value of e.
		environment = &e;
	}

	qLearningAgent& operator=( const qLearningAgent& other ) {
     	ALPHA = other.ALPHA;
      	GAMMA = other.GAMMA;
      	EPSILON = other.EPSILON;
      	environment = other.environment;

      	return *this;
  	}

	Action getAction();

	Action getPolicy();

	float getValue();
	
	~qLearningAgent();
};

