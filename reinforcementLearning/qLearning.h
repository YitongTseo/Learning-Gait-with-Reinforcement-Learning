#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>

using namespace std;

class qLearningAgent {

protected:
	Environment *environment;
	float ALPHA; //Learning Rate, between 0 and 1, small gives a "longer memory", large changes beliefs more quickly
	float GAMMA; //Discount Factor, between 0 and 1, we probably want it close to 1 to avoid the robot from making short
				 // term gains that jeopardize stability etc...
	float EPSILON; //Probability of randomly selecting an action, larger values lead to more exploration
	float computeValueFromQValues();
	Action computeActionFromQValues();
	float getQValue(const Action& action);

public:
	std::unordered_map<StateAction, float> beliefDict;
	void updateBeliefs(const State& state,
			   const Action& action,
			   const State& nextState,
			   const float reward);

	qLearningAgent(): ALPHA(0.1f), GAMMA(0.9f), EPSILON(0.1f){}

	qLearningAgent(Environment& e, 
				   float alpha = 0.1f, 
				   float gamma = 0.9f, 
				   float epsilon = 0.1f) : ALPHA(alpha), GAMMA(gamma), EPSILON(epsilon) {
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

