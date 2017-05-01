#include <iostream>
#include <unordered_map>
#include <vector>
#include <algorithm>    // std::max

using namespace std;

class qLearningAgent {

protected:
	Environment environment;
	const float ALPHA;
	const float GAMMA;
	const float EPSILON;
	std::unordered_map<StateAction, float> beliefDict;


public:
	void updateBeliefs(const State& state,
					   const Action& action,
					   const State& nextState,
					   const float reward);

	qLearningAgent(Environment& e): ALPHA(0.9f), GAMMA(0.1f), EPSILON(0.0f) {
		environment = e;
	}

	Action getAction(const State& state);

	void computeValueFromQValues(const State& state);

	Action computeActionFromQValues(const State& state);

	float getQValue(const State& state, const Action& action);
	
	~qLearningAgent();
};

