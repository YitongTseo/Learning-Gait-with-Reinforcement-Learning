#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <algorithm>    // std::max


class qLearningAgent {

protected:
	Environment environment;
	float ALPHA = 0.9f;
	float GAMMA = 0.1f;
	float EPSILON = 0.0f;
	//always have tuples
	unordered_map<tuple<State&, Action&>, float> beliefDict;


public:
	void updateBeliefs(const State& state,
					   const Action$ action,
					   const State& nextState,
					   const float reward) override;

	qLearningAgent(Environment& e) {
		environment = e;
	}

	Action getAction(const State& state) override;

	void qLearningAgent::computeValueFromQValues(const State& state) override;

	Action qLearningAgent::computeActionFromQValues(const State& state) override;

	float qLearningAgent::getQValue(const State& state, const Action& action) override;
	

};
