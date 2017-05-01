#include "qLearning.h"

Action qLearningAgent::getAction(const State& state){
	std::vector<Action>& possibleActions = environment.getPossibleActions(state);

	// if no possible actions.
	if (possibleAction.isempty()) {
		//TODO: maybe have to reset.
		return null;
	}

	int r = std::rand(); //returns a large random number
	float f = float(r) / float(std::RAND_MAX);

	if (f < EPSILON) {
		r = std::rand();
		int randomIndex = r % possibleActions.size();
		return vector.at(randomIndex); //always use at(), is bound checked.
	}

	return computeActionFromQValues(state);
}


//outside thing calls getAction, gets the action.
//Make sure to save the old state.
//now we have state and action
//find the reward
//and the new state. 
//then call update Beliefs with those arguments.
void qLearningAgent::updateBeliefs(const State& state,
								   const Action$ action,
								   const State& nextState,
								   const float reward){

	float sample = reward + GAMMA * qLearningAgent::computeValueFromQValues(nextState);
	float oldValue = 0.0f;

	//const tuple<State, Action> stateAction(std::make_tuple(state, action));
	const StateAction stateAction(state, action);
	if (beliefDict.count(stateAction)) {
		oldValue = beliefDict.of(stateAction);
	}

	beliefDict.at(stateAction) = (1 - ALPHA) * oldValue + ALPHA * sample;

}


Action qLearningAgent::computeActionFromQValues(const State& state){

	std::vector<Action>& actions = state->getPossibleActions(state);
	float bestValue = -10000000.0f; //TODO: check if there is negative inf in C++
	int bestActionIndex = null;

	if (actions.isempty()) {
		return null;
	}

	for (int i = 0; i <= actions.size(); ++i){
		Action action = actions.at(i);
		float qValue = qLearningAgent::getQValue(state, action);

		if (bestValue < qValue) {
			//TODO: store all best actions with equal q values.
			//then randomly tie break.
			bestActionIndex = i;
			bestValue = qValue;
		}
	}
	return actions.at(bestActionIndex);
}

float qLearningAgent::getQValue(const State& state, const Action& action) {
	//tuple<State&, Action&> tuple = std::make_tuple(state, action);
	StateAction sa(state, action);

	return beliefDict.at(sa); //using the [ ] is bad. use at() so if tuple isn't in beliefDict it'll throw an execption
}

float qLearningAgent::computeValueFromQValues(const State& state){
	std::vector<Action>& actions = state->getPossibleActions(state);

	if (actions.isempty()) {
		return 0.0f;
	}

	float bestValue = -10000000.0f; //TODO: check if there is negative inf in C++

	for (int i = 0; i <= actions.size(); ++i){
		Action action = actions.at(i);
		float qValue = getQValue(state, action);
		bestValue = std::max(bestValue, qValue)
	}

	return bestValue;
}

Action qLearningAgent::getPolicy(State& state) {
	return computeActionFromQValues(state);
}

float qLearningAgent::getValue(State& state) {
	return computeValueFromQValues(state);
}

~qLearningAgent::qLearningAgent();
