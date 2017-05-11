#include "qLearning.h"

Action qLearningAgent::getAction(){
	const std::vector<Action> possibleActions = environment->getPossibleActions();
	// if no possible actions.
	if (possibleActions.size() == 0) {
		//TODO: maybe have to reset.
		return Action(-9999,9999); //should really reset.
	}

	//randomFloat is between 0 and 1
	float randomFloat = float(std::rand()) / float(RAND_MAX);

	if (randomFloat < EPSILON) {
		int randomIndex = std::rand() % possibleActions.size();
		return possibleActions.at(randomIndex); //always use at(): it bound checks, [] does not
	}

	return computeActionFromQValues();
}

//outside thing calls getAction, gets the action.
//Make sure to save the old state.
//now we have state and action
//find the reward
//and the new state. 
//then call update Beliefs with those arguments.
void qLearningAgent::updateBeliefs(const State& state,
								   const Action& action,
					       		   const State& nextState,
				   				   const float reward){
  	float sample = reward + GAMMA * computeValueFromQValues();
  	float oldValue = 0.0f;

	const StateAction stateAction(state, action);
	
	if (beliefDict.count(stateAction) > 0) {
	  oldValue = beliefDict.at(stateAction);
	}
	beliefDict[stateAction] = (1 - ALPHA) * oldValue + ALPHA * sample;
}


Action qLearningAgent::computeActionFromQValues(){

	const std::vector<Action>& actions = environment->getPossibleActions();
	float bestValue = -10000000.0f; //TODO: check if there is negative inf in C++
	//storesindices for all actions with equally best qValues so we can break ties later.
	std::vector<int> bestActionIndices;

	for (int i = 0; i < actions.size(); ++i){
		Action action = actions.at(i);

		float qValue = getQValue(action);

		if (bestValue <= qValue) {
			if (bestValue < qValue) {
				//Just found a new bestValue so time to clear out the inferior indices.
				bestActionIndices.clear();
			}
			//Store all best actions with equal q values then randomly tie break.
			bestActionIndices.push_back(i);
			bestValue = qValue;
		}
	}
	int bestActionIndex = bestActionIndices.at(std::rand() % bestActionIndices.size());
	return actions.at(bestActionIndex);
}

float qLearningAgent::getQValue(const Action& action) {
	State state(environment->getCurrentState());
	StateAction sa(state, action);


	//if beliefDict contains sa then return the value for sa, else 0.0f
	return (beliefDict.count(sa) > 0) ? beliefDict.at(sa) : 0.0f; //TODO: THIS MIGHT BE A BAD IDEA
	//return (beliefDict.count(sa) > 0) ? beliefDict.at(sa) : 0.0f; //TODO: 

}

float qLearningAgent::computeValueFromQValues(){
  const std::vector<Action>& actions = environment->getPossibleActions();

	if (actions.size() == 0) {
		return 0.0f;
	}

	float bestValue = -10000000.0f; //TODO: check if there is negative inf in C++

	for (int i = 0; i < actions.size(); ++i){
		Action action = actions.at(i);
		float qValue = getQValue(action);
		bestValue = std::max(bestValue, qValue);
	}

	return bestValue;
}

Action qLearningAgent::getPolicy() {
	return computeActionFromQValues();
}

float qLearningAgent::getValue() {
	return computeValueFromQValues();
}

qLearningAgent::~qLearningAgent(){}
