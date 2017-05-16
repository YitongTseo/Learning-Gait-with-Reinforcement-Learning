#include "qLearning.h"

Action qLearningAgent::getAction(){
	const std::vector<Action> possibleActions = environment->getPossibleActions();
	// in all models we consider based on implementations of getPossibleActions, possible actions is nonempty

	//randomFloat is between 0 and 1
	float randomFloat = float(std::rand()) / float(RAND_MAX);

	//Move randomly with probability epsilon
	if (randomFloat < EPSILON) {
		int randomIndex = std::rand() % possibleActions.size();
		return possibleActions.at(randomIndex); 
	}

	return computeActionFromQValues();
}

//plugin calls getAction, gets the action.
//plugin also saves the old state.
//now we have state and action
//find the reward and the next state in the plugin
//then call update Beliefs with those arguments.
void qLearningAgent::updateBeliefs(const State& state,
								   const Action& action,
					       		   const State& nextState,
				   				   const float reward){
	//sample is equal to the reward at the current state plus the discounted value of all future reward beliefs
  	float sample = reward + GAMMA * computeValueFromQValues();
  	//Initialize the old value variable
  	float oldValue = 0.0f;

	const StateAction stateAction(state, action);
	
	//get the old belief value from the belief dictionary
	if (beliefDict.count(stateAction) > 0) {
	  oldValue = beliefDict.at(stateAction);
	}

	//update beliefs accoding to the Bellman update rule with learning parameter ALPHA
	beliefDict[stateAction] = (1 - ALPHA) * oldValue + ALPHA * sample;
}

//Chooses the best action to take from a given states by taking an argmax over possible actions with
//respect to current reward belief
Action qLearningAgent::computeActionFromQValues(){

	const std::vector<Action>& actions = environment->getPossibleActions();

	//This is orders of magnitude than any negative rewards we give, so we can be sure that the best
	//Q-value exceeds this initialization
	float bestValue = -10000000.0f; 

	//stores indices for all actions with equally best qValues so we can break ties later.
	std::vector<int> bestActionIndices;

	//Go through the actions and take an argmax over q-values
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
	//Randomly select an action to return from the list of all possible 
	//best actions determined by the previous argmax
	int bestActionIndex = bestActionIndices.at(std::rand() % bestActionIndices.size());
	return actions.at(bestActionIndex);
}

//Returns the current reward belief for a state action pair
// where the state is the current environment state.

float qLearningAgent::getQValue(const Action& action) {
	State state(environment->getCurrentState());
	StateAction sa(state, action);
	//if beliefDict contains sa then return the value for sa, else 0.0f
	return (beliefDict.count(sa) > 0) ? beliefDict.at(sa) : 0.0f; 

}

//Takes a maximum over all possible Q-values to determine the current belief of a state value.
float qLearningAgent::computeValueFromQValues(){
  const std::vector<Action>& actions = environment->getPossibleActions();

  	//This is orders of magnitude than any negative rewards we give, so we can be sure that the best
	//Q-value exceeds this initialization
	float bestValue = -10000000.0f; 

	//Iterate through possible actions from the current state and find the maximum q-value associated to any of these
	//getPossibleActions is guaranteed to be non-empty
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
