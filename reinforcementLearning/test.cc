#include "test.h"

void test::run() {
	toyEnvironment te();
	qLearning ql(te);
	int counter = 10;

	for (int i = 0; i <= counter; ++i){
		if (te.isTerminal()){
			cout << "reset!\n";
			te.reset();
		}
		//outside thing calls getAction, gets the action.
		Action a = ql.getAction();

		//Make sure to save the old state.
		State oldState = te.getCurrentState(); //TODO: make sure this is pass by value.

		//now we have state and action

		//find the reward
		float reward = te.getReward(oldState, action);

		//and the new state. 
		te.doAction(a);
		float nextState = te.getCurrentState();

		cout << "nextState: " << nextState << "oldState: " << oldState << "action: " << action <<" make sure these are not pass by reference \n";

		// cout << "beliefs: " << ql.belief

		//then call update Beliefs with those arguments.
		ql.updateBeliefs(oldState, action, nextState, reward);
	}
}