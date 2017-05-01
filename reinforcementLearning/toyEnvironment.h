#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>
//Header file for the qlearning.cc

//Takes in current state and reward state

class Environment {

public:
	virtual void getCurrentState(State& s);
	virtual void getPossibleActions(State& s);
	virtual void doAction(Action& a);
	virtual float getReward(State& state, Action& a);
	virtual void reset();
	virtual void isTerminal(); //Could implement isTerminal wihout making it virtual, but we leave the option open for now
};

class State {
protected:
	int position;

	State(int pos) {
		position = pos;
	}
};

class Action {
protected:
	int move; // +1, 0, 1

	Action(int m) {
		move = m;
	}
};

class ToyEnvironment : public Environment {

protected:
	//  -------------------------
	// |    |    |    |    |     |
	// |  R |    |  S |    |     |
	//  -------------------------
	//   0     1    2    3    4

	int position;
	const int rewardPosition; 
	State state;

public:


	ToyEnvironment() {
		position = 2;
		rewardPosition = 0;

	}

	//state is old state (before the action is taken)
	float getReward(State& state, Action& a) override {
		if ((state.position + a.move) == 0) {
			return 1;
		}
		return 0;
	}

	State getCurrentState() override {
		return state;
	}

	std::vector<Action>& getPossibleActions() override {
		int position = state.position;
		std::vector<Action> vecOfActions (0);
		if (position < 4) {
			vecOfActions.push_back(1);
		}
		if (position > 0) {
			vecOfActions.push_back(-1);
		}

		return vecOfActions;
	}

	void doAction(Action& a) override {
		state.position += a.move;
	}

	void reset() override {
		state.position = 2;
	}

	bool isTerminal() override {
		int position = state.position;
		if (position == 0) {
			return true;
		}
		return false;
	}

	~ToyEnvironment();

};