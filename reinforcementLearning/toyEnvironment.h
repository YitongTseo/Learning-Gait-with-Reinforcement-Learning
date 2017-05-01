#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>
//Header file for the qlearning.cc

//Takes in current state and reward state

class State {
public:
	int position;

	State(int pos): position(pos) {
	}

	~State();
};

class Action {
public:
	int move; // +1, 0, 1

	Action(int m): move(m) {
	}

	~Action();
};

class StateAction {
public: 
	State state;
	Action action;

	StateAction(State s, Action a): state(s), action(a) {
	}

	bool operator==(const StateAction &other) const {
	 	return (other.state.position == state.position
	            && other.action.move == action.move);
	 }

	~StateAction();

};

namespace std{

template<>

struct hash<StateAction>
  {
    std::size_t operator()(const StateAction& sa) const
    {
      using std::size_t;
      using std::hash;

      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:

    return ((hash<int>()(sa.action.move)
          ^ (hash<int>()(sa.state.position) << 1)) >> 1);
    }
  };
}

class Environment {

public:
	virtual State getCurrentState();
	virtual std::vector<Action>  getPossibleActions();
	virtual void doAction(Action& a);
	virtual float getReward(State& state, Action& a);
	virtual void reset();
	virtual bool isTerminal(); //Could implement isTerminal wihout making it virtual, but we leave the option open for now
	virtual ~Environment() {}
};


class ToyEnvironment : public Environment {

protected:
	//  -------------------------
	// |    |    |    |    |     |
	// |  R |    |  S |    |     |
	//  -------------------------
	//   0     1    2    3    4

	int rewardPosition; 
	State state;

public:


	ToyEnvironment() : rewardPosition(0), state(2) {
	}

	//state is old state (before the action is taken)
	float getReward(State& state, Action& a) { //override{
		if ((state.position + a.move) == 0) {
			return 1;
		}
		return 0;
	}

	State getCurrentState() {//override {
		return state;
	}

	std::vector<Action> getPossibleActions() {//override {
		int position = state.position;
		std::vector<Action> vecOfActions;
		vecOfActions.push_back(Action(0));
		if (position < 4) {
			vecOfActions.push_back(Action(1));
		}
		if (position > 0) {
			vecOfActions.push_back(Action(-1));
		}

		return vecOfActions;
	}

	void doAction(Action& a) {//override {
		state.position += a.move;
	}

	void reset() {//override {
		state.position = 2;
	}

	bool isTerminal() {//override {
		int position = state.position;
		if (position == 0) {
			return true;
		}
		return false;
	}

	~ToyEnvironment() {};

};
