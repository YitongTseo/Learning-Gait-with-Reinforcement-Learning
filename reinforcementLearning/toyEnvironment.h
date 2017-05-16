#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>
//Header file for the qlearning.cc

//Takes in current state and reward state

//State is just current position
class State {
public:
	int position;

	State(int pos): position(pos) {
	}

	~State() {}
};

//Move Forward, Backward or Stand Still
class Action {
public:
	int move; // +1, 0, 1

	Action(int m): move(m) {
	}

	~Action() {}
};

//Pair of State and action for hash map
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

	~StateAction() {}

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

//A basic environment to be overwritten
class Environment {

public:
  virtual State getCurrentState() {return State(0);}
  virtual std::vector<Action>  getPossibleActions() { return std::vector<Action>(); }
  virtual void doAction(Action& a) {}
  virtual float getReward(State& state, Action& a) {return 0.0f;}
  virtual void reset() {}
  virtual bool isTerminal() {return false;}
  Environment() {};
  virtual ~Environment() {};
};


class ToyEnvironment : public Environment {

protected:

	//  ---------------------------------------
	// |    |    |    |    |    |    |    |    |
	// |-100|-100|-100|-100|-100|-100|-100|-100|
	//  ----------------------------------------
	// |    |    |    |    |    |    |    |    |
	// |  1 |    |  S |    |    |    |    |100000|
	//  ----------------------------------------
	// |    |    |    |    |    |    |    |    |
	// |-100|-100|-100|-100|-100|-100|-100|-100|
	//  ----------------------------------------
	//   0    1    2    3    4    5    6    7

	int rewardPosition; 
	State state;

public:


	ToyEnvironment() : rewardPosition(0), state(2) {
	}

	//state is old state (before the action is taken)
	float getReward(State& state, Action& a) { //override{
		//moving up or down gives a large punishment
		if (a.move == 2 || a.move == -2) {
			return -100;
		}

        //If we reach the left side give a small reward
		if ((state.position + a.move) == 0) {
			return 1;
		}
        
        //If we reach the left side give a large reward

		if ((state.position + a.move) == 7) {
			return 100000;
		}
		return 0;
	}

	//returns state not by pointer but by value.
	virtual State getCurrentState() {
		return state;
	}

    //returns available moves given the current position
	virtual std::vector<Action> getPossibleActions() {
		int position = state.position;
		std::vector<Action> vecOfActions;
		vecOfActions.push_back(Action(0));
		if (position < 7) {
			vecOfActions.push_back(Action(1));
		}
		if (position > 0) {
			vecOfActions.push_back(Action(-1));
		}

		//----Take out if you want to stay in 1D----
		//represents going "up"
		vecOfActions.push_back(Action(2));
		//represents going "down"
		vecOfActions.push_back(Action(-2));

		return vecOfActions;
	}

	void doAction(Action& a) {
	  state.position += a.move;

	  //keep the position within bounds, not strictly necesary.
	  if (state.position > 7) {
	  	state.position = 7;
	  }
	  if (state.position < 0) {
	  	state.position = 0;
	  }

	  //----Take out if you want to stay in 1D----
	  if ((a.move == 2) || (a.move == -2)) {
	  	state.position = -999;
	  }
	}

    //moves agent back to start
	void reset() {
		state.position = 2;
	}

    //returns true if the agent has reached either end or moved up/down
	bool isTerminal() {
		int position = state.position;
		if (state.position == -999 || position == 0 || state.position == 7) {
			return true;
		}
		return false;
	}

	virtual ~ToyEnvironment() {};

};
