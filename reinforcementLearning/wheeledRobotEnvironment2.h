#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <math.h>
//Header file for the qlearning.cc

//Takes in current state and reward state

class State {
public:
	float leftWheelVelocity, rightWheelVelocity, robotOrientation;

	State(float l, float r, float roboDir): leftWheelVelocity(l), rightWheelVelocity(r), robotOrientation(roboDir) {}

	~State() {}
};


class Action {
public:
	//these increments are by index. only possible values; +1, 0, -1
	int leftIncrement, rightIncrement;

	Action(int l, int r): leftIncrement(l), rightIncrement(r) {}

	~Action() {}
};

class StateAction {
public: 
	State state;
	Action action;

	StateAction(State s, Action a): state(s), action(a) {
	}

	bool operator==(const StateAction &other) const {
	 	return (other.state.leftWheelVelocity == state.leftWheelVelocity &&
	 			other.state.rightWheelVelocity == state.rightWheelVelocity &&
	 			other.state.robotOrientation == state.robotOrientation &&
	            other.action.leftIncrement == action.leftIncrement &&
	            other.action.rightIncrement == action.rightIncrement);
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

      //note I have no idea if this is a good hash function. Sorry in advance. -Yitong
      return ((hash<float>()(sa.state.leftWheelVelocity)
               ^ (hash<float>()(sa.state.rightWheelVelocity) << 1)) >> 1)
      		   ^ ((hash<float>()(sa.state.robotOrientation) << 5) >> 2)
      		   ^ ((hash<int>()(sa.action.leftIncrement) << 3))
               ^ (hash<int>()(sa.action.rightIncrement) << 1);
    }
  };
}

class Environment {

public:
  virtual State getCurrentState() {return State(-99999.0f, 99999.0f, 91919191.0f);}
  virtual std::vector<Action>  getPossibleActions() { return std::vector<Action>(); }
  virtual void doAction(Action& a) {}
  virtual float getReward(State& state, Action& a) {return 12321.0f;}
  virtual void reset() {}
  virtual bool isTerminal(int robotXPos, int robotZPos) {return false;} //Could implement isTerminal wihout making it virtual, but we leave the option open for now
  Environment() {};
  virtual ~Environment() {};
};


class WheelsEnvironment : public Environment {

protected: 
	State state;
	float maxVelocity;

	//number of buckets to put velocity in range [-maxVelocity, maxVelocity]
	int numBuckets;

	float reward;

	//compute this once in the constructor to avoid confusion
	int midVelocityIndex;

public:
	std::vector<float> wheelBuckets;
	//e.g.
	//if mv = 1 and nb = 5:
	// -1   -0.5    0     0.5     1
	//  0     1     2      3      4
	//midVelocityIndex = 2


	int leftVelocityIndex, rightVelocityIndex;
	WheelsEnvironment(float mv = 1, int nb = 5) : state(0.0f, 0.0f, 0.0f), maxVelocity(mv), numBuckets(nb) {
		const float wheelIncrement = 2.0f * maxVelocity / (float(numBuckets) - 1);
		float bucket;
		for (int i = 0; i < numBuckets; ++i) {
			bucket = -float(maxVelocity) + (i * wheelIncrement);
			wheelBuckets.push_back(bucket);
		}

		//start the velocity of the wheels at either 0, or slightly positive.
		midVelocityIndex = numBuckets / 2;

		leftVelocityIndex = midVelocityIndex;
		rightVelocityIndex = midVelocityIndex;
		state.leftWheelVelocity = wheelBuckets.at(leftVelocityIndex);
		state.rightWheelVelocity = wheelBuckets.at(rightVelocityIndex);
	}

	//state is old state (before the action is taken)
	float getReward(State& state, Action& a) {
		return reward;
	}

	//returns state not by pointer but by value.
	virtual State getCurrentState() {
		return state;
	}

	void setRobotReward(float rv) {
		//round rv to the nearest 10s decimal place
		reward = rv;
	}

	void setRobotOrientation(float ro) {
		//round rv to the nearest 10s decimal place
		//TODO: rounding 0 - 2pi like this is sloppy as hell.
		ro = roundf(ro * 10.0f) / 10.0f;
		state.robotOrientation = ro;
	}

	virtual std::vector<Action> getPossibleActions() {
		std::vector<Action> actions;
		for(int i = -1; i <= 1; ++i) {
			for(int j = -1; j <= 1; ++j) {
				//skip any Action that would take us out of range.
				if (((leftVelocityIndex == 0) && (i == -1)) ||
					((leftVelocityIndex == numBuckets - 1) && (i == 1)) ||
					((rightVelocityIndex == 0) && (j == -1)) ||
					((rightVelocityIndex == numBuckets - 1) && (j == 1))) {
					continue;
				}
				actions.push_back(Action(i, j));
			}			
		}
		return actions;
	}

	void doAction(Action& a) {
		leftVelocityIndex += a.leftIncrement;
		rightVelocityIndex += a.rightIncrement;

		if (leftVelocityIndex >= numBuckets || 
			leftVelocityIndex < 0 ||
			rightVelocityIndex >= numBuckets || 
			rightVelocityIndex < 0) {
		   std::cout << "uh oh!!! our velocity indexes are out of bounds!";
			return;
		}
		state.leftWheelVelocity = wheelBuckets.at(leftVelocityIndex);
		state.rightWheelVelocity = wheelBuckets.at(rightVelocityIndex);
	}

	//when reset is called. the robot is gonna have to also be teleported back to position.
	void reset() {
		leftVelocityIndex = midVelocityIndex;
		rightVelocityIndex = midVelocityIndex;
		state.leftWheelVelocity = wheelBuckets.at(leftVelocityIndex);
		state.rightWheelVelocity = wheelBuckets.at(rightVelocityIndex);
	}

	bool isTerminal(int robotXPos, int robotZPos) {
		//pythagrom theorem!
		float distanceFromCenter = robotXPos * robotXPos + robotZPos * robotZPos;
		float radius = 5.0f;
		return radius > distanceFromCenter;
	}

	virtual ~WheelsEnvironment() {};

};
