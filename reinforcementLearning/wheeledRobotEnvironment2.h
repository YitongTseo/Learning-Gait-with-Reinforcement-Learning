#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <math.h>


//States include the velocity of both wheels and the direction the robot is facing (in the xy) plane
class State {
public:
	float leftWheelVelocity, rightWheelVelocity, robotOrientation;

	State(float l, float r, float roboDir): leftWheelVelocity(l), rightWheelVelocity(r), robotOrientation(roboDir) {}

	~State() {}
};

//Actions are accelerating or decelerating either wheel by 1 unit, so they are ordered pairs
//A unit is defined by looking at the specified range and dividing it into 
//the given number of buckets
class Action {
public:
	//these increments are by index. only possible values; +1, 0, -1
	int leftIncrement, rightIncrement;

	Action(int l, int r): leftIncrement(l), rightIncrement(r) {}

	~Action() {}
};

//State actions are state action pairs, the class was needed so they could be hashed
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
//Define a hash on state action pairs
struct hash<StateAction>
  {
    std::size_t operator()(const StateAction& sa) const
    {
      using std::size_t;
      using std::hash;

      // hash function uses bitshifting a XORs to combine has function of
      // individual floats appearin the state, action pair
      return ((hash<float>()(sa.state.leftWheelVelocity)
               ^ (hash<float>()(sa.state.rightWheelVelocity) << 1)) >> 1)
      		   ^ ((hash<float>()(sa.state.robotOrientation) << 5) >> 2)
      		   ^ ((hash<int>()(sa.action.leftIncrement) << 3))
               ^ (hash<int>()(sa.action.rightIncrement) << 1);
    }
  };
}

//A generic environement class with methods to be overwritten, state values are intialized to be bad
// so if not overwritten properly it will be clear
class Environment {

public:
  virtual State getCurrentState() {return State(-99999.0f, 99999.0f, 999999.0f);}
  virtual std::vector<Action>  getPossibleActions() { return std::vector<Action>(); }
  virtual void doAction(Action& a) {}
  virtual void reset() {}
  virtual bool isTerminal(int robotXPos, int robotZPos) {return false;} 
  Environment() {};
  virtual ~Environment() {};
};

//The environment class for a 2-wheeled-learner 
class WheelsEnvironment : public Environment {

protected: 
	State state;
	float maxVelocity;

	//number of buckets to divide the velocity range [-maxVelocity, maxVelocity] in to
	int numBuckets;

	//compute this once in the constructor to avoid confusion
	// this should approximately (due to rounding) correspond to both wheels being still
	int midVelocityIndex;

public:
	std::vector<float> wheelBuckets;
	//e.g.
	//if maxVelocity = 1 and nb = 5:
	// -1   -0.5    0     0.5     1   	Velocity
	//  0     1     2      3      4  	Index
	//midVelocityIndex = 2


	int leftVelocityIndex, rightVelocityIndex;

	WheelsEnvironment(float mv = 1, int nb = 5) : state(0.0f, 0.0f, 0.0f), maxVelocity(mv), numBuckets(nb) {
		//The speed to increment by is the range of velocities dided by the number of 
		//buckets desired (minus 1 since it includes both ends) Note minVelocity is defined to be -maxVelocity.
		const float wheelIncrement = 2.0f * maxVelocity / (float(numBuckets) - 1);

		//Define all possible velocities the wheels can take on, it is important to do this 
		//rather than incrementing velocities directly by the increment to avoid rounding errors leading 
		//to the same states appearing different
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

	//returns state not by pointer but by value.
	virtual State getCurrentState() {
		return state;
	}

	void setRobotOrientation(float ro) {
		//round rv to the nearest 10s decimal place
		ro = roundf(ro * 10.0f) / 10.0f;
		state.robotOrientation = ro;
	}

	//returns a list of pairs indicating which wheel can have velocity incremented in the positive direction 
	//and which wheels can be accelerated in the negative direct (0,0) is always a possible action
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

		//This was a check to make sure getPossibleActions was behaving correctly

		// if (leftVelocityIndex >= numBuckets || 
		// 	leftVelocityIndex < 0 ||
		// 	rightVelocityIndex >= numBuckets || 
		// 	rightVelocityIndex < 0) {
		//    std::cout << "uh oh!!! our velocity indexes are out of bounds!";
		// 	return;
		// }
		state.leftWheelVelocity = wheelBuckets.at(leftVelocityIndex);
		state.rightWheelVelocity = wheelBuckets.at(rightVelocityIndex);
	}

	//Neither of these were used in the turn_wheels learner. Since the ground plane is infinite and homogenous
	//the robot position does not effect its states or actions in a meaningful way

	//This resets internal states. When reset is called, the robot is has to be reset in Gazebo as well.
	void reset() {
		leftVelocityIndex = midVelocityIndex;
		rightVelocityIndex = midVelocityIndex;
		state.leftWheelVelocity = wheelBuckets.at(leftVelocityIndex);
		state.rightWheelVelocity = wheelBuckets.at(rightVelocityIndex);
	}

	//Defines a state to be terminal if the robot leaves a 5 meter radius of the initial circle.
	bool isTerminal(int robotXPos, int robotZPos) {
		//pythagorean theorem!
		float distanceFromCenter = robotXPos * robotXPos + robotZPos * robotZPos;
		float radius = 5.0f;
		return radius > distanceFromCenter;
	}

	virtual ~WheelsEnvironment() {};

};
