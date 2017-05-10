#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <math.h>


class State {
public:

	// [left front elbow, left front shoulder, left front hip, right front elbow, ... ]
	std::vector<float> jointPos; //18 robot joint angles. 3 per leg. (elbow, shoulder, hip)
	std::vector<float> roboOrientation; //Yaw, pitch, roll
	//We will restrict the state space in practice by never visiting some, (i.e. fixing fewer paramters that control all of these)
	const float FIXEDHIPPOS;

	State(): FIXEDHIPPOS(0.0f){
		roboOrientation.push_back(0.0f);
		roboOrientation.push_back(0.0f);
		roboOrientation.push_back(0.0f);

		for(int i = 0; i < 18; ++i ){
			jointPos.push_back(0.0f);
		}
	}

	//This constructor only takes in two joint angles. sets 3 legs to be the same.
	State(float leftAngle, float rightAngle, float yaw, float pitch, float roll): FIXEDHIPPOS(0.0f){
		roboOrientation.push_back(yaw);
		roboOrientation.push_back(pitch);
		roboOrientation.push_back(roll);

		for(int i = 0; i<=17; ++i ){
			if( i % 3 == 2){
				//the hip joints are: 2, 5, 8, 11, 14, 17
				jointPos.push_back(FIXEDHIPPOS);
			}
			else if( i % 6 < 2) {
				//for 0, 1, 6, 7, 12, 13
				jointPos.push_back(leftAngle);
			}
			else if( i % 6 > 2) {
				//for 3, 4, 9, 10, 15, 16
				jointPos.push_back(rightAngle);
			}
		}
	}

	void updateJoints(float leftAngle, float rightAngle) {
		for(int i = 0; i<=17; ++i ){
			if( i % 3 == 2){
				//the hip joints are: 2, 5, 8, 11, 14, 17
				jointPos.at(i) = FIXEDHIPPOS;
			}
			else if( i % 6 < 2) {
				//for 0, 1, 6, 7, 12, 13
				jointPos.at(i) = leftAngle;
			}
			else if( i % 6 > 2) {
				//for 3, 4, 9, 10, 15, 16
				jointPos.at(i) = rightAngle;
			}
		}
	}

	void updateYPR(float yaw, float pitch, float roll) {
		roboOrientation[0] = yaw;
		roboOrientation[1] = pitch;
		roboOrientation[2] = roll;
	}

	~State() {}
};

class Action {
public:
	//Actions will teleport the left and right joints to the positions corresponding to
	//SixLegsEnvironment.jointBuckets[leftJIndex] and SixLegsEnvironment.jointBuckets[rightJIndex] respectively
	int leftJIndex, rightJIndex;

	Action(int l, int r): leftJIndex(l), rightJIndex(r) {}

	~Action() {}
};

class StateAction {
public:
	State state;
	Action action;

	StateAction(State s, Action a): state(s), action(a) {
	}

	bool operator==(const StateAction &other) const {
		for(int i = 0; i < other.state.jointPos.size(); ++i){
			if (state.jointPos[i] != other.state.jointPos[i]) {
				return false;
			}
		}
		return (state.roboOrientation[0] == other.state.roboOrientation[0] &&
				    state.roboOrientation[1] == other.state.roboOrientation[1] &&
				    state.roboOrientation[2] == other.state.roboOrientation[2] &&
	          other.action.leftJIndex == action.leftJIndex &&
	          other.action.rightJIndex == action.rightJIndex);
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

			std::size_t h = 0;
		  for(int i = 0; i < sa.state.jointPos.size(); ++i) {
				h = h ^ (hash<float>()(sa.state.jointPos[i]) << i);
			}

      //note I have no idea if this is a good hash function. Sorry in advance. -Yitong
      return ((hash<float>()(sa.state.roboOrientation[0])
               ^ (hash<float>()(sa.state.roboOrientation[1]) << 1)) >> 1)
      		     ^ ((hash<float>()(sa.state.roboOrientation[2]) << 5) >> 2)
      		     ^ ((hash<int>()(sa.action.leftJIndex) << 3))
               ^ (hash<int>()(sa.action.rightJIndex) << 1)
               ^ h;
    }
  };
}

class Environment {

public:
  virtual State getCurrentState() {return State(-99999.0f, 99999.0f, 2222, 3333, 4444);}
  virtual std::vector<Action>  getPossibleActions() { return std::vector<Action>(); }
  virtual void doAction(Action& a) {}
  virtual float getReward(State& state, Action& a) {return 12321.0f;}
  virtual void reset() {}
  virtual bool isTerminal(int robotXPos, int robotZPos) {return false;} //Could implement isTerminal wihout making it virtual, but we leave the option open for now
  Environment() {};
  virtual ~Environment() {};
};


//Actions in this class teleport limb positions.
class SixTeleportLegsEnvironment : public Environment {

protected:
	State state;
	float maxJointExtension;
	float minJointExtension;

	//number of buckets to put velocity in range [-maxVelocity, maxVelocity]
	int numBuckets;
	// float reward;

	//compute this once in the constructor to avoid confusion
	int midJointIndex;

public:
	std::vector<float> jointBuckets;
	//e.g.
	//if maxEst = 1, minExt = -1 and nb = 5:
	// -1   -0.5    0     0.5     1
	//  0     1     2      3      4
	//midJointIndex = 2


	int curLeftJointIndex, curRightJointIndex;


	SixTeleportLegsEnvironment(float maxExt = 1.0f, float minExt = -1.0f, int nb = 5) :
																								 state(0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
																								 maxJointExtension(maxExt),
																								 minJointExtension(minExt),
																								 numBuckets(nb) {


 	const float jointIncrement = (maxJointExtension - minJointExtension) / (float(numBuckets) - 1);
 	float bucket;
 	for (int i = 0; i < numBuckets; ++i) {
 		bucket = minJointExtension + (i * jointIncrement);
 		jointBuckets.push_back(bucket);
 	}

 	//start the velocity of the wheels at either 0, or slightly positive.
 	midJointIndex = numBuckets / 2;

 	curLeftJointIndex = midJointIndex;
 	curRightJointIndex = midJointIndex;
 	state.updateJoints(jointBuckets.at(curLeftJointIndex), //left angle position
 	 						       jointBuckets.at(curRightJointIndex)); //right angle position

 	state.updateYPR(0, 0, 0);
 }

	//returns state not by pointer but by value.
	virtual State getCurrentState() {
		return state;
	}

	void setRobotOrientation(float yaw, float pitch, float roll) {
		//round rv to the nearest 10s decimal place
		//TODO: rounding -2 pi to 2pi like this is sloppy as hell.
		yaw = roundf(yaw * 10.0f) / 10.0f;
		pitch = roundf(pitch * 10.0f) / 10.0f;
		roll = roundf(roll * 10.0f) / 10.0f;

		state.updateYPR(yaw, pitch, roll);
	}

	virtual std::vector<Action> getPossibleActions() {
		std::vector<Action> actions;
		//only consider actions that change the joint angles by one bucket up, no buckets, or one bucket down.
		for(int i = -1; i <= 1; ++i) {
			for(int j = -1; j <= 1; ++j) {
				//skip any Action that would take us out of range.
				if (((curLeftJointIndex == 0) && (i == -1)) ||
					  ((curLeftJointIndex == numBuckets - 1) && (i == 1)) ||
					  ((curRightJointIndex == 0) && (j == -1)) ||
					  ((curRightJointIndex == numBuckets - 1) && (j == 1))) {
					continue;
				}
				//push an action with left and right indices modified by i and j respecitvely.
				actions.push_back(Action(curLeftJointIndex + i, curRightJointIndex + j));
			}
		}
		return actions;
	}

	void doAction(Action& a) {
		curLeftJointIndex = a.leftJIndex;
		curRightJointIndex = a.rightJIndex;

		if (curLeftJointIndex >= numBuckets ||
				curLeftJointIndex < 0 ||
				curRightJointIndex >= numBuckets ||
				curRightJointIndex < 0) {
		   std::cout << "uh oh!!! our velocity indexes are out of bounds!";
			return;
		}

		state.updateJoints(jointBuckets.at(curLeftJointIndex), //left angle position
											 jointBuckets.at(curRightJointIndex)); //right angle position
	}

	//when reset is called. the robot is gonna have to also be teleported back to position.
	void reset() {
		curLeftJointIndex = midJointIndex;
		curRightJointIndex = midJointIndex;
		state.updateJoints(jointBuckets.at(curLeftJointIndex), //left angle position
											 jointBuckets.at(curRightJointIndex)); //right angle position
		state.updateYPR(0, 0, 0);
	}

	bool isTerminal(int robotXPos, int robotZPos) {
		//pythagrom theorem!
		float distanceFromCenter = robotXPos * robotXPos + robotZPos * robotZPos;
		float radius = 5.0f;
		return radius > distanceFromCenter;
	}

	virtual ~SixTeleportLegsEnvironment() {};

};
