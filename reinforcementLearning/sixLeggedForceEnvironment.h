#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <math.h>


class State {
public:

	// [left front elbow, left front shoulder, left front hip, right front elbow, ... ]
	std::vector<float> jointPos; //18 robot joint angles. 3 per leg. (elbow, shoulder, hip)
	std::vector<float> jointVel; //18 robot joint velocities. 3 per leg. (elbow, shoulder, hip)
	std::vector<float> roboOrientation; //Yaw, pitch, roll
	//We will restrict the state space in practice by never visiting some, (i.e. fixing fewer paramters that control all of these)
	int numJoints;

	State(int nm): numJoints(nm){
		roboOrientation.push_back(0.0f);
		roboOrientation.push_back(0.0f);
		roboOrientation.push_back(0.0f);

		for(int i = 0; i < numJoints; ++i ){
			jointPos.push_back(0.0f);
			jointVel.push_back(0.0f);
		}
	}

	//This constructor only takes in two joint angles. sets 3 legs to be the same.
	// State(float leftAngle, float rightAngle, float yaw, float pitch, float roll): FIXEDHIPPOS(0.0f){
	// 	roboOrientation.push_back(yaw);
	// 	roboOrientation.push_back(pitch);
	// 	roboOrientation.push_back(roll);
	//
	// 	for(int i = 0; i<=17; ++i ){
	// 		if( i % 3 == 2){
	// 			//the hip joints are: 2, 5, 8, 11, 14, 17
	// 			jointPos.push_back(FIXEDHIPPOS);
	// 		}
	// 		else if( i % 6 < 2) {
	// 			//for 0, 1, 6, 7, 12, 13
	// 			jointPos.push_back(leftAngle);
	// 		}
	// 		else if( i % 6 > 2) {
	// 			//for 3, 4, 9, 10, 15, 16
	// 			jointPos.push_back(rightAngle);
	// 		}
	//
	// 		jointVel.push_back(0.0f);
	// 	}
	// }

	void updateJoints(std::vector<float> positions) {
		for(int i = 0; i< numJoints; ++i ){
			jointPos.at(i) = positions.at(i);

			// if( i % 3 == 2){
			// 	//the hip joints are: 2, 5, 8, 11, 14, 17
			// 	jointPos.at(i) = 0.0f;
			// }
			// else if (i % 3 == 0) {
			// 	//For now we can only control the knees
			// 	jointPos.at(i) = positions.at(i);
			// } else {
			// 	jointPos.at(i) = 0.0f;
			// }
		}
	}

	void updateVelocities(std::vector<float> velocities) {
		for(int i = 0; i< numJoints; ++i ){
			jointVel.at(i) = 0.0f;//velocities.at(i);
		}
	}

	void updateYPR(float yaw, float pitch, float roll) {
		roboOrientation[0] = yaw;
		roboOrientation[1] = pitch; //TODO: maybe think about making this zero. robo can't fall down this way
		roboOrientation[2] = roll;
	}

	bool operator!=(const State &other) const {
		for(int i = 0; i < other.jointPos.size(); ++i){
			if (jointPos[i] != other.jointPos[i]) {
				return true;
			}
			// if (state.jointVel[i] != other.state.jointVel[i]) {
			// 	return false;
			// }
		}
		return (roboOrientation[0] != other.roboOrientation[0] ||
			    roboOrientation[1] != other.roboOrientation[1] ||
			    roboOrientation[2] != other.roboOrientation[2]);
	 }

	~State() {}
};

class Action {
public:
	//Actions will apply force equal to increment * state.jointForceIncrement to the joint corresponding to jointIndex
	int jointIndex;
	float force;

	Action(float l, int ji): force(l), jointIndex(ji) {}

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
			// if (state.jointVel[i] != other.state.jointVel[i]) {
			// 	return false;
			// }
		}
		return (state.roboOrientation[0] == other.state.roboOrientation[0] &&
				    state.roboOrientation[1] == other.state.roboOrientation[1] &&
				    state.roboOrientation[2] == other.state.roboOrientation[2] &&
	          other.action.force == action.force &&
	          other.action.jointIndex == action.jointIndex);
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
			int size = sa.state.jointPos.size();

			std::size_t h = 0;
		  for(int i = 0; i < size; ++i) {
				h = h ^ (hash<float>()(sa.state.jointPos[i]) << i);
			}


      //note I have no idea if this is a good hash function. Sorry in advance. -Yitong
      return ((hash<float>()(sa.state.roboOrientation[0])
               ^ (hash<float>()(sa.state.roboOrientation[1]) << 1)) >> 1)
  		       ^ ((hash<float>()(sa.state.roboOrientation[2]) << 5) >> 2)
  		       ^ ((hash<float>()(sa.action.force) << 3))
               ^ (hash<int>()(sa.action.jointIndex) << 1)
               ^ h;
    }
  };
}

class Environment {

public:
  virtual State getCurrentState() {return State(-18);}
  virtual std::vector<Action>  getPossibleActions() { return std::vector<Action>(); }
  virtual void doAction(Action& a) {}
  virtual float getReward(State& state, Action& a) {return 12321.0f;}
  virtual void reset() {}
  virtual bool isTerminal(int robotXPos, int robotZPos) {return false;} //Could implement isTerminal wihout making it virtual, but we leave the option open for now
  Environment() {};
  virtual ~Environment() {};
};



//Actions in this class set joint position velocity.
class SixLegsForceEnvironment : public Environment {

protected:
	State state;

	//number of buckets to put joint positions in range [minJointExtension, maxJointExtension]
	int numForceBuckets, numExtensionBuckets;

	//compute this once in the constructor to avoid confusion
	int midExtensionIndex, midForceIndex;

	float maxJointExtension, minJointExtension;
	float maxJointForce, minJointForce;
	std::vector<float> jointExtensionBuckets, forceBuckets;

	int jointIndex;
	float legForce;

public:
	
	float jointForceIncrement;
	//e.g.
	//if maxEst = 1, minExt = -1 and nb = 5:
	// -1   -0.5    0     0.5     1
	//  0     1     2      3      4
	//midJointIndex = 2


	//best to keep the buckets as odd numbers
	SixLegsForceEnvironment(int numJoints = 12, float maxExt = 1.57f, float minExt = -1.57f, float minForce = -25000.0f, float maxForce = 25000.0f, int nbExt = 9, int nbForce = 11) :
																								 state(numJoints),
																								 maxJointExtension(maxExt),
																								 minJointExtension(minExt),
																								 maxJointForce(maxForce),
																								 minJointForce(minForce),
																								 numExtensionBuckets(nbExt),
																								 numForceBuckets(nbForce),
																								 jointIndex(0),
																							     legForce(0.0f){


 	const float jointExtensionIncrement = (maxJointExtension - minJointExtension) / (float(numExtensionBuckets) - 1);
 	float bucket;
 	for (int i = 0; i < numExtensionBuckets; ++i) {
 		bucket = minJointExtension + (i * jointExtensionIncrement);
 		jointExtensionBuckets.push_back(bucket);
 	}

	jointForceIncrement = (maxJointForce - minJointForce) / (float(numForceBuckets) - 1);
	for (int i = 0; i < numForceBuckets; ++i) {
		bucket = minJointForce + (i * jointForceIncrement);
		forceBuckets.push_back(bucket);
	}

 	//start the Force of the wheels at either 0, or slightly positive.
 	midExtensionIndex = numExtensionBuckets / 2;
	std::vector<float> startingJointPositions;
	for (int i = 0; i < state.numJoints; ++i) {
		//we want to start all the joints at the SAME POSITION. Hence calling jointBuckets.
		startingJointPositions.push_back(jointExtensionBuckets.at(midExtensionIndex));
	}
	state.updateJoints(startingJointPositions);

	//TODO: deal with velocities
	// midF
	// std::vector<float> startin
	// for (int i = 0; i < 18; ++i) {
	// 	//we want to start all the joints at the SAME POSITION. Hence calling join
	// 	startingJointPositions.push_back(jointBuckets
	// }
 }

	//returns state not by pointer but by value.
	virtual State getCurrentState() {
		return state;
	}

	void setRobotOrientationYPR(float yaw, float pitch, float roll) {
		//round rv to the nearest 10s decimal place
		//TODO: rounding -2 pi to 2pi like this is sloppy as hell.
		yaw = roundf(yaw * 10.0f) / 10.0f;
		pitch = roundf(pitch * 10.0f) / 10.0f;
		roll = roundf(roll * 10.0f) / 10.0f;

		state.updateYPR(yaw, pitch, roll);
	}

	void setRobotPosition(std::vector<float> positions) {
		std::vector<float> bucketedPositions;

		for(int i = 0; i< state.numJoints; ++i ){

			// if( i % 3 == 2){
			// 	//the hip joints are: 2, 5, 8, 11, 14, 17
			// 	bucketedPositions.push_back(0.0f);
			// }
			// else if (i % 3 == 0) {


				//For now we can only control the knees
				//of all the buckets which one is closest to positions.at(i)

				//TODO: make this not O(N)
				float average = 0.0f;
				//initialize closestFloat to the last possible bucket value. That way if all the checks fall thru it'll be correct.
				float closestFloat = jointExtensionBuckets.at(jointExtensionBuckets.size() - 1);

				for(int j = 1; j < jointExtensionBuckets.size(); ++j) {
					average = (jointExtensionBuckets.at(j - 1) + jointExtensionBuckets.at(j)) / 2.0f;

					if (positions.at(i) < average) {
						// we know that actual must fall between jointExtensionBuckets.at(j - 1) and jointExtensionBuckets.at(j)
						// and that it must be closest to jointExtensionBuckets.at(j - 1)
						closestFloat = jointExtensionBuckets.at(j - 1);
						break;
					}
				}
				bucketedPositions.push_back(closestFloat);

			// } else {
			// 	bucketedPositions.push_back(0.0f);
			// }
		}

		for (int i = 0; i < state.numJoints; ++i) {
			std::cout << "\n bucketedPosition i" << i << ": " << bucketedPositions.at(i);
		}
		state.updateJoints(bucketedPositions);
	}

	void setJointIndexLegForce(int ji, float lf) {
		jointIndex = ji;
		legForce = lf;
	}

	//BE SURE TO SET JOINT INDEX and LEG FORCE before calling this method!!!!!
	virtual std::vector<Action> getPossibleActions() {
		std::vector<Action> actions;

		// if (jointIndex % 2 == 0) {
		// 	std::cout << "ooo, u just passed a knee joint. don't wanna do that.";
		// 	return actions;
		// }

		for (int i = 0; i < forceBuckets.size(); ++i) {
			actions.push_back(Action(forceBuckets.at(i), jointIndex));
		}

		return actions;
	}

	//when reset is called. the robot is gonna have to also be teleported back to position.
	void reset() {
		std::vector<float> empty;
		for (int i = 0; i < state.numJoints; ++i) {
			empty.push_back(0.0f);
		}
		state.updateJoints(empty);
		state.updateVelocities(empty);
		state.updateYPR(0.0f, 0.0f, 0.0f);
	}

	//TODO: have a reward associated with reward that's very shitty.
	bool isTerminal() {
		float pitch = state.roboOrientation[1];
		float roll = state.roboOrientation[2];

		if (std::abs(roll) > 2 || std::abs(pitch) > 2){ //Ashes, ashes, we all fall down...
			return true;
		}
		return false;
	}

	virtual ~SixLegsForceEnvironment() {};

};
