//code written by: Yitong Tseo, David Burt, Zander Majercik
#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <math.h>

//State spaces consists of joint positions and robot orientation
class State {
public:

	//if using a robot from Stompy's family: given an N limbed robot there are 3N robot joint angles (elbow, shoulder, hip)
	// jointPos = [left front elbow, left front shoulder, left front hip, right front elbow, right front shoulder, right front hip, left middle elbow]

	//if using a robot from Crabby/Buggy's family: given an N limbed robot there are 2N robot joint angles (elbow, hip)
	// jointPos = [left front elbow, left front hip, right front elbow, right front hip, left middle elbow]
	std::vector<float> jointPos; 
	std::vector<float> roboOrientation; //[Yaw, pitch, roll]
	//We will restrict the state space in practice by giving some joints smaller buckets, (i.e. fixing fewer paramters that control all of these)
	int numJoints;

	State(int nm): numJoints(nm){

		//Initialize the robot orientation to 0 (default)
		roboOrientation.push_back(0.0f);
		roboOrientation.push_back(0.0f);
		roboOrientation.push_back(0.0f);

		//Initialize all joint positions to 0 (default)
		for(int i = 0; i < numJoints; ++i ){
			jointPos.push_back(0.0f);
		}
	}

	//used to synchronize beliefs about the current joint states with the actual position in Gazebo
	void updateJoints(std::vector<float> positions) {
		for(int i = 0; i< numJoints; ++i ){
			jointPos.at(i) = positions.at(i);
		}
	}

	//Print out current state to help keep track of what the robot is doing
	void print(){
		for(int i = 0; i < jointPos.size(); ++i){
			std::cout << "\n state i"  << i <<  "  joint pos: "<<jointPos[i];
		}
		std::cout << "\n state yaw:" <<roboOrientation[0];
		std::cout << "\n state pitch:" <<roboOrientation[1];
		std::cout << "\n state roll:" <<roboOrientation[2];
	}

	//used to synchronize beliefs about the current orientation with the actual orientation in Gazebo
	void updateYPR(float yaw, float pitch, float roll) {
		roboOrientation[0] = yaw;
		roboOrientation[1] = pitch;
		roboOrientation[2] = roll;
	}


	bool operator!=(const State &other) const {
		for(int i = 0; i < other.jointPos.size(); ++i){
			if (jointPos[i] != other.jointPos[i]) {
				return true;
			}
		}
		return (roboOrientation[0] != other.roboOrientation[0] ||
			    roboOrientation[1] != other.roboOrientation[1] ||
			    roboOrientation[2] != other.roboOrientation[2]);
	 }

	~State() {}
};

//Actions are forces to be applied to each joint
class Action {
public:
	//Actions will apply the specified amount of force to the joint corresponding to jointIndex
	int jointIndex;
	float force;

	Action(float l, int ji): force(l), jointIndex(ji) {}

	~Action() {}
};

//State action pairs are needed as a seperate class for hashing in the dictionary
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
	            other.action.force == action.force &&
	            other.action.jointIndex == action.jointIndex);
	 }

	~StateAction() {}

};

namespace std{

template<>

//need a hash function in order to store StateAction instances in the beliefDict in qLearningAgent
struct hash<StateAction>
  {
    std::size_t operator()(const StateAction& sa) const
    {
      using std::size_t;
      using std::hash;
      /*
		  NOTE: hash function loosely based off of example found here using bit shifting and XOR on individual floats:
	      http://stackoverflow.com/questions/17016175/c-unordered-map-using-a-custom-class-type-as-the-key      */

	  int size = sa.state.jointPos.size();  

	  std::size_t h = 0;
	  for(int i = 0; i < size; ++i) {
	  	h = h ^ (hash<float>()(sa.state.jointPos[i]) << i);
	  }

      return ((hash<float>()(sa.state.roboOrientation[0])
            ^ (hash<float>()(sa.state.roboOrientation[1]) << 1)) >> 1)
  		    ^ ((hash<float>()(sa.state.roboOrientation[2]) << 5) >> 2)
  	        ^ ((hash<float>()(sa.action.force) << 3))
            ^ (hash<int>()(sa.action.jointIndex) << 1)
            ^ h;
    }
  };
}

//the default Environment class initializes everything, default values  are bad to ensure
//we can tell if it is properly overridden
class Environment {

public:
  virtual State getCurrentState() {return State(-18);}
  virtual std::vector<Action>  getPossibleActions() { return std::vector<Action>(); }
  virtual void doAction(Action& a) {}
  virtual float getReward(State& state, Action& a) {return -9999999.0f;}
  virtual void reset() {}
  virtual bool isTerminal(int robotXPos, int robotZPos) {return false;} 
  Environment() {};
  virtual ~Environment() {};
};



class SixLegsForceEnvironment : public Environment {

protected:
	State state;

	//number of buckets to put joint forces in range [minJointForce, maxJointForce] 
	//number of buckets to put positions in range [minJointExtension, maxJointExtension] respectively
	//NOTE: best to keep the numExtensionBuckets and numForceBuckets as odd numbers 
	// so that for and for maxJointExtension == -minJointExtension and minForce=-maxForce
	//the middle index is exactly 0.
	int numForceBuckets, numExtensionBuckets;
	float maxJointExtension, minJointExtension;
	float maxJointForce, minJointForce;

	//we compute jointExtensionBuckets, forceBuckets once in the constructor and then never change them. For bucketing purposes.
	//Every action should only include forces from forceBuckets 
	//and every state should only have joint positions that are in jointExtensionBuckets
	std::vector<float> jointExtensionBuckets, forceBuckets;

	//compute these once in the constructor to avoid confusion
	int midExtensionIndex, midForceIndex;

	//necessary evil global variables which are set via setJointIndexLegForce() so that getPossibleActions() doesn't need to take arguments
	//qLearning.cpp relies on the getPossibleActions() method from all environments to be argumentless.
	int jointIndex;
	float legForce;

public:
	

	//constructor takes the number of joints, the possible extension (which depend on the model),
	// and the number of buckets desired
	SixLegsForceEnvironment(int numJoints = 12, 
						   float maxExt = 1.57f, 
						   float minExt = -1.57f, 
						   float minForce = -10000.0f, 
						   float maxForce = 10000.0f, 
						   int nbExt = 9, 
						   int nbForce = 15) : state(numJoints),
											   maxJointExtension(maxExt),
											   minJointExtension(minExt),
											   maxJointForce(maxForce),
											   minJointForce(minForce),
											   numExtensionBuckets(nbExt),
											   numForceBuckets(nbForce),
											   jointIndex(0),  //NOTE: these will be set by the plugin via setJointIndexLegForce()
										       legForce(0.0f){ //so we only initialize so the C++ compiler won't yell at us.


		//initialize the bucket vectors: jointExtensionBuckets and forceBuckets. They will serve
		//as references whenever we need to set a jointPosition in state or a force value in an action.
		//This way our StateAction space will remain discrete and hopefully small 
		//(that is we avoid considering rounded values as distinct)
	 	const float jointExtensionIncrement = (maxJointExtension - minJointExtension) / (float(numExtensionBuckets) - 1);
	 	float bucket;
	 	for (int i = 0; i < numExtensionBuckets; ++i) {
	 		bucket = minJointExtension + (i * jointExtensionIncrement);
	 		jointExtensionBuckets.push_back(bucket);
	 	}

		const float jointForceIncrement = (maxJointForce - minJointForce) / (float(numForceBuckets) - 1);
		for (int i = 0; i < numForceBuckets; ++i) {
			bucket = minJointForce + (i * jointForceIncrement);
			forceBuckets.push_back(bucket);
		}

	 	//integer division so we start the position of the wheels at either 0, or slightly positive.
	 	midExtensionIndex = numExtensionBuckets / 2;
		std::vector<float> startingJointPositions;
		for (int i = 0; i < state.numJoints; ++i) {
			//we want to start all the joints at the same neutral position
			startingJointPositions.push_back(jointExtensionBuckets.at(midExtensionIndex));
		}
		state.updateJoints(startingJointPositions);
 	}

	//returns state not by pointer but by value.
	virtual State getCurrentState() {
		return state;
	}

	void setRobotOrientationYPR(float yaw, float pitch, float roll) {
		//round each orientation component to the nearest 10s decimal place
		//In this way we round the continuous range [-pi, pi] into ~62 buckets
		//many of these are unlikely to be ever reached (for example extreme values of pitch for stable model)
		yaw = roundf(yaw * 10.0f) / 10.0f;
		pitch = roundf(pitch * 10.0f) / 10.0f;
		roll = roundf(roll * 10.0f) / 10.0f;

		state.updateYPR(yaw, pitch, roll);
	}

	void setRobotPosition(std::vector<float> positions) {
		std::vector<float> bucketedPositions;

		for(int i = 0; i< state.numJoints; ++i ){
			//Have to find which bucket in numExtensionBuckets the continous float positions.at(i) is closest to 
			//Since the number of buckets is small (on the order of 9),
			// we do not bother with binary search as linear search is almost as efficient

			//initialize closestFloat to the last possible bucket value. That way if all the checks fall through it'll be correct.
			float closestFloat = jointExtensionBuckets.at(jointExtensionBuckets.size() - 1);

			//Iterate through the buckets, when we first find a bucket bigger than the position, compare 
			//position to the midpoint this bucket and round based on comparing to this
			for(int j = 1; j < jointExtensionBuckets.size(); ++j) {
				
				if (positions.at(i) < jointExtensionBuckets.at(j)) {
					float average = ( jointExtensionBuckets.at(j - 1)+ jointExtensionBuckets.at(j)) / 2.0f;
					if (positions.at(i) > average){
						closestFloat = jointExtensionBuckets.at(j);
					}
					else {closestFloat = jointExtensionBuckets.at(j-1);}
					break;
				}


			}
			bucketedPositions.push_back(closestFloat);
		}
		state.updateJoints(bucketedPositions);
	}

	//GIVE THIS METHOD THE jointIndex and legForce before calling qLearningAgent.getAction()
	// because getAction() calls getPossibleActions() which depends on these values!
	void setJointIndexLegForce(int ji, float lf) {
		jointIndex = ji;
		legForce = lf;
	}

	//BE SURE TO SET JOINT INDEX and LEG FORCE before calling this method!!!!!
	virtual std::vector<Action> getPossibleActions() {
		std::vector<Action> actions;
		//The amount of force applied to a joint is given by the list of all possible force buckets independent of state
		//Note that this makes more sense than incrementing force, since changes in velocity are a function of force over time
		for (int i = 0; i < forceBuckets.size(); ++i) {
			actions.push_back(Action(forceBuckets.at(i), jointIndex));
		}
		return actions;
	}

	//Resets internal beliefs. When reset is called be sure to teleport the robot back 
	//to starting position with the ModelPlugin and reset joint positions.
	void reset() {
		std::vector<float> empty;
		for (int i = 0; i < state.numJoints; ++i) {
			empty.push_back(0.0f);
		}
		state.updateJoints(empty);
		state.updateVelocities(empty);
		state.updateYPR(0.0f, 0.0f, 0.0f);
	}

	//returns true if the robot has fallen over (and most likely can't get up)
	//If you want the robot to walk then be sure to punish the robot with a large negative reward when isTerminal() == true
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