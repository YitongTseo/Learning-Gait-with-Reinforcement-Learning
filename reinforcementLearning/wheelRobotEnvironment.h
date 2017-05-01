#include <iostream>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <ignition/math.h>
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



class WheeledRobotEnvironment : public Environment {

protected:
	WheeledRobot robot;
	const int nWheelStates = 0; //Placeholder
	const int maxVelocity = 0; //Placeholder
	const float wheelIncrement = 2 * maxVelocity / nWheelStates;
	std::vector<float> wheelBuckets;

	State s;

public:

	WheeledRobotEnvironment(WheeledRobot& r) : robot(r) {
		float bucket = -float(maxVelocity);
		while (bucket < float(maxVelocity)){
			wheelBuckets.push_back(bucket);
			bucket += wheelIncrement;
		}
		reset();
	}

	void getCurrentState(State& s) override {
		return s;
	}

	std::vector<tuple<int, int>>& getPossibleActions(State& s) override;

	void doAction(Action& a) override {
		//Pass to robot or Gazebo.
	}
	void reset() override {
		robot.reset() //Robot reset.
	};
	void isTerminal() override {
		//Call Gazebo for terminal states.
	}

	~WheeledRobotEnvironment();


};