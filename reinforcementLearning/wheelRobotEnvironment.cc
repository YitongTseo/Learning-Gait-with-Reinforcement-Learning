#include "WheeledRobotEnvironment.h"

std::vector<tuple<int, int>>& WheeledRobotEnvironment::getPossibleActions(State& s) override {
	//each wheel can either move up or down by a bucket's worth of velocity.
	std::vector<tuple<int, int>> actions;

	for (int i = -1; i <= 1; ++i){
		for (int j = -1; j <= 1; ++j){
			actions.push_back(tuple(i, j));
		}
	}

	//TODO: comment or maybe restructure.
	if s.velocity() >= maxVelocity { //Here velocity is queried from the state
		actions.erase(2);
		for(int i=5; i<actions.size(); ++i)
			actions.erase(i);
	}
	if s.velocity() <= -maxVelocity {
		actions.erase(7);
		for (int i=3; i>=0; --i)
			actions.erase(i);
	}

	return actions;
}

//returns velocities for left and right wheel.
// std::vector<tuple<float, float>> doAction(Action& a) override {
// 		//Pass to robot or Gazebo.

// 	//returns bucket that pertains
// }



// tuple<float, float> takeStep(tuple<float, float> wheelVelocities, cartPosition, cartVeloctiy) {
// 	//create a state

// 	tupleVector = getPossibleActions(state);

// 	tuple //(0, 1)
// 	return doAction(tuple)

// }

