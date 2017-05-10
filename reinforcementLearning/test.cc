//#pragma once
#include "sixLeggedForceEnvironment.h"
//#include "qLearning.h"
//#include "qLearning.cpp"

int main() {
  std::cout << "is this working?\n";
  SixLegsForceEnvironment sle;
  State oldState(sle.getCurrentState());

  for(int i = 0; i < oldState.jointPos.size(); ++i ){
     std::cout << "\nprint joint pos i  " << oldState.jointPos.at(i);
  }

  std::vector<float> positions;
  // 0.3925
  for(int i = 0; i < (oldState.jointPos.size() / 2); ++i ){
     positions.push_back(2.0f);
     positions.push_back(0.587f);
  }  

  std::cout<< "\ntime to set those positions! \n\n";
  sle.setRobotPosition(positions);

  sle.setRobotOrientationYPR(0.33f, 1.55f, .23f);

  State newState(sle.getCurrentState());
  for(int i = 0; i < newState.jointPos.size(); ++i ){
     std::cout << "\nprint joint pos " << i << ": " << newState.jointPos.at(i);
  }

  std::cout << "\n yaw: " << newState.roboOrientation[0] << "  pitch: " << newState.roboOrientation[1] <<  "roll: " << newState.roboOrientation[2];

  float legForce = 0.0f;
  int jointIndex = 0;
  std::vector<Action> actions = sle.getPossibleActions(jointIndex, legForce);
  
  for(int i = 0; i < actions.size(); ++i ){
    std::cout << "\naction increment: " << actions.at(i).increment << " jointindex: " << actions.at(i).jointIndex;
  }

  std::cout << "\n isterminal: " << sle.isTerminal() << "\n";

  sle.reset();
  State newestState(sle.getCurrentState());
  for(int i = 0; i < newestState.jointPos.size(); ++i ){
     std::cout << "\nprint joint pos " << i << ": " << newestState.jointPos.at(i);
  }

  std::cout << "\n yaw: " << newestState.roboOrientation[0] << "  pitch: " << newestState.roboOrientation[1] <<  "roll: " << newestState.roboOrientation[2];

  qLearningAgent ql(sle);

  //print out of the beliefDict. for debugging purposes only.
  // for(auto kv : ql.beliefDict) {
  //   StateAction sa = kv.first;
  //   float value = kv.second;
  //   std::cout << "state: " << sa.state.position << " action: " << sa.action.move;
  //   std::cout << " BELIEF:  " << value << "\n";
  // }

  // State newState(sle.getCurrentState());

  // for(int i = 0; i < newState.jointPos.size(); ++i ){
  //   std::cout << "\nnew state i "  << i << " "<< newState.jointPos.at(i);
  // }
}

/*
int main() {
  ToyEnvironment te;
  qLearningAgent ql(te);
  int counter = 100000;

  for (int i = 0; i <= counter; ++i){
    if (te.isTerminal()){
      te.reset();
    }

    //Make sure to save the old state.
    State oldState(te.getCurrentState()); //TODO: make sure this is pass by value.

    //outside thing calls getAction, gets the action.
    Action action = ql.getAction();
    //TODO: we'll likely get an error from trying to assign action to Null
    //if no possible actions and it's not a terminal state. We should probably just reset in those cases.


    //find the reward
    float reward = te.getReward(oldState, action);

    //and the new state.
    te.doAction(action);
    State nextState = te.getCurrentState();

    //then call update Beliefs with those arguments.
    ql.updateBeliefs(oldState, action, nextState, reward);
  }

  //print out of the beliefDict. for debugging purposes only.
  for(auto kv : ql.beliefDict) {
    StateAction sa = kv.first;
    float value = kv.second;
    std::cout << "state: " << sa.state.position << " action: " << sa.action.move;
    std::cout << " BELIEF:  " << value << "\n";
  }
}
*/
