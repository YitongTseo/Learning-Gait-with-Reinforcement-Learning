
#include "sixLeggedForceEnvironment.h"


int main() {

  SixLegsForceEnvironment sle;
  State oldState(sle.getCurrentState());

  //Check joint positions
  for(int i = 0; i < oldState.jointPos.size(); ++i ){
     std::cout << "\nprint joint pos i  " << oldState.jointPos.at(i);
  }

  std::vector<float> positions;

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

}
