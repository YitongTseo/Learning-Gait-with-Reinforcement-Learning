//#pragma once
#include "sixLeggedEnvironment.h"
//#include "qLearning.h"
//#include "qLearning.cpp"

int main() {
  std::cout << "is this working?";
  SixLegsEnvironment sle;
  State oldState(sle.getCurrentState());

  for(int i = 0; i < oldState.jointPos.size(); ++i ){
    std::cout << "\nprint joint pos i  " << oldState.jointPos.at(i);
  }
  std::vector<Action> actions = sle.getPossibleActions();

  for(int i = 0; i < actions.size(); ++i ){
    std::cout << "\naction right: " << actions.at(i).rightJIndex << " left: " << actions.at(i).leftJIndex;
  }
  sle.doAction(actions.at(0));


  State newState(sle.getCurrentState());

  for(int i = 0; i < newState.jointPos.size(); ++i ){
    std::cout << "\nnew state i "  << i << " "<< newState.jointPos.at(i);
  }
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
