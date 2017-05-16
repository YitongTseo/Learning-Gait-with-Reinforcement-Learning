/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include "reinforcementLearning/wheeledRobotEnvironment2.h"
#include "reinforcementLearning/qLearning.cpp"
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdlib.h> 
#include <stdio.h>
#include <vector>

using namespace gazebo;

  enum {
    RIGHT,
    LEFT,
  };

  class TurnWheels : public ModelPlugin
  {
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      //store the pointers to the joints
      this->jointsVector = this->model->GetJoints();

      //initialize a wheels environement to interact with
      we = WheelsEnvironment();

      //construct a q learning agent with states, actions and rewards dicated by the wheeled environment
      ql = qLearningAgent(we);

      //set the method OnUpdate() as a listener. It'll be called every time step.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&TurnWheels::OnUpdate, this, _1));

    }

  // Called by the world update start event
  public: void OnUpdate(const common::UpdateInfo & /*_info*/)
  {
  	//leftJoint is a pointer to the left wheel and rightJoint to the right wheel
    physics::JointPtr leftJoint = this->jointsVector[0];
    physics::JointPtr rightJoint = this->jointsVector[1];

    //old State stores the orientation and velocities in the model as a state variable
    State oldState(this->we.getCurrentState());

    //get the action determined by the Q-Learning algorithm from the Q-learner class based on old State
    Action action = this->ql.getAction();

    //print statements to give information about current wheel velocities and changes in velocity
    cout << "\nleft wheel velocity " << oldState.leftWheelVelocity;
    cout << "\nleft wheel increment " << action.leftIncrement;

    cout << "\nright wheel velocity " << oldState.leftWheelVelocity;
    cout << "\nright wheel increment " << action.leftIncrement;
    
    //update the environment
    this->we.doAction(action);
    cout << "\nleft wheel velocity " << oldState.leftWheelVelocity;

    //get the new velocities in the model dictated by taking the action (i.e. accelerate wheels appropriately)
    State nextState = this->we.getCurrentState();

    //update wheel speeds in Gazebo
    leftJoint->SetVelocity(0, nextState.leftWheelVelocity);
    rightJoint->SetVelocity(0, nextState.rightWheelVelocity);



    //Get the models new velocity, position and orientation
    math::Vector3 relativeVelocity = this->model->GetRelativeLinearVel();
    math::Pose relativePose = this->model->GetRelativePose();
    math::Vector3 relativeRotation = relativePose.rot.GetAsEuler();
    math::Vector3 relativePosition = relativePose.pos;

    //Print out the velocity, position and orientation to monitor progress
    std::cout << "\nrelative POSITION: x: " << relativePosition.x << " y: " 
      << relativePosition.y << " z: " << relativePosition.z;

    std::cout << "\n         relative VELOCITY: x: " << relativeVelocity.x << " y: " 
      << relativeVelocity.y << " z: " << relativeVelocity.z;

    std::cout << "\n         GETLENGTH " << relativeVelocity.GetLength();

    std::cout << "\n robotOrientation" << nextState.robotOrientation << endl;

	// update the sample reward in the wheeled evironment,
	//learn to move forward in the x direction
	//reward is positive displacement in x direction, minus a penalty 
	// for straying of course in the y-direction
    float reward = relativePosition.x - abs(relativePosition.y);

    //print the reward
    cout << "\nreward " << reward;

    //Update the wheeled environment so it know the new robot orientation
    this->we.setRobotOrientation(relativeRotation.z);
    
    //call update Beliefs with the old State, the action taken, the new state and the reward 
    this->ql.updateBeliefs(oldState, action, nextState, reward);

  }

  private: WheelsEnvironment we;
  private: qLearningAgent ql;

    // Pointer to the model
  private: physics::ModelPtr model;

  // Array of JointPtrs
  private: std::vector<physics::JointPtr> jointsVector;

    // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TurnWheels)

