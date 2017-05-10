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
#include "reinforcementLearning/sixLeggedForceEnvironment.h"
#include "reinforcementLearning/qLearning.cpp"
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdlib.h> 
#include <stdio.h>
#include <vector>

namespace gazebo
{

  class MoveLegs : public ModelPlugin
  {
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      //store the pointers to the joints
      this->jointsVector = this->model->GetJoints();

      std::cout<<"\n1";
      we = SixLegsForceEnvironment();
      std::cout<<"2";
      ql = qLearningAgent(we);
      std::cout<<"3\n";

      jointCount = 0;
      count = 0;


      //set the method OnUpdate() as a listener. It'll be called every time step.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MoveLegs::OnUpdate, this, _1));

    }

  // Called by the world update start event
  public: void OnUpdate(const common::UpdateInfo & /*_info*/)
  {
    //count helps us skip time steps (100 at a time currently)
    count++;
    if (count < 1000) {
      return;
    }
    count = 0;


    State oldState(this->we.getCurrentState());
    float legForce = this->jointsVector.at(jointCount)->GetForce(0);

    cout << "\njoint index: " << jointCount;
    cout << "\nleg force: " << legForce;


    this->we.setJointIndexLegForce(jointCount, legForce);
    Action action = this->ql.getAction();

    cout << "action: jointIndex: " << action.jointIndex << "  increment: " << action.increment;


    //move the actual joints in Gazebo
    physics::JointPtr leftJoint = this->jointsVector[action.jointIndex];
    float newLegForce = legForce + action.increment * we.jointForceIncrement;
    leftJoint->SetForce(0, newLegForce);

    physics::JointPtr rightJoint = this->jointsVector[action.jointIndex + 3];
    rightJoint->SetForce(0, newLegForce);



    //gotta feed the state the current positions
    std::vector<float> positions;
    for (int i = 0; i < this->jointsVector.size(); ++i) {
      float jointAngle = float(this->jointsVector.at(i)->GetAngle(0).Radian());
      positions.push_back(jointAngle);
    }
    we.setRobotPosition(positions);


    State nextState = this->we.getCurrentState();
   


    math::Vector3 relativeVelocity = this->model->GetRelativeLinearVel();
    math::Pose relativePose = this->model->GetRelativePose();
    math::Vector3 relativeRotation = relativePose.rot.GetAsEuler();
    math::Vector3 relativePosition = relativePose.pos;

    std::cout << "\nrelative POSITION: x: " << relativePosition.x << " y: " 
      << relativePosition.y << " z: " << relativePosition.z;

    std::cout << "\n         relative VELOCITY: x: " << relativeVelocity.x << " y: " 
      << relativeVelocity.y << " z: " << relativeVelocity.z;

    std::cout << "\n         GETLENGTH " << relativeVelocity.GetLength();

   
    //the greater the velocity the better.
    float reward = relativePosition.x * 10;

    //we want to punish high roll.
    reward += std::abs(relativeRotation.x) * 5;
    
    cout << "\nreward " << reward;

    //Relative rotation comes in the form: RPY, so to put it in the right order...
    this->we.setRobotOrientationYPR(relativeRotation.z, relativeRotation.y, relativeRotation.x);
    
    //then call update Beliefs with those arguments.
    this->ql.updateBeliefs(oldState, action, nextState, reward);

    if (this->we.isTerminal()){
      //then call update Beliefs with those arguments.
      float terribleReward = -1000.0f;
      this->ql.updateBeliefs(oldState, action, nextState, terribleReward);
      this->we.reset(); //reset environment so q learning can learn the correct beliefs


      this->model->Reset();//teleport the model back to original position

      //reset links and joints
      for (int i = 0; i < 18; ++i) {
        this->jointsVector.at(i)->Reset();
      }
      std::vector<physics::LinkPtr> links = this->model->GetLinks();
      for (int i = 0; i < links.size(); ++i) {
        links.at(i)->Reset();
      }
    }

    //increment the jointCount.
    //Right now we're skipping everything but the knee joints.
    //jointCount = (jointCount + 3) % 18;
    jointCount = (jointCount + 6) % 18;
  }

  private: SixLegsForceEnvironment we;
  private: qLearningAgent ql;

  //this is going to alwasy be between 0 - 17, controls which joint we're interested in.
  private: int jointCount;
  //this will be between 0 and some number.
  private: int count;

    // Pointer to the model
  private: physics::ModelPtr model;

  // Array of JointPtrs
  private: std::vector<physics::JointPtr> jointsVector;

    // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MoveLegs)
}
