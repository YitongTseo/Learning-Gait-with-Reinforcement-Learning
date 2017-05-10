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
      we = WheelsEnvironment();
      std::cout<<"2";
      ql = qLearningAgent(we);
      std::cout<<"3\n";

      for (int i = 0; i < this->jointsVector.size(); ++i) {
        physics::JointPtr joint = this->jointsVector.at(i);
        cout << i << " joint Name: " << joint->GetName() << endl;
        cout << joint->GetAngle(0) << endl;
      }

      

      //cout << "\nGETANGLE " << this->jointsVector.at(2)->GetRelativeLinearVel(0) << endl; //Debug
      // this->jointsVector.at(1)->SetPosition(0, 1);

      //set the method OnUpdate() as a listener. It'll be called every time step.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MoveLegs::OnUpdate, this, _1));

    }

  // Called by the world update start event
  public: void OnUpdate(const common::UpdateInfo & /*_info*/)
  {

    this->jointsVector.at(0)->SetForce(0, 10.15f);
    this->jointsVector.at(1)->SetForce(0, 10.15f);
    this->jointsVector.at(3)->SetForce(0, 10.15f);
    this->jointsVector.at(4)->SetForce(0, 10.15f);
    // for (int i = 0; i < this->jointsVector.size(); ++i) {
    //   physics::JointPtr joint = this->jointsVector.at(i);
    //   cout << "joint Name: " << joint->GetName();
    // }


    // this->jointsVector.at(0)->SetPosition(0, 2);
    // this->jointsVector.at(1)->SetPosition(0, 1);
    
    // Apply a small linear velocity to the model.
    // this->model->SetLinearVel(math::Vector3(0.06, 0, 0));

    //     if (this->we.isTerminal(relativePosition.x, relativePosition.z)){
    //   this->we.reset();
    //   this->model.reset();//TODO!: teleport the model back to original position
    // }

    // physics::JointPtr leftJoint = this->jointsVector[0];
    // physics::JointPtr rightJoint = this->jointsVector[1];


    // State oldState(this->we.getCurrentState());
    // Action action = this->ql.getAction();

    // cout << "\nleft wheel velocity " << oldState.leftWheelVelocity;
    // cout << "\nleft wheel increment " << action.leftIncrement;

    // cout << "\nright wheel velocity " << oldState.leftWheelVelocity;
    // cout << "\nright wheel increment " << action.leftIncrement;
    
    // cout << "\n size of bucket" << we.wheelBuckets.size();

    // //find the reward
    // float reward = this->we.getReward(oldState, action);

    // cout << "\nreward " << reward;

    // //update the environment
    // this->we.doAction(action);
    // cout << "\nleft wheel velocity " << oldState.leftWheelVelocity;


    // // leftJoint->SetVelocity(0, .7);
    // State nextState = this->we.getCurrentState();
    // //move the actual joints in Gazebo
    // leftJoint->SetVelocity(0, nextState.leftWheelVelocity);
    // rightJoint->SetVelocity(0, nextState.rightWheelVelocity);




    // math::Vector3 relativeVelocity = this->model->GetRelativeLinearVel();
    // math::Pose relativePose = this->model->GetRelativePose();
    // math::Vector3 relativeRotation = relativePose.rot.GetAsEuler();
    // math::Vector3 relativePosition = relativePose.pos;

    // std::cout << "\nrelative POSITION: x: " << relativePosition.x << " y: " 
    //   << relativePosition.y << " z: " << relativePosition.z;

    // std::cout << "\n         relative VELOCITY: x: " << relativeVelocity.x << " y: " 
    //   << relativeVelocity.y << " z: " << relativeVelocity.z;

    // std::cout << "\n         GETLENGTH " << relativeVelocity.GetLength();

    // std::cout << "\n robotOrientation" << nextState.robotOrientation << endl;

    // //learn to move forward in the x direction
    // this->we.setRobotReward(relativePosition.x - abs(relativePosition.y));
    // this->we.setRobotOrientation(relativeRotation.z);
    
    // //then call update Beliefs with those arguments.
    // this->ql.updateBeliefs(oldState, action, nextState, reward);

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
  GZ_REGISTER_MODEL_PLUGIN(MoveLegs)
}
