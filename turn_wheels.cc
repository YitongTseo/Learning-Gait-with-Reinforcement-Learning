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
//#include <math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

namespace gazebo
{
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

      //set the method OnUpdate() as a listener. It'll be called every time step.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&TurnWheels::OnUpdate, this, _1));

    }

  // Called by the world update start event
  public: void OnUpdate(const common::UpdateInfo & /*_info*/)
  {
    // Apply a small linear velocity to the model.
    //this->model->SetLinearVel(math::Vector3(0.06, 0, 0));

    physics::JointPtr leftJoint = this->jointsVector[0];
    physics::JointPtr rightJoint = this->jointsVector[1];

    double velocity = leftJoint->GetVelocity(0);

    leftJoint->SetVelocity(0, .7);

    std::cout << "\n\nhey! what's the left joint velocity? ";
    std::cout << velocity;

    // tuple = takeStep(...)
    // leftWheel = tuple[0]

    math::Vector3 relativeVelocity = this->model->GetRelativeLinearVel();
    math::Pose relativePose = this->model->GetRelativePose();
    math::Quaternion relativeRotation = relativePose.rot;
    math::Vector3 relativePosition = relativePose.pos;

    std::cout << "\nrelative POSITION: x: " << relativePosition.x << " y: " 
      << relativePosition.y << " z: " << relativePosition.z;

    std::cout << "\n         relative VELOCITY: x: " << relativeVelocity.x << " y: " 
      << relativeVelocity.y << " z: " << relativeVelocity.z;

  }

    // Pointer to the model
  private: physics::ModelPtr model;

  // Array of JointPtrs
  private: std::vector<physics::JointPtr> jointsVector;

    // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(TurnWheels)
}
