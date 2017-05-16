/*
 * Adapted from template code from 2012 Open Source Robotics Foundation
 * by: Yitong Tseo, David Burt, Zander Majercik in accordance to the Apache License
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

using namespace gazebo;


  class MoveLegs : public ModelPlugin
  {
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      //store the pointers to the joints
      this->jointsVector = this->model->GetJoints();

      float alpha = 0.1f;
      float gamma = 0.9f;
      float epsilon = 0.2f;
      we = SixLegsForceEnvironment();
      ql = qLearningAgent(we, alpha, gamma, epsilon);

      std::cout << "\nnumJoints: " << this->jointsVector.size();
      for (int i = 0; i < this->jointsVector.size(); ++i) {
        std::cout << "\n" << i << " joint name: " << this->jointsVector.at(i)->GetName();
      }

      jointCount = 1;
      count = 0;
      CSumCount = 0;
      CSum = 0;
      PrevCSum = 0;

      //set the method OnUpdate() as a listener. It'll be called every time step.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MoveLegs::OnUpdate, this, _1));

    }

  public: void OnUpdate(const common::UpdateInfo & /*_info*/)
  {
    //count helps us skip time steps to give our robot a chance to move a bit before thinking about its next action
    count++;
    if (count < 500) {
      return;
    }
    count = 0;


    State oldState(this->we.getCurrentState());
    float legForce = this->jointsVector.at(jointCount)->GetForce(0);

    //For viewing force applied to joints
    //cout << "\njoint index: " << jointCount;
    //cout << "\nleg force: " << legForce;

    //Set the force
    this->we.setJointIndexLegForce(jointCount, legForce);


    Action action = this->ql.getAction();
    cout << "action: jointIndex: " << action.jointIndex << "  force: " << action.force;

    float newLegForce = action.force;
    for (int j = 0; j < 4; ++j) {
      if (j % 4 == jointCount) {
        physics::JointPtr frontJoint = this->jointsVector[j];
        frontJoint->SetForce(0, newLegForce);

        //set the knee joints to exert the opposite force as the hip joints.
        //thereby creating an elLiptical walk pattern.
        physics::JointPtr midJoint = this->jointsVector[j + 4];
        midJoint->SetForce(0, newLegForce);

        physics::JointPtr rearJoint = this->jointsVector[j + 8];
        rearJoint->SetForce(0, newLegForce);
      }
    }

    //gotta feed the state the current positions
    std::vector<float> positions;
    for (int i = 0; i < this->jointsVector.size(); ++i) {
      float jointAngle = 0.0f;
      if ( 0 <= i && i <= 3) {
        jointAngle = float(this->jointsVector.at(i)->GetAngle(0).Radian());

      }
      positions.push_back(jointAngle);
    }
    we.setRobotPosition(positions);


    State nextState = this->we.getCurrentState();
    cout << "\n nextState!";
    nextState.print();


    math::Vector3 relativeVelocity = this->model->GetRelativeLinearVel();
    math::Pose relativePose = this->model->GetRelativePose();
    math::Vector3 relativeRotation = relativePose.rot.GetAsEuler();
    math::Vector3 relativePosition = relativePose.pos;

    std::cout << "\nrelative POSITION: x: " << relativePosition.x << " y: "
      << relativePosition.y << " z: " << relativePosition.z;

    std::cout << "\n         relative VELOCITY: x: " << relativeVelocity.x << " y: "
      << relativeVelocity.y << " z: " << relativeVelocity.z;

    std::cout << "\n         roll: " << relativeRotation.x << " pitch: "
      << relativeRotation.y << " yaw: " << relativeRotation.z;


    //As discussed in our presentation and paper, we experimented
    //with a number of different reward functions.
    //The current reward function rewards walking down the y axis.

    float reward = relativeVelocity.y * 100;

    //we want to punish high roll. maybe roll above a threshold? let's say 0.5
    // if (std::abs(relativeRotation.x) > 0.5) {
    //reward += std::abs(relativeRotation.x);
    // }

    cout << "\nreward " << reward;

    //Keep a count to track the average reward, and ensure that it is increasing (that the model is learning)
    if (CSumCount > 1000) {
      PrevCSum = (CSum/ float(CSumCount));
      CSumCount = 0;
      CSum = 0;
    }
    CSumCount++;
    CSum += reward;
    cout << "\ncount: "  << CSumCount <<" Average: " << (CSum / CSumCount) << " Old Average: " << (PrevCSum);

    //Relative rotation comes in the form: RPY, so to put it in the right order...
    this->we.setRobotOrientationYPR(relativeRotation.z, relativeRotation.y, relativeRotation.x);

    //then call update Beliefs with those arguments.
    this->ql.updateBeliefs(oldState, action, nextState, reward);

    //if the last 5 states have been the same then we should probably restart so our robo doesn't get into a rut.
    bool last5StatesAreSame = false;
    //if we have 5 states stored then check if they are all equal.
    if (last5States.size() >= 6) {
      last5StatesAreSame = true;
      for (int i = 1; i < last5States.size(); ++i) {
        if (last5States.at(i - 1) != last5States.at(i)) {
          last5StatesAreSame = false;
          break;
        }
      }
    }

    //should we restart?
    if (this->we.isTerminal()) {
      //then call update Beliefs with those arguments.
      float terribleReward = -1000.0f;
      this->ql.updateBeliefs(oldState, action, nextState, terribleReward);
      this->we.reset(); //reset environment so q learning can learn the correct beliefs


      this->model->Reset();//teleport the model back to original position

      //reset links and joints
      for (int i = 0; i < this->jointsVector.size(); ++i) {
        this->jointsVector.at(i)->Reset();
      }
      std::vector<physics::LinkPtr> links = this->model->GetLinks();
      for (int i = 0; i < links.size(); ++i) {
        links.at(i)->Reset();
      }
    }

    if (last5States.size() >= 6) {
      last5States.erase(last5States.begin()); //pop off the front (oldest state)
    }
    last5States.push_back(nextState);

    //increment the jointCount.
    //Right now we're skipping everything but the knee joints.
    jointCount = (jointCount + 1) % 4;
  }

  private: 
    SixLegsForceEnvironment we;
    qLearningAgent ql;

    //this is going to alwasy be between 0 - 17, controls which joint we're interested in.
    int jointCount;
    //this will be between 0 and some number.
    int count;
    std::vector<State> last5States;

    //save the last cumulative sum of the reward and the count so we can get some measure of if our model is improving
    float CSum, PrevCSum;
    int CSumCount;

    // Pointer to the model
    physics::ModelPtr model;

    // Array of JointPtrs
    std::vector<physics::JointPtr> jointsVector;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MoveLegs)
