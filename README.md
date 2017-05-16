David Burt, Yitong Tseo, Zander Majercik

# Learning-Gait-with-Reinforcement-Learning
Learning 3D gait for wheeled vehicles, six legged models, and quadrupedal models using Q-Learning and other Reinforcement Learning techniques


Uses Gazebo (an ROS package) for physics simulation.
Must have Gazebo installed. (make sure Ruby is up to date) and have a C++ compiler (make sure XCode is up to date)

To run create a build folder following the instructions here: http://gazebosim.org/tutorials?tut=animated_box


## Basic Structure of the Code:

.world files specify a world consisting of models (i.e. sun, ground, and one of our jointed models)
In the .world file the models are also given plugins which can listen to attributes of the models, control the joints of the models, reset the world, etc.

Our models are stored under the model/ directory. For the legged models, we made standalone leg models which we then fit into a cohesive model with a body (e.g. Crab is a model composed of six CrabLeg models)

turn_wheels.cc and six_legs.cc are our two plugins (six_legs is currently set up to handle six legged movement but can easily be refitted for four legged movement). turn_wheels.cc and six_legs.cc can be thought of as the "middle man" between qLeaerning and actually interfacing with the models. 

turn_wheels.cc and six_legs.cc create an instance of a subclass of Environment (WheelsEnvironment SixLegsForceEnvironment respectively) and create an instance of qLearningAgents, feeding the qLearningAgent an Environment subclass.

qLearningAgent was written to be able to learn given any instance of an Environment (which it needs for the getPossibleActions() method) as well as some State, Action, and StateAction classes (all defined in wheeledRobotEnvironment2.h and sixLeggedForceEnvironment.h). 

toy environment was designed to test the q-learning agent in a basic state action space which we could also work through by hand.



## TIPS and TRICKS to connect everything correctly:

let <path> be the path into the folder Learning-Gait-with-Reinforcement-Learning.... (e.g. Users/zandermajercik/AI/Learning-Gait-with-Reinforcement-Learning)

### Probably want to enter the following commands/add them to ~/.bashrc:

export GAZEBO_RESOURCE_PATH=/usr/local/share/gazebo-8:/usr/local/share/gazebo_models:

export GAZEBO_RESOURCE_PATH=<path>:${GAZEBO_RESOURCE_PATH} <- pay extra attention to the colon there.

export GAZEBO_PLUGIN_PATH=<path>/build: 

export GAZEBO_MODEL_PATH=<path>/models

### To check things are set correctly:

echo $GAZEBO_MODEL_PATH should get you <path>/model_editor_models
(e.g. /Users/zandermajercik/AI/Learning-Gait-with-Reinforcement-Learning/model_editor_models)

echo $GAZEBO_PLUGIN_PATH should get something like
Users/zandermajercik/AI/Learning-Gait-with-Reinforcement-Learning/build:

echo $GAZEBO_RESOURCE_PATH should get something like:
/Users/zandermajercik/AI/Learning-Gait-with-Reinforcement-Learning:/usr/local/share/gazebo-8:/usr/local/share/gazebo_models:



## Misc Tips and Tricks

for me at least gazebo is stored at:
/usr/local/share/gazebo-8/

**if gazebo isn’t starting.**
type "gazebo —verbose" for more information

**if you find something like: Error [RTShaderSystem.cc:408] Unable to find shader lib.**
export GAZEBO_RESOURCE_PATH=/usr/local/share/gazebo-8:/usr/local/share/gazebo_models:${GAZEBO_RESOURCE_PATH}


^but before doing that unset the GAZEBO_RESOURCE_PATH
