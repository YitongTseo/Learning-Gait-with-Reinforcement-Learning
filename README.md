# Learning-Gait-with-Reinforcement-Learning
Learning 3D gait for wheeled vehicles, six legged models, and quadrupedal models using Q-Learning and other Reinforcement Learning techniques


Uses Gazebo (an ROS package) for physics simulation.
Must have Gazebo installed. (make sure Ruby is up to date) and have a C++ compiler (make sure XCode is up to date)

To run create a build folder following the instructions here: http://gazebosim.org/tutorials?tut=animated_box


TIPS and TRICKS to make things work.


for me at least gazebo is stored at:
/usr/local/share/gazebo-8/

if gazebo isn’t starting.
type gazebo —verbose

if you find something like: Error [RTShaderSystem.cc:408] Unable to find shader lib.
export GAZEBO_RESOURCE_PATH=/usr/local/share/gazebo-8:/usr/local/share/gazebo_models:${GAZEBO_RESOURCE_PATH}
^but before doing that unset the GAZEBO_RESOURCE_PATH



when plugins don’t work first check what the path is set to.
echo $GAZEBO_PLUGIN_PATH

To unset the path if its a mess
unset GAZEBO_PLUGIN_PATH

Just appends the current pwd to the end of GAZEBO_PLUGIN_PATH (INSIDE THE /build folder!!!!!!)
export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH

instead, you probably want to use this. (INSIDE THE /build folder!!!!!!)
export GAZEBO_PLUGIN_PATH=$PWD


if it can’t find the model:
it’s probably a problem with
GAZEBO_MODEL_PATH
http://answers.gazebosim.org/question/177/gazebo-default-directory-and-can-not-insert-model/
