cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(animated_box)

# Find packages
find_package(gazebo REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Create libraries and executables
add_library(turn_wheels SHARED turn_wheels.cc)
target_link_libraries(turn_wheels ${GAZEBO_LIBRARIES})# ${Boost_LIBRARIES}

add_library(move_legs SHARED move_legs.cc)
target_link_libraries(move_legs ${GAZEBO_LIBRARIES}) # ${Boost_LIBRARIES}) 
#Maybe if can't link the turnWheels library we have to comment out ${Boost_LIBRARIES})

