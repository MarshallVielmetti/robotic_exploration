/**
 * @file ompl_dubins_test.cpp
 * @brief Quick test for OMPL Dubins path planning
 */

// #include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>

#include <iostream>

int main() {
  // Create a Dubins state space

  ompl::base::DubinsStateSpace dubinsSpace(1.0);  // 1.0 is the turning radiuso

  // Create an initial pose
  auto state1 = dubinsSpace.allocState();
  state1->as<ompl::base::DubinsStateSpace::StateType>()->setXY(0.0, 0.0);
  state1->as<ompl::base::DubinsStateSpace::StateType>()->setYaw(0.0);

  // Create a goal pose
  auto state2 = dubinsSpace.allocState();
  state2->as<ompl::base::DubinsStateSpace::StateType>()->setXY(2.0, 3.0);
  state2->as<ompl::base::DubinsStateSpace::StateType>()->setYaw(0.0);

  auto path = dubinsSpace.dubins(state1, state2);

  std::cout << "Path length: " << path.length() << std::endl;

  // interpolate 20 points along the dubins path
  auto tempState = dubinsSpace.allocState();
  for (double t = 0; t <= 1.0; t += 1.0 / 20.0) {
    std::cout << "Interpolating at t=" << t << std::endl;
    dubinsSpace.interpolate(state1, state2, t, tempState);
    std::cout << "Interpolated state at t=" << t << ": "
              << "x=" << tempState->as<ompl::base::DubinsStateSpace::StateType>()->getX()
              << ", y=" << tempState->as<ompl::base::DubinsStateSpace::StateType>()->getY()
              << ", yaw=" << tempState->as<ompl::base::DubinsStateSpace::StateType>()->getYaw() << std::endl;
  }

  // Print the path information
  std::cout << "Path length: " << path.length() << std::endl;
  std::cout << "Path Type " << *path.type_ << std::endl;
  std::cout << "Path Lengths " << path.length_[0] << " " << path.length_[1] << " " << path.length_[2] << std::endl;

  dubinsSpace.freeState(state1);
  dubinsSpace.freeState(state2);
  dubinsSpace.freeState(tempState);

  //   std::cout << "OMPL Dubins path planning test compiles!" << std::endl;
  return 0;
}
