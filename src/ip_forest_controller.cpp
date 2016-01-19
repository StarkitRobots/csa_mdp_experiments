#include "rosban_regression_forests/core/forest.h"

#include <control_bridge.hpp>
#include <joint_listener.hpp>

#include <ros/ros.h>

using regression_forests::Forest;

int main(int argc, char ** argv)
{
  std::string policy_path = ros::getROSArg(argc, argv, "policy_path");
  if (policy_path == "")
  {
    std::cerr << "Usage: rosrun .... policy_path:=<file>" << std::endl;
    exit(EXIT_FAILURE);
  }
  // Load policy
  std::unique_ptr<Forest> policy = Forest::loadFile(policy_path + ".data");
  std::cerr << "Nb Trees: " << policy->nbTrees() << std::endl;

  ros::init(argc, argv, "ip_forest_controller");
  ros::NodeHandle nh;

  std::cerr << "Namespace: " << ros::this_node::getNamespace() << std::endl;

  // Control properties
  int frequency = 10;//Hz
  std::string dof = "axis";
  
  // INITIALIZATION
  ros::Rate r(frequency);
  // Communications with controllers
  std::string ns = ros::this_node::getNamespace();
  ControlBridge bridge(nh, {dof}, ns + "/");        // Write command
  JointListener listener(nh, ns + "/joint_states"); // Read status

  double cmd = 0;

  std::cout << "pos,vel,cmd" << std::endl;

  while (ros::ok())
  {
    // Read value
    auto joints = listener.getStatus();
    try {
      JointListener::JointState joint_state = joints.at(dof);
      // Normalizing pos in -pi, pi
      joint_state.pos = fmod(joint_state.pos, 2 * M_PI);
      if (joint_state.pos > M_PI)
      {
        joint_state.pos = joint_state.pos - 2 * M_PI;
      }
      // Creating State
      Eigen::VectorXd state(2);
      state(0) = joint_state.pos;
      state(1) = joint_state.vel;
      cmd = policy->getValue(state);
      std::cout << state(0) << "," << state(1) << "," << cmd << std::endl;
    }
    catch (const std::out_of_range &exc) {
      std::cerr << "Failed to find " << dof << " in the published joints" << std::endl;
    }

    bridge.send({{dof,cmd}});
    ros::spinOnce();
    r.sleep();
  }
}
