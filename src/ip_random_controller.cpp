#include <control_bridge.hpp>
#include <joint_listener.hpp>

#include <ros/ros.h>

#include <chrono>
#include <random>



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ip_random_controller");
  ros::NodeHandle nh;

  std::cerr << "Namespace: " << ros::this_node::getNamespace() << std::endl;
  
  // Random control properties
  int frequency = 10;//Hz
  double maxTorque = 2;//N*m
  std::string dof = "axis";
  
  // INITIALIZATION
  ros::Rate r(frequency);
  // Communications with controllers
  std::string ns = ros::this_node::getNamespace();
  ControlBridge bridge(nh, {dof}, ns + "/");        // Write command
  JointListener listener(nh, ns + "/joint_states"); // Read status
  // Random
  unsigned long seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> distribution(-maxTorque,maxTorque);


  // csv header
  std::cout << "time,pos,vel,eff,cmd" << std::endl;

  double cmd = 0;

  while (ros::ok())
  {
    // Read value
    auto joints = listener.getStatus();
    try {
      JointListener::JointState state = joints.at(dof);
      state.pos = fmod(state.pos, 2 * M_PI);
      std::cout << ros::Time::now().toSec() << ","
                << state.pos << ","
                << state.vel << ","
                << state.eff << ","
                << cmd << std::endl;
    }
    catch (const std::out_of_range &exc) {
      std::cerr << "Failed to find " << dof << " in the published joints" << std::endl;
    }

    cmd = distribution(generator);
    bridge.send({{dof,cmd}});
    ros::spinOnce();
    r.sleep();
  }
}
