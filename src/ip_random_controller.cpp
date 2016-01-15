#include <control_bridge.hpp>

#include <ros/ros.h>

#include <chrono>
#include <random>



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ip_random_controller");
  ros::NodeHandle nh;

  std::cout << "Namespace: " << ros::this_node::getNamespace() << std::endl;
  
  // Random control properties
  int frequency = 10;//Hz
  double maxTorque = 3;//N*m
  std::string dof = "axis";
  
  // INITIALIZATION
  ros::Rate r(frequency);
  // Communications with controllers
  ControlBridge bridge(nh, {dof}, ros::this_node::getNamespace() + "/");
  // Random
  unsigned long seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> distribution(-maxTorque,maxTorque);

  while (ros::ok())
  {
    double value = distribution(generator);
    bridge.send({{dof,value}});
    ros::spinOnce();
    r.sleep();
  }
}
