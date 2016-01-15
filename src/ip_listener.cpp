#include <joint_listener.hpp>

#include <ros/ros.h>

#include <chrono>
#include <random>



int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ip_listener");
  ros::NodeHandle nh;

  std::cout << "Namespace: " << ros::this_node::getNamespace() << std::endl;
  
  // Listening properties
  int frequency = 10;//Hz
  
  // INITIALIZATION
  ros::Rate r(frequency);
  // Subscribe to joint_state
  JointListener listener(nh, ros::this_node::getNamespace() + "/joint_states");

  while (ros::ok())
  {
    auto joints = listener.getStatus();
    std::cout << "At time: " << ros::Time::now().toSec() << std::endl;
    for (const std::pair<std::string, JointListener::JointState> &joint : joints) {
      std::cout << "\t" << joint.first << std::endl;
      std::cout << "\t\tPos:" << joint.second.pos << std::endl;
      std::cout << "\t\tVel:" << joint.second.vel << std::endl;
      std::cout << "\t\tEff:" << joint.second.eff << std::endl;
    }
    ros::spinOnce();
    r.sleep();
  }
}
