/// A random controller for the Cart-Pole problem
/// In this problem, a cart is controlled, moving along a linear joint.
/// One or more pendulum are fixed on the cart and the goal is to stabilize them
#include <control_bridge.hpp>
#include <joint_listener.hpp>

#include <rosban_regression_forests/tools/random.h>

#include <ros/ros.h>

#include <chrono>

int main(int argc, char ** argv)
{
  // Parsing ros arguments
  std::string robot = ros::getROSArg(argc, argv, "robot");
  std::string pole_count_string = ros::getROSArg(argc, argv, "_pole_count");

  if (robot == "" || pole_count_string == "")
  {
    std::cerr << "Usage: rosrun .... robot:=<file> _pole_count:=<path>" << std::endl;
    exit(EXIT_FAILURE);
  }

  int pole_count = std::stoi(pole_count_string);

  ros::init(argc, argv, "cp_random_controller");
  ros::NodeHandle nh;
  
  // Random control properties
  int frequency = 10;//Hz
  double max_torque = 100;//N
  std::string cmd_name = "cart";

  // INITIALIZATION
  ros::Rate r(frequency);
  // Communications with controllers
  ControlBridge bridge(nh, {cmd_name}, "/" + robot + "/");        // Write command
  JointListener listener(nh, "/" + robot + "/joint_states"); // Read status
  // Random
  std::default_random_engine generator = regression_forests::get_random_engine();
  std::uniform_real_distribution<double> action_distribution(-max_torque, max_torque);

  // csv header
  std::cout << "run,time,cart_pos,cart_speed,";
  // state and action variables + measured effort
  for (int i = 1; i <= pole_count; i++)
  {
    std::cout << "theta" << i << ","
              << "omega" << i << ",";
  }
  std::cout << "cmd" << std::endl;

  // Value means: to Normalize
  std::vector<std::pair<std::string, bool>> dofs = {{"cart_joint",false}};
  for (int dof_id = 1; dof_id <= pole_count; dof_id++)
  {
    std::string dof_name = "axis" + std::to_string(dof_id);
    dofs.push_back(std::pair<std::string,bool>(dof_name,true));
  }

  double cmd = 0;
  int run = 1;

  while (ros::ok())
  {
    // Read value
    auto joints = listener.getStatus();
    std::string dof_name;
    try {
      std::ostringstream line;
      line << run << "," << ros::Time::now().toSec() << ",";
      for (const auto & dof : dofs)
      {
        dof_name = dof.first;
        JointListener::JointState state = joints.at(dof_name);
        // Normalizing pos in -pi, pi for angular dofs
        if (dof.second)
        {
          while (state.pos > M_PI / 2)
          {
            state.pos -= 2 * M_PI;
          }
          while (state.pos < -M_PI)
          {
            state.pos += 2 * M_PI;
          }
        }
        // Writing values
        line << state.pos << ","
             << state.vel << ",";
      }
      line << cmd;
      std::cout << line.str() << std::endl;
    }
    catch (const std::out_of_range &exc) {
      std::cerr << "Failed to find " << dof_name << " in the published joints" << std::endl;
    }
    cmd = action_distribution(generator);
    bridge.send({{"cart", cmd}});
    ros::spinOnce();
    r.sleep();
  }
}
