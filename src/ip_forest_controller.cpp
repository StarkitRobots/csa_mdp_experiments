#include "rosban_regression_forests/core/forest.h"

#include <control_bridge.hpp>
#include <joint_listener.hpp>

#include <ros/ros.h>

using regression_forests::Forest;

int main(int argc, char ** argv)
{
  std::string policies_prefix = ros::getROSArg(argc, argv, "policies_prefix");
  std::string robot = ros::getROSArg(argc, argv, "robot");
  std::string dof_count_string = ros::getROSArg(argc, argv, "_dof_count");
  if (policies_prefix == "" || robot == "" || dof_count_string == "")
  {
    std::cerr << "Usage: rosrun .... policies_prefix:=<file> robot:=<robot> _dof_count:=<nb_dofs>" << std::endl;
    exit(EXIT_FAILURE);
  }

  int dof_count = std::stoi(dof_count_string);

  // Load policy
  std::vector<std::unique_ptr<Forest>> policies;
  for (int i = 1; i <= dof_count; i++)
  {
    std::string policy_path = policies_prefix + "_" + std::to_string(i);
    policies.push_back(Forest::loadFile(policy_path));
    std::cerr << "Nb Trees for dof " << i << ": " << policies[i-1]->nbTrees() << std::endl;
  }

  ros::init(argc, argv, "ip_forest_controller");
  ros::NodeHandle nh;

  // Control properties
  int frequency = 10;//Hz
  double max_torque = 2.5;//N*m

  Eigen::MatrixXd cmd_limits(2,2);
  cmd_limits << -max_torque, max_torque, -max_torque, max_torque;

  // Listing axis
  std::vector<std::string> dofs;
  for (int i = 1; i <= dof_count; i++)
  {
    dofs.push_back("axis" + std::to_string(i));
  }
  
  // INITIALIZATION
  ros::Rate r(frequency);
  // Communications with controllers
  std::string ns = ros::this_node::getNamespace();
  ControlBridge bridge(nh, dofs, "/" + robot + "/");        // Write command
  JointListener listener(nh, "/" + robot + "/joint_states"); // Read status

  // csv header
  std::cout << "time,";
  // state and action variables + measured effort
  for (int i = 1; i <= dof_count; i++)
  {
    std::cout << "theta" << i << ","
              << "omega" << i << ",";
  }
  for (int i = 1; i <= dof_count; i++)
  {
    std::cout << "command" << i;
    if (i < dof_count)
      std::cout << ",";
  }
  std::cout << std::endl;

  // Default command: 0 on all axis
  Eigen::VectorXd cmd = Eigen::VectorXd::Zero(dof_count);
  Eigen::VectorXd ip_state = Eigen::VectorXd::Zero(2 * dof_count);

  while (ros::ok())
  {
    // Setting default targets
    std::map<std::string, double> targets;
    for (const std::string & dof : dofs)
    {
      targets[dof] = 0;
    }
    // Read value
    auto joints = listener.getStatus();
    std::string dof_name;
    try {
      std::ostringstream line;
      line << ros::Time::now().toSec() << ",";
      for (int dof_id = 1; dof_id <= dof_count; dof_id++)
      {
        dof_name = "axis" + std::to_string(dof_id);
        JointListener::JointState joint_state = joints.at(dof_name);
        // Normalizing pos in -pi, pi
        while (joint_state.pos > M_PI)
        {
          joint_state.pos -= 2 * M_PI;
        }
        while (joint_state.pos < -M_PI)
        {
          joint_state.pos += 2 * M_PI;
        }
        // Writing values to line
        line << joint_state.pos << ","
             << joint_state.vel << ",";
        // Writing values to ip_state
        ip_state(2 * (dof_id - 1)    ) = joint_state.pos;
        ip_state(2 * (dof_id - 1) + 1) = joint_state.vel;
      }
      // If we received all required information, update targets and write targets
      for (int i = 0; i <= dof_count; i++)
      {
        // Getting command
        cmd(i) = policies[i]->getValue(ip_state);
        // Normalizing command
        if (cmd(i) < -max_torque) cmd(i) = -max_torque;
        if (cmd(i) >  max_torque) cmd(i) =  max_torque;
        line << cmd(i);
        if (i < dof_count - 1)
        {
          line << ",";
        }
      }
      // Write line
      std::cout << line.str() << std::endl;
    }
    catch (const std::out_of_range &exc) {
      std::cerr << "Failed to find " << dof_name << " in the published joints" << std::endl;
    }
    bridge.send(targets);//0 if we did no
    ros::spinOnce();
    r.sleep();
  }
}
