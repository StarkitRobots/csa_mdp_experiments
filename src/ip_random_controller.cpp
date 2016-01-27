#include <control_bridge.hpp>
#include <joint_listener.hpp>

#include <rosban_regression_forests/tools/random.h>

#include <ros/ros.h>

#include <chrono>

int main(int argc, char ** argv)
{
  // Parsing ros arguments
  std::string robot = ros::getROSArg(argc, argv, "robot");
  std::string dof_count_string = ros::getROSArg(argc, argv, "_dof_count");

  if (robot == "" || dof_count_string == "")
  {
    std::cerr << "Usage: rosrun .... robot:=<file> _dof_count:=<path>" << std::endl;
    exit(EXIT_FAILURE);
  }

  int dof_count = std::stoi(dof_count_string);

  ros::init(argc, argv, "ip_random_controller");
  ros::NodeHandle nh;
  
  // Random control properties
  int frequency = 10;//Hz
  double maxTorque = 2.5;//N*m

  Eigen::MatrixXd cmd_limits(2,2);
  cmd_limits << -maxTorque, maxTorque, -maxTorque, maxTorque;

  // Listing axis
  std::vector<std::string> dofs;
  for (int i = 1; i <= dof_count; i++)
  {
    dofs.push_back("axis" + std::to_string(i));
  }
  
  // INITIALIZATION
  ros::Rate r(frequency);
  // Communications with controllers
  ControlBridge bridge(nh, dofs, "/" + robot + "/");        // Write command
  JointListener listener(nh, "/" + robot + "/joint_states"); // Read status
  // Random
  std::default_random_engine generator = regression_forests::get_random_engine();

  // csv header
  std::cout << "time,";
  // state and action variables + measured effort
  for (int i = 1; i <= dof_count; i++)
  {
    std::cout << "theta" << i << ","
              << "omega" << i << ","
              << "effort" << i << ","
              << "command" << i;
    if (i < dof_count)
      std::cout << ",";
  }
  std::cout << std::endl;

  Eigen::VectorXd cmd = Eigen::VectorXd::Zero(dof_count);

  while (ros::ok())
  {
    // Read value
    auto joints = listener.getStatus();
    std::string dof_name;
    try {
      std::ostringstream line;
      line << ros::Time::now().toSec() << ",";
      for (int dof_id = 1; dof_id <= dof_count; dof_id++)
      {
        dof_name = "axis" + std::to_string(dof_id);
        JointListener::JointState state = joints.at(dof_name);
        // Normalizing pos in -pi, pi
        while (state.pos > M_PI)
        {
          state.pos -= 2 * M_PI;
        }
        while (state.pos < -M_PI)
        {
          state.pos += 2 * M_PI;
        }
        // Writing values
        line << state.pos << ","
             << state.vel << ","
             << state.eff << ","
             << cmd(dof_id - 1);
        if (dof_id < dof_count)
        {
          line << ",";
        }
      }
      std::cout << line.str() << std::endl;
    }
    catch (const std::out_of_range &exc) {
      std::cerr << "Failed to find " << dof_name << " in the published joints" << std::endl;
    }
    cmd = regression_forests::getUniformSamples(cmd_limits, 1, &generator)[0];
    std::map<std::string, double> motors_orders;
    for (int dof_id = 1; dof_id <= dof_count; dof_id++)
    {
      std::string motor_name = "axis" + std::to_string(dof_id);
      motors_orders[motor_name] = cmd(dof_id - 1);
    }
    bridge.send(motors_orders);
    ros::spinOnce();
    r.sleep();
  }
}
