#include "problems/problem_factory.h"

#include "interface/interface.h"

#include <control_bridge.hpp>
#include <joint_listener.hpp>

#include <rosban_regression_forests/core/forest.h>

#include <ros/ros.h>

#include <chrono>
#include <fstream>

class Config : public rosban_utils::Serializable
{
public:
  Config()
    {
    }

  std::string class_name() const override
    {
      return "Config";
    }

  void to_xml(std::ostream &out) const override
    {
      rosban_utils::xml_tools::write<int>("nb_runs", nb_runs, out);
      rosban_utils::xml_tools::write<int>("nb_steps", nb_steps, out);
      rosban_utils::xml_tools::write_vector<std::string>("policies", policies, out);
      rosban_utils::xml_tools::write<std::string>("problem", problem, out);
      control_config.write("control", out);
    }

  void from_xml(TiXmlNode *node) override
    {
      nb_runs  = rosban_utils::xml_tools::read<int>(node, "nb_runs");
      nb_steps = rosban_utils::xml_tools::read<int>(node, "nb_steps");
      policies = rosban_utils::xml_tools::read_vector<std::string>(node, "policies");
      problem  = rosban_utils::xml_tools::read<std::string>(node, "problem");
      control_config.read(node, "control");
    }

  int nb_runs;
  int nb_steps;
  std::vector<std::string> policies;
  std::string problem;
  rosban_control::ControlConfig control_config;
};

/// 3 different states exists
/// Waiting: The main loop wait for the contact to be established
/// Running:  Controlled actions are applied:
/// Reset:   A special policy is used to bring back the robot to initial conditions
enum State
{
  Waiting, Running, Reset
};


int main(int argc, char ** argv)
{
  // Parsing ros arguments
  std::string config_path = ros::getROSArg(argc, argv, "config_path");

  if (config_path == "")
  {
    std::cerr << "Usage: rosrun .... config_path:=<file>" << std::endl;
    exit(EXIT_FAILURE);
  }

  ros::init(argc, argv, "random_controller");
  ros::NodeHandle nh;

  // Going to the specified path
  if (chdir(config_path.c_str()))
  {
    std::cerr << "Failed to set '" << config_path << "' as working directory." << std::endl;
    exit(EXIT_FAILURE);
  }

  // Loading config
  Config config;
  config.load_file();

  // Building problem
  ControlProblem * problem = ProblemFactory::build(config.problem);

  std::vector<std::string> effectors       = config.control_config.effectors;
  std::vector<std::string> linear_sensors  = config.control_config.linear_sensors;
  std::vector<std::string> angular_sensors = config.control_config.angular_sensors;

  std::vector<std::unique_ptr<regression_forests::Forest>> policies;
  for (const std::string &path : config.policies)
  {
    policies.push_back(regression_forests::Forest::loadFile(path));
  }

  // INITIALIZATION
  ros::Rate r(config.control_config.frequency);
  // Communications with controllers
  ControlBridge bridge(nh, effectors, "/" + config.control_config.robot + "/"   );// Write command
  JointListener listener(nh, "/" + config.control_config.robot + "/joint_states");// Read status

  // Opening log file
  std::ofstream logs;
  logs.open("logs.csv");

  // csv header
  logs << "time,run,step,";
  // measured position and speed
  for (const std::string & sensor : linear_sensors)
  {
    logs << "pos_" << sensor << ","
         << "vel_" << sensor << ",";
  }
  for (const std::string & sensor : angular_sensors)
  {
    logs << "pos_" << sensor << ","
         << "vel_" << sensor << ",";
  }
  // Commands
  for (size_t i = 0; i < effectors.size(); i++)
  {
    logs << "cmd_" << effectors[i];
    if (i < effectors.size() - 1) logs << ",";
  }
  logs << std::endl;

  State state = State::Waiting;

  for (int run = 1; run <= config.nb_runs; run++)
  {
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(effectors.size());
    Eigen::VectorXd last_state, last_cmd;

    int step = 0;
    bool finish_run = false;
    double trajectory_reward = 0;

    while (ros::ok() && !finish_run)
    {
      //Treat messages
      ros::spinOnce();
      // Reading state
      auto joints = listener.getStatus();
      try
      {
        // Analyzing state (exception if thrown if something goes wrong)
        Eigen::VectorXd new_state = joints2State(joints, config.control_config);
        // Once the state has been properly received, jump to reset state
        if (state == State::Waiting)
          state = State::Reset;
        // If state was reset and new_state is a valid start, update state
        if (state == State::Reset && problem->isValidStart(new_state))
          state = State::Running;
        // If running state, update command and write status
        if (state == State::Running)
        {
          for (int dim = 0; dim < cmd.rows(); dim++)
          {
            cmd(dim) = policies[dim]->getValue(new_state);
          }
          logs << ros::Time::now().toSec() << ","
                    << run << "," << step << ",";
          // Write state
          for (int i = 0; i < new_state.rows(); i++)
          {
            logs << new_state(i) << ",";
          }
          // Write command
          for (int i = 0; i < cmd.rows(); i++)
          {
            logs << cmd(i);
            if (i < cmd.rows() - 1) logs << ",";
          }
          logs << std::endl;
          // If we're not at the first step, add sample to mre
          if (step != 0)
          {
            double reward = problem->getReward(last_state, last_cmd, new_state);
            trajectory_reward += reward;
          }
          // If a terminal state is reached, break the run
          if (problem->isTerminal(new_state) || step > config.nb_steps)
          {
            state = State::Waiting;
            cmd = cmd * 0;//No more active command
            break;
          }
          step++;
          last_state = new_state;
          last_cmd = cmd;
        }
        // Return to a valid initial state
        else
        {
          cmd = problem->getResetCmd(new_state);
        }
      }
      // sensors were missing
      catch (const std::out_of_range &exc)
      {
        std::cerr << exc.what() << std::endl;
        if (state != State::Waiting)
        {
          std::cerr << "Connection has been lost" << std::endl;
          break;
        }
      }
      // Send command (TODO: externalize)
      std::map<std::string, double> motors_orders;
      for (int i = 0; i < cmd.rows(); i++)
      {
        std::string motor_name = effectors[i];
        motors_orders[motor_name] = cmd(i );
      }
      bridge.send(motors_orders);
      // Sleep if necessary
      r.sleep();
    }//End of while OK
    std::cout << "Reward for run " << run << ": " << trajectory_reward << std::endl;
    // If ros is not ok, do not loop anymore
    if (!ros::ok()) break;
  }

  logs.close();
}
