#include "problems/cart_pole.h"

#include "interface/interface.h"

#include <control_bridge.hpp>
#include <joint_listener.hpp>

#include <rosban_regression_forests/tools/random.h>

#include <ros/ros.h>

#include <chrono>

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
      rosban_utils::xml_tools::write<std::string>("problem", problem, out);
      control_config.write("control", out);
    }

  void from_xml(TiXmlNode *node) override
    {
      nb_runs = rosban_utils::xml_tools::read<int>(node, "nb_runs");
      problem = rosban_utils::xml_tools::read<std::string>(node, "problem");
      control_config.read(node, "control");
    }

  int nb_runs;
  std::string problem;
  rosban_control::ControlConfig control_config;
};

/// 3 different states exists
/// Waiting: The main loop wait for the contact to be established
/// Random:  Random actions are applied:
/// Reset:   A special policy is used to bring back the robot to initial conditions
enum State
{
  Waiting, Random, Reset
};


int main(int argc, char ** argv)
{
  // Parsing ros arguments
  std::string config_path = ros::getROSArg(argc, argv, "config");

  if (config_path == "")
  {
    std::cerr << "Usage: rosrun .... config:=<file>" << std::endl;
    exit(EXIT_FAILURE);
  }

  ros::init(argc, argv, "random_controller");
  ros::NodeHandle nh;

  // Loading config
  Config config;
  config.load_file(config_path);

  // Choosing problem
  ControlProblem * problem = NULL;
  if (config.problem == "CartPole")
  {
    problem = new CartPole();
  }
  else
  {
    std::cerr << "Unknown problem: '" << config.problem << "'" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::vector<std::string> effectors       = config.control_config.effectors;
  std::vector<std::string> linear_sensors  = config.control_config.linear_sensors;
  std::vector<std::string> angular_sensors = config.control_config.angular_sensors;

  // INITIALIZATION
  ros::Rate r(config.control_config.frequency);
  // Communications with controllers
  ControlBridge bridge(nh, effectors, "/" + config.control_config.robot + "/"   );// Write command
  JointListener listener(nh, "/" + config.control_config.robot + "/joint_states");// Read status

  // Random
  std::default_random_engine generator = regression_forests::get_random_engine();

  // csv header
  std::cout << "time,run,step,";
  // measured position and speed
  for (const std::string & sensor : linear_sensors)
  {
    std::cout << "pos_" << sensor << ","
              << "vel_" << sensor << ",";
  }
  for (const std::string & sensor : angular_sensors)
  {
    std::cout << "pos_" << sensor << ","
              << "vel_" << sensor << ",";
  }
  // Commands
  for (size_t i = 0; i < effectors.size(); i++)
  {
    std::cout << "cmd_" << effectors[i];
    if (i < effectors.size() - 1) std::cout << ",";
  }
  std::cout << std::endl;

  State state = State::Waiting;

  for (int run = 1; run <= config.nb_runs; run++)
  {
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(effectors.size());
    Eigen::VectorXd last_state;

    int step = 0;

    while (ros::ok())
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
          state = State::Random;
        // If random state, update command and write status
        if (state == State::Random)
        {
          cmd = regression_forests::getUniformSamples(problem->getActionLimits(), 1, &generator)[0];
          std::cout << ros::Time::now().toSec() << ","
                    << run << "," << step << ",";
          // Write state
          for (int i = 0; i < new_state.rows(); i++)
          {
            std::cout << new_state(i) << ",";
          }
          // Write command
          for (int i = 0; i < cmd.rows(); i++)
          {
            std::cout << cmd(i);
            if (i < cmd.rows() - 1) std::cout << ",";
          }
          std::cout << std::endl;
          // If a terminal state is reached, break the run
          if (problem->isTerminal(new_state))
          {
            state = State::Waiting;
            break;
          }
          step++;
          last_state = new_state;
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
  }
}
