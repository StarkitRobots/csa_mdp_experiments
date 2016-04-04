#include "problems/problem_factory.h"

#include "interface/interface.h"

#include <control_bridge.hpp>
#include <joint_listener.hpp>

#include <rosban_regression_forests/tools/random.h>
#include <rosban_csa_mdp/solvers/mre.h>

#include <ros/ros.h>

#include <chrono>
#include <fstream>

#include <sys/stat.h>

using csa_mdp::MRE;

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
      rosban_utils::xml_tools::write<std::string>("problem", problem, out);
      control_config.write("control", out);
      mre_config.write("mre", out);
    }

  void from_xml(TiXmlNode *node) override
    {
      nb_runs  = rosban_utils::xml_tools::read<int>(node, "nb_runs");
      nb_steps = rosban_utils::xml_tools::read<int>(node, "nb_steps");
      problem  = rosban_utils::xml_tools::read<std::string>(node, "problem");
      control_config.read(node, "control");
      mre_config.read(node, "mre");
    }

  int nb_runs;
  int nb_steps;
  std::string problem;
  rosban_control::ControlConfig control_config;
  csa_mdp::MRE::Config mre_config;
};

/// 3 different states exists
/// Waiting: The main loop wait for the contact to be established
/// Random:  Random actions are applied:
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
    std::cerr << "Usage: rosrun .... config_path:=<path>" << std::endl;
    std::cerr << "\tNote: <path> folder should contain a file named Config.xml" << std::endl;
    exit(EXIT_FAILURE);
  }

  ros::init(argc, argv, "mre_controller");
  ros::NodeHandle nh;

  // Going to the specified path
  if (chdir(config_path.c_str()))
  {
    std::cerr << "Failed to set '" << config_path << "' as working directory." << std::endl;
    exit(EXIT_FAILURE);
  }

  // Create a directory for details if required
  std::string details_path("details");  
  struct stat folder_stat;
  if (stat(details_path.c_str(), &folder_stat) != 0 || !S_ISDIR(folder_stat.st_mode))
  {
    std::string mkdir_cmd = "mkdir " + details_path;
    // If it fails, exit
    if (system(mkdir_cmd.c_str()))
    {
      std::cerr << "Failed to create '" << details_path << "' folder" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  // Loading config
  Config config;
  config.load_file();

  // Building problem
  ControlProblem * problem = ProblemFactory::buildControl(config.problem);

  std::vector<std::string> effectors       = config.control_config.effectors;
  std::vector<std::string> linear_sensors  = config.control_config.linear_sensors;
  std::vector<std::string> angular_sensors = config.control_config.angular_sensors;

  // Creating mre instance
  MRE mre(config.mre_config,
          [problem](const Eigen::VectorXd &state)
          {
            return problem->isTerminal(state);
          });

  // INITIALIZATION
  ros::Rate r(config.control_config.frequency);
  // Communications with controllers
  ControlBridge bridge(nh, effectors, "/" + config.control_config.robot + "/"   );// Write command
  JointListener listener(nh, "/" + config.control_config.robot + "/joint_states");// Read status

  // Logging trajectories
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

  // Logging trajectories
  std::ofstream reset_logs;
  reset_logs.open("reset_logs.csv");
  // csv header
  reset_logs << "time,run,step,";
  // measured position and speed
  for (const std::string & sensor : linear_sensors)
  {
    reset_logs << "pos_" << sensor << ","
               << "vel_" << sensor << ",";
  }
  for (const std::string & sensor : angular_sensors)
  {
    reset_logs << "pos_" << sensor << ","
               << "vel_" << sensor << ",";
  }
  // Commands
  for (size_t i = 0; i < effectors.size(); i++)
  {
    reset_logs << "cmd_" << effectors[i];
    if (i < effectors.size() - 1) reset_logs << ",";
  }
  reset_logs << std::endl;

  // Logging time consumption
  std::ofstream time_logs;
  time_logs.open("time_logs.csv");
  time_logs << "run,type,time" << std::endl;

  State state = State::Waiting;

  for (int run = 1; run <= config.nb_runs; run++)
  {
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(effectors.size());
    Eigen::VectorXd last_state, last_cmd;

    // Initiate everything needed for the run
    int step = 0;
    int reset_step = 0;
    bool finish_run = false;
    double trajectory_reward = 0;
    r.reset();//Updating value might have used some time, need to update

    while (ros::ok() && !finish_run)
    {
      // Treating messages
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
        // If running, update command and write status
        if (state == State::Running)
        {
          cmd = mre.getAction(new_state);
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
            csa_mdp::Sample new_sample(last_state,
                                       last_cmd,
                                       new_state,
                                       reward);
            try{
              mre.feed(new_sample);
            }
            catch(const std::runtime_error &exc)
            {
              std::cerr << "failed to add sample to MRE:" << std::endl
                        << "\tlast_state: " << last_state.transpose() << std::endl
                        << "\tisTerminal(last_state): " << problem->isTerminal(last_state) << std::endl;
              throw exc;
            }
          }
          // If a terminal state is reached, break the run
          if (problem->isTerminal(new_state) || step >= config.nb_steps)
          {
            state = State::Waiting;
            cmd = cmd * 0;//No more active command
            finish_run = true;
          }
          step++;
          last_state = new_state;
          last_cmd = cmd;
        }
        // Return to a valid initial state
        else
        {
          reset_step++;
          cmd = problem->getResetCmd(new_state);
          // Write reset_logs
          reset_logs << ros::Time::now().toSec() << ","
                     << run << "," << reset_step << ",";
          // Write state
          for (int i = 0; i < new_state.rows(); i++)
          {
            reset_logs << new_state(i) << ",";
          }
          // Write command
          for (int i = 0; i < cmd.rows(); i++)
          {
            reset_logs << cmd(i);
            if (i < cmd.rows() - 1) reset_logs << ",";
          }
          reset_logs << std::endl;
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
        motors_orders[motor_name] = cmd(i);
      }
      bridge.send(motors_orders);
      // Sleep if necessary
      r.sleep();
    }//End of while OK
    std::cout << "Reward for run " << run << ": " << trajectory_reward << std::endl;
    std::cout << "Updating policy " << run << std::endl;
    mre.updatePolicy();
    std::cout << "\tTime spent to compute q_value   : " << mre.getQValueTime() << "[s]"
              << std::endl;
    std::cout << "\tTime spent to compute the policy: " << mre.getPolicyTime() << "[s]"
              << std::endl;
    std::string prefix = details_path + "/T" + std::to_string(run) + "_";
    std::cout << "Saving all with prefix " << prefix << std::endl;
    //mre.saveStatus(prefix);
    // Log time
    time_logs << run << ",qValue," << mre.getQValueTime() << std::endl;
    time_logs << run << ",policy," << mre.getPolicyTime() << std::endl;
    // If ros is not ok, do not loop anymore
    if (!ros::ok()) break;
  }
  logs.close();
  time_logs.close();
  reset_logs.close();
}
