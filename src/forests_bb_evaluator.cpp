#include "problems/problem_factory.h"

#include <rosban_regression_forests/core/forest.h>

#include <rosban_utils/serializable.h>

#include <ros/ros.h>

#include <chrono>
#include <fstream>

void write_log(std::ostream &out,
               int run, int step,
               const Eigen::VectorXd &state,
               const Eigen::VectorXd &action)
{
  out << run << "," << step << ",";
  for (int i = 0; i < state.rows(); i++)
  {
    out << state(i) << ",";
  }
  for (int i = 0; i < action.rows(); i++)
  {
    out << action(i);
    if (i < action.rows() - 1) out << ",";
  }
  out << std::endl;
}

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
    }

  void from_xml(TiXmlNode *node) override
    {
      nb_runs  = rosban_utils::xml_tools::read<int>(node, "nb_runs");
      nb_steps = rosban_utils::xml_tools::read<int>(node, "nb_steps");
      policies = rosban_utils::xml_tools::read_vector<std::string>(node, "policies");
      problem  = rosban_utils::xml_tools::read<std::string>(node, "problem");
    }

  int nb_runs;
  int nb_steps;
  std::vector<std::string> policies;
  std::string problem;
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
  BlackBoxProblem * problem = ProblemFactory::buildBlackBox(config.problem);

  std::vector<std::unique_ptr<regression_forests::Forest>> policies;
  for (const std::string &path : config.policies)
  {
    policies.push_back(regression_forests::Forest::loadFile(path));
  }

  // Opening reward files
  std::ofstream reward_logs;
  reward_logs.open("rewards.csv");
  reward_logs << "run,totalReward" << std::endl;

  // Opening log file
  std::ofstream logs;
  logs.open("logs.csv");

  // csv header
  logs << "run,step,";
  // State information
  int x_dim = problem->getStateLimits().rows();
  for (int i = 0; i < x_dim; i++)
  {
    logs << "state_" << i << ",";
  }
  // Commands
  int u_dim = problem->getActionLimits().rows();
  for (int i = 0; i < u_dim; i++)
  {
    logs << "action_" << i;
    if (i < u_dim - 1) logs << ",";
  }
  logs << std::endl;

  for (int run = 1; run <= config.nb_runs; run++)
  {
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(u_dim);
    Eigen::VectorXd current_state = problem->getStartingState();

    // Initiate everything needed for the run
    int step = 0;
    bool finish_run = false;
    double trajectory_reward = 0;

    while (!finish_run)
    {
      // Computing action
      for (int dim = 0; dim < cmd.rows(); dim++)
      {
        cmd(dim) = policies[dim]->getValue(current_state);
        // Bounding action
        double min_cmd = problem->getActionLimits()(dim,0);
        double max_cmd = problem->getActionLimits()(dim,1);
        if (cmd(dim) < min_cmd) cmd(dim) = min_cmd;
        if (cmd(dim) > max_cmd) cmd(dim) = max_cmd;
      }
      // Get and apply new sample
      csa_mdp::Sample new_sample = problem->getSample(current_state, cmd);
      write_log(logs, run, step, current_state, cmd);
      trajectory_reward += new_sample.reward;
      current_state = new_sample.next_state;
      // If a terminal state is reached, break the run
      if (problem->isTerminal(current_state) || step >= config.nb_steps)
      {
        cmd = cmd * 0;//No more active command
        finish_run = true;
      }
      step++;
    }//End of while OK
    write_log(logs, run, step, current_state, cmd);
    std::cout << "Reward for run " << run << ": " << trajectory_reward << std::endl;
    reward_logs << run << "," << trajectory_reward << std::endl;
  }

  reward_logs.close();
  logs.close();
}
