#include "problems/problem_factory.h"

#include <rosban_regression_forests/tools/random.h>

#include <rosban_utils/serializable.h>

#include <ros/ros.h>

#include <chrono>
#include <fstream>

using csa_mdp::Sample;

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
      rosban_utils::xml_tools::write<int>("nb_runs" , nb_runs , out);
      rosban_utils::xml_tools::write<int>("nb_steps", nb_steps, out);
      rosban_utils::xml_tools::write<std::string>("problem", problem, out);
    }

  void from_xml(TiXmlNode *node) override
    {
      nb_runs  = rosban_utils::xml_tools::read<int>(node, "nb_runs");
      nb_steps = rosban_utils::xml_tools::read<int>(node, "nb_steps");
      problem  = rosban_utils::xml_tools::read<std::string>(node, "problem");
    }

  int nb_runs;
  int nb_steps;
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

  // Opening logs file
  std::ofstream logs;
  logs.open("logs.csv");

  // csv header
  logs << "run,step,";
  // measured position and speed
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
    Eigen::VectorXd starting_state = problem->getStartingState();
    std::vector<Sample> run_samples = problem->getRandomTrajectory(starting_state,
                                                                   config.nb_steps);
    for (size_t step = 0; step < run_samples.size(); step++)
    {
      const Sample & s = run_samples[step];
      write_log(logs, run, step, s.state, s.action);
    }
  }
  logs.close();
}
