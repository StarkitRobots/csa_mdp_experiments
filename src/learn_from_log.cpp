#include "rosban_csa_mdp/core/history.h"
#include "rosban_csa_mdp/solvers/fpf.h"

#include "problems/cart_pole.h"

#include <ros/ros.h>

using csa_mdp::History;
using csa_mdp::Problem;
using csa_mdp::FPF;


int main(int argc, char ** argv)
{
  std::string log_path = ros::getROSArg(argc, argv, "log_path");
  std::string problem_name = ros::getROSArg(argc, argv, "problem");
  std::string fpf_config = ros::getROSArg(argc, argv, "fpf_config");
  std::string output_path = ros::getROSArg(argc, argv, "output");
  if (log_path == "" || problem_name == "" || fpf_config == "" || output_path == "")
  {
    std::cerr << "Usage: rosrun ..."
              << " log_path:=<file>"
              << " problem:=<problem>"
              << " fpf_config:=<config>"
              << " output:=<path>"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  // Learning config
  FPF::Config conf;
  conf.load_file(fpf_config);

  // file format: time, pos, vel, eff, cmd
  std::vector<size_t> state_index;
  std::vector<size_t> action_index;

  csa_mdp::Problem * problem = NULL;
  if (problem_name == "CartPole")
  {
    problem = new CartPole();
    int dim = conf.getStateLimits().rows();
    for (int i = 0; i < dim; i++)
    {
      state_index.push_back(i + 2);
    }
    action_index.push_back(dim + 2);
  }
  else
  {
    throw std::runtime_error("The problem named '" + problem_name + "' is not handled yet");
  }

  Problem::RewardFunction reward_func = [problem](const Eigen::VectorXd &src,
                                                  const Eigen::VectorXd &action,
                                                  const Eigen::VectorXd &result)
    {
      return problem->getReward(src, action, result);
    };

  // Reading csv file
  History h = History::readCSV(log_path, state_index, action_index, reward_func, true);

  // Producing Samples
  std::vector<csa_mdp::Sample> samples = h.getBatch();

  FPF solver;
  solver.conf = conf;
  auto is_terminal = [problem](const Eigen::VectorXd &state){return problem->isTerminal(state);};
  solver.solve(samples, is_terminal);
  solver.conf.save_file(output_path + "config.xml");
  solver.getValueForest().save(output_path + "values_forest.data");
  for (int dim = 0; dim < solver.conf.getActionLimits().rows(); dim++)
  {
    solver.getPolicyForest(dim).save(output_path + "policy_forest_"+ std::to_string(dim) + ".data");
  }
  config.save_file();
}
