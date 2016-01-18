#include "problems/double_integrator.h"

#include "rosban_regression_forests/core/forest.h"

#include <ros/ros.h>

using regression_forests::Forest;

int main(int argc, char ** argv)
{
  std::string policy_path = ros::getROSArg(argc, argv, "policy_file");
  if (policy_path == "")
  {
    std::cerr << "Usage: rosrun .... policy_file:=<file>" << std::endl;
    exit(EXIT_FAILURE);
  }
  // Load policy
  std::unique_ptr<Forest> policy_forest = Forest::loadFile(policy_path);

  std::cout << "Policy forest has " << policy_forest->nbTrees() << " trees" << std::endl;

  auto policy = [&policy_forest](const Eigen::VectorXd &state)
    {
      Eigen::VectorXd action(1);
      action(0) = policy_forest->getValue(state);
      std::cout << "state: " << state.transpose() << std::endl;
      std::cout << "action: " << action.transpose() << std::endl;
      return action;
    };

  DoubleIntegrator di;
  Eigen::VectorXd initial_state(2);
  initial_state << 1, 0;
  std::vector<csa_mdp::Sample> samples;
  samples = di.simulateTrajectory(initial_state, 200, policy);

  double reward = 0;
  double disc = 0.98;
  double gain = 1;
  for (const auto &sample : samples)
  {
    std::cout << "gain: " << gain << std::endl;
    std::cout << "reward: " << reward << std::endl;
    std::cout << "sample.reward: " << sample.reward << std::endl;
    reward += sample.reward * gain;
    gain *= disc;
  }
  std::cout << "Trajectory length: " << samples.size() << std::endl;
  std::cout << "Total discounted reward: " << reward << std::endl;
}
