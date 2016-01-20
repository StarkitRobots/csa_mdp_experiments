#include "problems/double_integrator.h"

#include "rosban_csa_mdp/solvers/mre.h"

#include <iostream>

using csa_mdp::FPF;
using csa_mdp::MRE;

int main()
{
  DoubleIntegrator di(DoubleIntegrator::Version::Weinstein2012);

  // Exploration size
  int nb_trajectories = 1000;
  int trajectory_max_length = 200;

  // MRE properties
  int max_points = 4;
  double reward_max = 0;
  int plan_period = 10;

  // FPF properties
  FPF::Config fpf_conf;
  fpf_conf.setStateLimits(di.getStateLimits());
  fpf_conf.setActionLimits(di.getActionLimits());
  fpf_conf.horizon = 10;
  fpf_conf.discount = 0.98;
  fpf_conf.max_action_tiles = 40;
  fpf_conf.q_value_conf.k = 3;
  fpf_conf.q_value_conf.n_min = 1;
  fpf_conf.q_value_conf.nb_trees = 25;
  fpf_conf.q_value_conf.min_var = std::pow(10, -4);
  fpf_conf.q_value_conf.appr_type = regression_forests::ApproximationType::PWC;
  fpf_conf.policy_samples = 100;
  fpf_conf.policy_conf.k = 2;
  fpf_conf.policy_conf.n_min = 25;
  fpf_conf.policy_conf.nb_trees = 25;
  fpf_conf.policy_conf.min_var = std::pow(10, -4);
  fpf_conf.policy_conf.appr_type = regression_forests::ApproximationType::PWL;

  MRE mre(di.getStateLimits(),
          di.getActionLimits(),
          max_points,
          reward_max,
          plan_period,
          fpf_conf,
          [di](const Eigen::VectorXd &state) { return di.isTerminal(state); });

  std::cout << "trajectory_idx,trajectory_reward,cumulative_reward" << std::endl;

  double cumulative_reward = 0;
  for (int trajectory_idx = 1; trajectory_idx <= nb_trajectories; trajectory_idx++)
  {
    // Setting up initial state and variables
    double trajectory_reward = 0;
    Eigen::VectorXd state(2);
    state << 1, 0;
    int steps = 0;
    double gain = 1.0;
    // While trajectory is smaller than the maximal number of steps and state is not terminal
    while (steps < trajectory_max_length && !di.isTerminal(state))
    {
      Eigen::VectorXd action = mre.getAction(state);
      csa_mdp::Sample sample = di.getSample(state, action);
      // Updating MRE + traj_reward
      mre.feed(state, action, sample.reward);
      trajectory_reward += sample.reward * gain;
      // Updating state and gain and steps
      gain = gain * fpf_conf.discount;
      state = sample.next_state;
      steps++;
    }
    mre.endTrajectory();
    cumulative_reward += trajectory_reward;
    // Writing down values
    std::cout << trajectory_idx << ","
              << trajectory_reward << ","
              << cumulative_reward << std::endl;

  }
}
