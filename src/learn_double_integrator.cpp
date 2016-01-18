#include "problems/double_integrator.h"

#include "rosban_csa_mdp/solvers/fpf.h"

using csa_mdp::FPF;

int main()
{
  DoubleIntegrator di;
  Eigen::VectorXd initial_state(2);
  initial_state << 1, 0;
  int trajectory_max_length = 200;
  int nb_trajectories = 200;
  std::vector<csa_mdp::Sample> mdp_samples = di.getRandomBatch(initial_state,
                                                               trajectory_max_length,
                                                               nb_trajectories);
  FPF::Config conf;
  conf.setStateLimits(di.getStateLimits());
  conf.setActionLimits(di.getActionLimits());
  conf.horizon = 10;
  conf.discount = 0.98;
  conf.max_action_tiles = 40;
  conf.q_value_conf.k = 3;
  conf.q_value_conf.n_min = 1;
  conf.q_value_conf.nb_trees = 25;
  conf.q_value_conf.min_var = std::pow(10, -4);
  conf.q_value_conf.appr_type = regression_forests::ApproximationType::PWC;
  conf.policy_samples = 10000;
  conf.policy_conf.k = 2;
  conf.policy_conf.n_min = 1500;
  conf.policy_conf.nb_trees = 25;
  conf.policy_conf.min_var = std::pow(10, -4);
  conf.policy_conf.appr_type = regression_forests::ApproximationType::PWL;
  FPF solver;
  solver.conf = conf;
  auto is_terminal = [di](const Eigen::VectorXd &state){return di.isTerminal(state);};
  solver.solve(mdp_samples, is_terminal);
  solver.getValueForest().save("/tmp/test_di_values.data");
  solver.getPolicyForest(0).save("/tmp/test_di_policy.data");
}
