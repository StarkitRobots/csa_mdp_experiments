#include "rosban_csa_mdp/core/history.h"
#include "rosban_csa_mdp/solvers/fpf.h"

#include <ros/ros.h>

using csa_mdp::Sample;
using csa_mdp::Problem;
using csa_mdp::FPF;


int main(int argc, char ** argv)
{
  std::string samples_path = ros::getROSArg(argc, argv, "samples_path");
  if (samples_path == "")
  {
    std::cerr << "Usage: rosrun .... samples_path:=<file>" << std::endl;
    exit(EXIT_FAILURE);
  }

  /// CONFIG

  // Limits:
  double pos_max = M_PI;
  double vel_max = 3 * M_PI;// rad / s
  double torque_max = 2.5;// N * m
  Eigen::MatrixXd state_limits(2,2), action_limits(1,2);
  state_limits << -pos_max, pos_max, -vel_max, vel_max;
  action_limits << -torque_max, torque_max;

  // Learning config
  FPF::Config conf;
  conf.setStateLimits(state_limits);
  conf.setActionLimits(action_limits);
  conf.horizon = 40;
  conf.discount = 0.98;
  conf.max_action_tiles = 100;
  conf.q_value_conf.k = 3;
  conf.q_value_conf.n_min = 1;
  conf.q_value_conf.nb_trees = 25;
  conf.q_value_conf.min_var = std::pow(10, -8);
  conf.q_value_conf.appr_type = regression_forests::ApproximationType::PWC;
  conf.policy_samples = 10000;
  conf.policy_conf.k = 2;
  conf.policy_conf.n_min = 20;
  conf.policy_conf.nb_trees = 25;
  conf.policy_conf.min_var = std::pow(10, -4);
  conf.policy_conf.appr_type = regression_forests::ApproximationType::PWL;

  // file format: src_pos, src_vel, cmd, dst_pos, dst_vel, reward
  std::vector<size_t> src_state_index = {0,1};// {src_pos, src_vel}
  std::vector<size_t> action_index    = {2};  // {cmd}
  std::vector<size_t> dst_state_index = {3,4};// {dst_pos, dst_vel}
  int reward_col = 5;

  // Reading csv file
  std::vector<Sample> samples = Sample::readCSV(samples_path,
                                                src_state_index,
                                                action_index,
                                                dst_state_index,
                                                reward_col,
                                                true);

  FPF solver;
  solver.conf = conf;
  auto is_terminal = [](const Eigen::VectorXd &state){(void)state; return false;};
  solver.solve(samples, is_terminal);
  solver.getValueForest().save("/tmp/test_ip_sim_values.data");
  solver.getPolicyForest(0).save("/tmp/test_ip_sim_policy.data");


}
