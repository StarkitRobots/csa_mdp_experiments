#include "rosban_csa_mdp/core/history.h"
#include "rosban_csa_mdp/solvers/fpf.h"

#include <ros/ros.h>

using csa_mdp::History;
using csa_mdp::Problem;
using csa_mdp::FPF;


int main(int argc, char ** argv)
{
  std::string log_path = ros::getROSArg(argc, argv, "log_path");
  if (log_path == "")
  {
    std::cerr << "Usage: rosrun .... log_path:=<file>" << std::endl;
    exit(EXIT_FAILURE);
  }

  /// CONFIG

  // Limits:
  double pos_max = M_PI;
  double vel_max = 4 * M_PI;// rad / s
  double torque_max = 2.0;// N * m
  Eigen::MatrixXd state_limits(2,2), action_limits(1,2);
  state_limits << -pos_max, pos_max, -vel_max, vel_max;
  action_limits << -torque_max, torque_max;

  // Learning config
  FPF::Config conf;
  conf.setStateLimits(state_limits);
  conf.setActionLimits(action_limits);
  conf.horizon = 20;
  conf.discount = 0.95;
  conf.max_action_tiles = 50;
  conf.q_value_conf.k = 3;
  conf.q_value_conf.n_min = 10;
  conf.q_value_conf.nb_trees = 25;
  conf.q_value_conf.min_var = std::pow(10, -8);
  conf.q_value_conf.appr_type = regression_forests::ApproximationType::PWC;
  conf.policy_samples = 10000;
  conf.policy_conf.k = 2;
  conf.policy_conf.n_min = 100;
  conf.policy_conf.nb_trees = 25;
  conf.policy_conf.min_var = std::pow(10, -2);
  conf.policy_conf.appr_type = regression_forests::ApproximationType::PWL;

  // file format: time, pos, vel, eff, cmd
  std::vector<size_t> state_index = {1,2};// {pos,vel}
  std::vector<size_t> action_index = {4};// {cmd}

  // Specifying reward function
  Problem::RewardFunction reward_func = [pos_max, torque_max](const Eigen::VectorXd &src,
                                                              const Eigen::VectorXd &action,
                                                              const Eigen::VectorXd &result)
    {
      (void)src;// Unused
      // Normalizing position if necessary
      double position = fmod(result(0), 2 * M_PI);
      if (position > M_PI)
      {
        position = position - 2 * M_PI;
      }
      double pos_cost   = std::pow(result(0) / pos_max   , 2);
      double speed_cost = std::pow(result(1)             , 2);
      double force_cost = std::pow(action(0) / torque_max, 2);
      return - (pos_cost + speed_cost + force_cost);
    };

  // Reading csv file
  History h = History::readCSV(log_path, state_index, action_index, reward_func, true);

  // Producing Samples
  std::vector<csa_mdp::Sample> samples = h.getBatch();

  FPF solver;
  solver.conf = conf;
  auto is_terminal = [](const Eigen::VectorXd &state){(void)state; return false;};
  solver.solve(samples, is_terminal);
  solver.getValueForest().save("/tmp/test_ip_sim_values.data");
  solver.getPolicyForest(0).save("/tmp/test_ip_sim_policy.data");


}
