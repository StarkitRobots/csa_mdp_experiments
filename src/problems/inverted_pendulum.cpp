#include "problems/inverted_pendulum.h"

double InvertedPendulum::max_torque = 2.25;
double InvertedPendulum::max_axis_vel = 6;
double InvertedPendulum::start_pos_tol = M_PI / 180;// 1 deg of tol
double InvertedPendulum::start_vel_tol = M_PI / 180;// 1 deg/s of tol

InvertedPendulum::InvertedPendulum()
{
  Eigen::MatrixXd state_limits(2,2), action_limits(1,2);
  state_limits <<
    -M_PI, M_PI,
    -max_axis_vel, max_axis_vel;
  action_limits << -max_torque, max_torque;
  setStateLimits(state_limits);
  setActionLimits(action_limits);
}

bool InvertedPendulum::isTerminal(const Eigen::VectorXd & state) const
{
  if (std::fabs(state(1)) > max_axis_vel) return true;
  return false;
}

double InvertedPendulum::getReward(const Eigen::VectorXd &state,
                                   const Eigen::VectorXd &action,
                                   const Eigen::VectorXd &dst)
{
  (void) action;
  if (isTerminal(dst) || isTerminal(state)) {
    return -50;
  }
  double pos_cost = std::fabs(dst(0) / M_PI);
  double torque_cost = std::pow(action(0) / max_torque, 2);
  return -(pos_cost + torque_cost);
}

Eigen::VectorXd InvertedPendulum::getSuccessor(const Eigen::VectorXd & state,
                                               const Eigen::VectorXd & action)
{
  (void) state;
  (void) action;
  throw std::runtime_error("Not implemented");
}

bool InvertedPendulum::isValidStart(const Eigen::VectorXd &state) const
{
  bool pos_ok = std::fabs(state(0)) > M_PI - start_pos_tol; 
  bool vel_ok = std::fabs(state(1)) < start_vel_tol;
  return pos_ok && vel_ok;
}

Eigen::VectorXd InvertedPendulum::getResetCmd(const Eigen::VectorXd &state) const
{
  (void)state;
  Eigen::VectorXd cmd(1);
  cmd(0) = 0;
  return cmd;
}
