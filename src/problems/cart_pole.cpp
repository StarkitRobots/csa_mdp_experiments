#include "problems/cart_pole.h"

double CartPole::max_pos = 0.4;
double CartPole::max_vel = 10;
double CartPole::max_torque = 100;
double CartPole::max_axis_vel = 25;
double CartPole::start_pos_tol = 0.05;
double CartPole::start_vel_tol = 0.01;

CartPole::CartPole()
{
  Eigen::MatrixXd state_limits(4,2), action_limits(1,2);
  state_limits <<
    -max_pos, max_pos,
    -max_vel, max_vel,
    -M_PI, M_PI,
    -max_axis_vel, max_axis_vel;
  action_limits << -max_torque, max_torque;
  setStateLimits(state_limits);
  setActionLimits(action_limits);
}

bool CartPole::isTerminal(const Eigen::VectorXd & state) const
{
  // Cart is not allowed to go too close to the borders
  if (std::fabs(state(0)) > max_pos)
    return true;
  return false;
}

double CartPole::getReward(const Eigen::VectorXd &state,
                           const Eigen::VectorXd &action,
                           const Eigen::VectorXd &dst)
{
  (void) action;
  if (isTerminal(dst) || isTerminal(state)) {
    return -50;
  }
  double cart_cost = std::fabs(dst(0) / max_pos);
  double poles_cost = 0;
  for (int i = 2; i < dst.rows(); i += 2)
  {
    poles_cost += std::fabs(dst(i) / M_PI);
  }
  return -(cart_cost + poles_cost);
}

Eigen::VectorXd CartPole::getSuccessor(const Eigen::VectorXd & state,
                                       const Eigen::VectorXd & action)
{
  (void) state;
  (void) action;
  throw std::runtime_error("Not implemented");
}

bool CartPole::isValidStart(const Eigen::VectorXd &state) const
{
  bool pos_ok = std::fabs(state(0)) < start_pos_tol; 
  bool vel_ok = std::fabs(state(1)) < start_vel_tol;
  return pos_ok && vel_ok;
}

Eigen::VectorXd CartPole::getResetCmd(const Eigen::VectorXd &state) const
{
  static double kp = 10;
  static double kd = 5;
  Eigen::VectorXd cmd(1);
  cmd(0) = - (kp * state(0) + kd * state(1));
  return cmd;
}
