#include "problems/cart_pole.h"

double CartPole::max_pos = 1;
double CartPole::max_vel = 2.5;
double CartPole::max_torque = 10;
double CartPole::max_axis_vel = 15;
double CartPole::start_axis_pos_tol = M_PI / 180;
double CartPole::start_axis_vel_tol = 0.01;
double CartPole::start_cart_pos_tol = 0.05;
double CartPole::start_cart_vel_tol = 0.01;

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
  const Eigen::MatrixXd & state_limits = getStateLimits();
  // Getting out of range
  for (int dim = 0; dim < state.rows(); dim++)
  {
    if (state(dim) > state_limits(dim,1) ||
        state(dim) < state_limits(dim,0))
    {
      return true;
    }
  }
  return false;
}

double CartPole::getReward(const Eigen::VectorXd &state,
                           const Eigen::VectorXd &action,
                           const Eigen::VectorXd &dst)
{
  (void) action;
  if (isTerminal(dst) || isTerminal(state)) {
    return -200;
  }
  bool binary_reward = true;
  if (binary_reward)
  {
    if (std::fabs(dst(2)) < M_PI / 12)
    {
      return 0;
    }
    return -1;
  }
  double cart_cost = std::pow(dst(0) / max_pos, 4);
  double poles_cost = 0;
  for (int i = 2; i < dst.rows(); i += 2)
  {
    poles_cost += std::pow(dst(i) / M_PI, 2);
    //poles_cost += std::fabs(dst(i) / M_PI);
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
  bool cart_pos_ok = std::fabs(state(0)) < start_cart_pos_tol; 
  bool cart_vel_ok = std::fabs(state(1)) < start_cart_vel_tol;
  double axis_error = M_PI - std::fabs(state(2));
  bool axis_pos_ok = axis_error < start_axis_pos_tol; 
  bool axis_vel_ok = std::fabs(state(3)) < start_axis_vel_tol;
  return axis_pos_ok && axis_vel_ok && cart_pos_ok && cart_vel_ok;
}

Eigen::VectorXd CartPole::getResetCmd(const Eigen::VectorXd &state) const
{
  static double kp = 10;
  static double kd = 5;
  Eigen::VectorXd cmd(1);
  cmd(0) = - (kp * state(0) + kd * state(1));
  return cmd;
}
