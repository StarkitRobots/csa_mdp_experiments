#include "problems/cart_pole.h"

CartPole::CartPole()
  : max_pos(1), max_vel(5), max_torque(20), max_axis_vel(20),
    start_cart_pos_tol(0.05),
    start_cart_vel_tol(0.01),
    start_axis_pos_tol(M_PI/180),
    start_axis_vel_tol(0.01)
{
  updateLimits();
}

void CartPole::updateLimits()
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

  setStateNames({"cart_pos", "cart_speed", "theta", "omega"});
  setActionNames({"cart_cmd"});
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
  bool binary_reward = false;
  if (binary_reward)
  {
    bool pole_ok = std::fabs(dst(2)) < M_PI / 10;
    bool cart_ok = true;//std::fabs(dst(0)) < max_pos / 10;
    if ( cart_ok && pole_ok) return 0;
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
  static double kd = 10;
  Eigen::VectorXd cmd(1);
  cmd(0) = - (kp * state(0) + kd * state(1));
  // Avoid to stay stable with pendulum vertical
  if (std::fabs(state(2)) < M_PI / 10 &&
      std::fabs(state(3)) < 0.001)
  {
    cmd(0) = max_torque;
  }
  // Ensuring that limits are respected
  cmd(0) = std::min(max_torque, std::max(-max_torque, cmd(0)));
  return cmd;
}

void CartPole::to_xml(std::ostream & out) const
{
  rosban_utils::xml_tools::write<double>("max_pos"           , max_pos           , out);
  rosban_utils::xml_tools::write<double>("max_vel"           , max_vel           , out);
  rosban_utils::xml_tools::write<double>("max_torque"        , max_torque        , out);
  rosban_utils::xml_tools::write<double>("max_axis_vel"      , max_axis_vel      , out);
  rosban_utils::xml_tools::write<double>("start_cart_pos_tol", start_cart_pos_tol, out);
  rosban_utils::xml_tools::write<double>("start_cart_vel_tol", start_cart_vel_tol, out);
  rosban_utils::xml_tools::write<double>("start_axis_pos_tol", start_axis_pos_tol, out);
  rosban_utils::xml_tools::write<double>("start_axis_vel_tol", start_axis_vel_tol, out);
}

void CartPole::from_xml(TiXmlNode * node)
{
  rosban_utils::xml_tools::try_read<double>(node, "max_pos"           , max_pos           );
  rosban_utils::xml_tools::try_read<double>(node, "max_vel"           , max_vel           );
  rosban_utils::xml_tools::try_read<double>(node, "max_torque"        , max_torque        );
  rosban_utils::xml_tools::try_read<double>(node, "max_axis_vel"      , max_axis_vel      );
  rosban_utils::xml_tools::try_read<double>(node, "start_cart_pos_tol", start_cart_pos_tol);
  rosban_utils::xml_tools::try_read<double>(node, "start_cart_vel_tol", start_cart_vel_tol);
  rosban_utils::xml_tools::try_read<double>(node, "start_axis_pos_tol", start_axis_pos_tol);
  rosban_utils::xml_tools::try_read<double>(node, "start_axis_vel_tol", start_axis_vel_tol);
  updateLimits();
}

std::string CartPole::class_name() const
{
  return "cart_pole";
}
