#include "problems/cart_pole.h"

CartPole::CartPole()
  : max_pos(1), max_vel(5), max_torque(20), max_axis_vel(20),
    start_cart_pos_tol(0.05),
    start_cart_vel_tol(0.01),
    start_axis_pos_tol(M_PI/180),
    start_axis_vel_tol(0.01),
    pole_length(0.3),
    reward_type(RewardType::Continuous)
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
                           const Eigen::VectorXd &dst) const
{
  (void) action;
  if (isTerminal(dst) || isTerminal(state)) {
    return -200;
  }
  double cart_pos = dst(0);
  double theta = dst(2);
  switch(reward_type)
  {
    case RewardType::Binary:
    {
      bool pole_ok = std::fabs(theta) < M_PI / 10;
      bool cart_ok = true;//std::fabs(dst(0)) < max_pos / 10;
      if ( cart_ok && pole_ok) return 0;
      return -1;
    }
    case RewardType::Continuous:
    {
      double cart_cost = std::pow(cart_pos / max_pos, 4);
      double poles_cost = std::pow(theta / M_PI, 2);
      return -(cart_cost + poles_cost);
    }
    case RewardType::Pilco:
    {
      double pole_x = cart_pos - sin(theta) * pole_length;
      double pole_y = cos(theta) * pole_length;
      double dx = pole_x;
      double dy = pole_y - pole_length;
      double d2 = dx * dx + dy * dy;
      double a = 1 / (pole_length * pole_length);//at distance 2 *pole_length, reward = e^{-1} - 1
      //std::cout << "pole_x : " << pole_x << std::endl
      //          << "pole_y : " << pole_y << std::endl
      //          << "dx : " << dx << std::endl
      //          << "dy : " << dy << std::endl
      //          << "d2 : " << d2 << std::endl
      //          << "a  : " << a << std::endl
      //          << "R : " << exp(-0.5 * d2 * a) - 1 << std::endl;
      return exp(-0.5 * d2 * a) - 1;
    }
  }
  throw std::runtime_error("Unkown reward_type in CartPole");
}

Eigen::VectorXd CartPole::getSuccessor(const Eigen::VectorXd & state,
                                       const Eigen::VectorXd & action,
                                       std::default_random_engine * engine) const
{
  (void) state;
  (void) action;
  (void) engine;
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
  rosban_utils::xml_tools::write<double>("max_pos"           , max_pos               , out);
  rosban_utils::xml_tools::write<double>("max_vel"           , max_vel               , out);
  rosban_utils::xml_tools::write<double>("max_torque"        , max_torque            , out);
  rosban_utils::xml_tools::write<double>("max_axis_vel"      , max_axis_vel          , out);
  rosban_utils::xml_tools::write<double>("start_cart_pos_tol", start_cart_pos_tol    , out);
  rosban_utils::xml_tools::write<double>("start_cart_vel_tol", start_cart_vel_tol    , out);
  rosban_utils::xml_tools::write<double>("start_axis_pos_tol", start_axis_pos_tol    , out);
  rosban_utils::xml_tools::write<double>("start_axis_vel_tol", start_axis_vel_tol    , out);
  rosban_utils::xml_tools::write<std::string>("reward_type  ", to_string(reward_type), out);
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
  std::string reward_type_str;
  rosban_utils::xml_tools::try_read<std::string>(node, "reward_type", reward_type_str);
  if (reward_type_str != "")
  {
    reward_type =  loadRewardType(reward_type_str);
  }
  updateLimits();
}

std::string CartPole::class_name() const
{
  return "cart_pole";
}

std::string to_string(CartPole::RewardType type)
{
  switch (type)
  {
    case CartPole::RewardType::Binary: return "binary";
    case CartPole::RewardType::Continuous: return "continuous";
    case CartPole::RewardType::Pilco: return "pilco";
  }
  throw std::runtime_error("Unknown type in to_string(Type)");
}

CartPole::RewardType loadRewardType(const std::string &type)
{
  if (type == "binary"    ) return CartPole::RewardType::Binary;
  if (type == "continuous") return CartPole::RewardType::Continuous;
  if (type == "pilco"     ) return CartPole::RewardType::Pilco;
  throw std::runtime_error("Unknown CartPole::RewardType: '" + type + "'");
}
