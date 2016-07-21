#include "problems/simulated_cart_pole.h"

SimulatedCartPole::SimulatedCartPole()
  : max_pos(1), max_vel(5), max_torque(20), max_axis_vel(20),
    pole_length(0.3), cart_mass(0.5), pendulum_mass(0.5),
    friction(0.1),
    gravity(9.82),//although this value seems weird, it's the default in pilco
    integration_step(0.001), simulation_step(0.1),
    reward_type(RewardType::Pilco)
{
  updateLimits();
}

void SimulatedCartPole::updateLimits()
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

  setStateNames({"cart_pos", "cart_vel", "theta", "omega"});
  setActionNames({"cart_cmd"});
}

bool SimulatedCartPole::isTerminal(const Eigen::VectorXd & state) const
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

double SimulatedCartPole::getReward(const Eigen::VectorXd &state,
                           const Eigen::VectorXd &action,
                           const Eigen::VectorXd &dst)
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
  throw std::runtime_error("Unkown reward_type in SimulatedCartPole");
}

Eigen::VectorXd SimulatedCartPole::getSuccessor(const Eigen::VectorXd & state,
                                                const Eigen::VectorXd & action)
{
  // Integrating action with the system dynamics
  double elapsed = 0;
  Eigen::VectorXd current_state = state;
  while (elapsed < simulation_step)
  {
    // Defining short names for variables
    double dt = std::min(simulation_step - elapsed, integration_step);
    double vel = current_state(1);
    double theta = current_state(2);
    double omega = current_state(3);
    double omega2 = omega * omega;
    double sin_t = sin(theta + M_PI);//Adding pi because in pilco, 0 has not the same meaning
    double cos_t = cos(theta + M_PI);//Adding pi because in pilco, 0 has not the same meaning
    double cos_t2 = cos_t * cos_t;
    double M = cart_mass;
    double m = pendulum_mass;
    double l = pole_length;
    double u = action(0);
    double f = friction;
    double g = gravity;
    // Computing gradient
    Eigen::VectorXd grad = current_state;
    grad(0) = vel;
    grad(1) = ( 2*m*l*omega2*sin_t + 3*m*g*sin_t*cos_t
                + 4*u - 4*f*vel )/( 4*(M+m)-3*m*cos_t2 );
    grad(2) = omega;
    grad(3) =  (-3*m*l*omega2*sin_t*cos_t - 6*(M+m)*g*sin_t
                - 6*(u-f*vel)*cos_t )/( 4*l*(m+M)-3*m*l*cos_t2 );
    // Updating state
    Eigen::VectorXd next_state = current_state + grad * dt;
    elapsed += dt;
    current_state = next_state;
  }
  return current_state;
}

Eigen::VectorXd SimulatedCartPole::getStartingState()
{
  Eigen::VectorXd state = Eigen::VectorXd::Zero(4);
  state(3) = M_PI;
  return state;
}

void SimulatedCartPole::to_xml(std::ostream & out) const
{
  rosban_utils::xml_tools::write<double>("max_pos"           , max_pos               , out);
  rosban_utils::xml_tools::write<double>("max_vel"           , max_vel               , out);
  rosban_utils::xml_tools::write<double>("max_torque"        , max_torque            , out);
  rosban_utils::xml_tools::write<double>("max_axis_vel"      , max_axis_vel          , out);
  rosban_utils::xml_tools::write<double>("pole_length"       , pole_length           , out);
  rosban_utils::xml_tools::write<double>("cart_mass"         , cart_mass             , out);
  rosban_utils::xml_tools::write<double>("pendulum_mass"     , pendulum_mass         , out);
  rosban_utils::xml_tools::write<double>("friction"          , friction              , out);
  rosban_utils::xml_tools::write<double>("gravity"           , gravity               , out);
  rosban_utils::xml_tools::write<double>("integration_step"  , integration_step      , out);
  rosban_utils::xml_tools::write<double>("simulation_step"   , simulation_step       , out);
  rosban_utils::xml_tools::write<std::string>("reward_type  ", to_string(reward_type), out);
}

void SimulatedCartPole::from_xml(TiXmlNode * node)
{
  rosban_utils::xml_tools::try_read<double>(node, "max_pos"           , max_pos           );
  rosban_utils::xml_tools::try_read<double>(node, "max_vel"           , max_vel           );
  rosban_utils::xml_tools::try_read<double>(node, "max_torque"        , max_torque        );
  rosban_utils::xml_tools::try_read<double>(node, "max_axis_vel"      , max_axis_vel      );
  rosban_utils::xml_tools::try_read<double>(node, "pole_length"       , pole_length       );
  rosban_utils::xml_tools::try_read<double>(node, "cart_mass"         , cart_mass         );
  rosban_utils::xml_tools::try_read<double>(node, "pendulum_mass"     , pendulum_mass     );
  rosban_utils::xml_tools::try_read<double>(node, "friction"          , friction          );
  rosban_utils::xml_tools::try_read<double>(node, "gravity"           , gravity           );
  rosban_utils::xml_tools::try_read<double>(node, "integration_step"  , integration_step  );
  rosban_utils::xml_tools::try_read<double>(node, "simulation_step"   , simulation_step   );
  std::string reward_type_str;
  rosban_utils::xml_tools::try_read<std::string>(node, "reward_type", reward_type_str);
  if (reward_type_str != "")
  {
    reward_type =  loadRewardType(reward_type_str);
  }
  updateLimits();
}

std::string SimulatedCartPole::class_name() const
{
  return "cart_pole";
}

std::string to_string(SimulatedCartPole::RewardType type)
{
  switch (type)
  {
    case SimulatedCartPole::RewardType::Binary: return "binary";
    case SimulatedCartPole::RewardType::Continuous: return "continuous";
    case SimulatedCartPole::RewardType::Pilco: return "pilco";
  }
  throw std::runtime_error("Unknown type in to_string(Type)");
}

SimulatedCartPole::RewardType SimulatedCartPole::loadRewardType(const std::string &type)
{
  if (type == "binary"    ) return SimulatedCartPole::RewardType::Binary;
  if (type == "continuous") return SimulatedCartPole::RewardType::Continuous;
  if (type == "pilco"     ) return SimulatedCartPole::RewardType::Pilco;
  throw std::runtime_error("Unknown SimulatedCartPole::RewardType: '" + type + "'");
}
