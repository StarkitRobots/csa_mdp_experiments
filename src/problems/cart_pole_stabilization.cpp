#include "problems/cart_pole_stabilization.h"

#include "rosban_random/tools.h"

#include <iostream>

namespace csa_mdp
{

// Above this threshold, trial is a failure
double CartPoleStabilization::theta_max = M_PI / 2;//[rad]
// While this limit is not directly given in the article, this value
// is used for n_{dttheta} (page 5 bottom right colum)
double CartPoleStabilization::omega_max = 2;//[rad/s]
// limit of allowed actions
double CartPoleStabilization::action_max = 50;//[N]
double CartPoleStabilization::noise_max = 10;//[N]

double CartPoleStabilization::integration_step = 0.001;// Information are required here
double CartPoleStabilization::simulation_step  = 0.1;
double CartPoleStabilization::pendulum_mass    = 2.0;
double CartPoleStabilization::cart_mass        = 6.0;
double CartPoleStabilization::pendulum_length  = 0.5;
double CartPoleStabilization::g                = 9.8;


CartPoleStabilization::CartPoleStabilization()
  : learning_space(LearningSpace::Angular)
{
  Eigen::MatrixXd state_limits = Eigen::MatrixXd(4,2);
  state_limits(0,0) = -theta_max;
  state_limits(0,1) =  theta_max;
  state_limits(1,0) = -omega_max;
  state_limits(1,1) =  omega_max;
  state_limits(2,0) = -1;
  state_limits(2,1) =  1;
  state_limits(3,0) = -1;
  state_limits(3,1) =  1;
  Eigen::MatrixXd action_limits = Eigen::MatrixXd(1,2);
  action_limits(0,0) = -action_max;
  action_limits(0,1) =  action_max;

  setStateLimits(state_limits);
  setActionLimits({action_limits});
  setStateNames({"theta","omega","cos(theta)","sin(theta)"});
  setActionNames(0,{"torque"});
}

bool CartPoleStabilization::isTerminal(const Eigen::VectorXd& state) const
{
  Eigen::VectorXd state_full = whateverToFull(state);
  for (int i = 0; i < 2; i++)
  {
    if (state_full(i) < getStateLimits()(i,0) || state_full(i) > getStateLimits()(i,1))
      return true;
  }
  return false;
}

double CartPoleStabilization::getReward(const Eigen::VectorXd &action,
                                        const Eigen::VectorXd &result) const
{
  Eigen::VectorXd result_full = whateverToFull(result);
  if (isTerminal(result_full)) {
    return -1000;
  }
  double pos_cost   = std::pow(result_full(0) / theta_max , 2);
  double speed_cost = std::pow(result_full(1)             , 2);
  double force_cost = std::pow(action(1) / action_max, 2);
  return - (pos_cost + speed_cost + force_cost);
}

std::vector<int> CartPoleStabilization::getLearningDimensions() const
{
  switch(learning_space)
  {
    case LearningSpace::Angular: return {0,1};
    case LearningSpace::Cartesian: return {2,3,1};
    case LearningSpace::Full: return {0,1,2,3};
  }
  throw std::runtime_error("CartPoleStabilization::getLearningDimensions: unknown learning_space");
}

Problem::Result CartPoleStabilization::getSuccessor(const Eigen::VectorXd &state,
                                                    const Eigen::VectorXd &action,
                                                    std::default_random_engine * engine) const
{
  Result r;
  switch (detectSpace(state))
  {
    case LearningSpace::Full:
      r.successor = getFullSuccessor(state, action, engine);
      break;
    case LearningSpace::Angular:
      r.successor = getAngularSuccessor(state, action, engine);
      break;
    case LearningSpace::Cartesian:
      r.successor = getCartesianSuccessor(state, action, engine);
      break;
    default:
      throw std::logic_error("CartPoleStabilization::getSuccessor: Unhandled learning space");
  }
  r.reward = getReward(action,r.successor);
  r.terminal = isTerminal(r.successor);
  return r;
}

Eigen::VectorXd CartPoleStabilization::getFullSuccessor(const Eigen::VectorXd &state,
                                                        const Eigen::VectorXd &action,
                                                        std::default_random_engine * engine) const
{
  std::uniform_real_distribution<double> noise_distribution(-noise_max, noise_max);
  // Adding noise to action
  double noisy_action = action(1) + noise_distribution(*engine);
  // Integrating action with the system dynamics
  double elapsed = 0;
  Eigen::VectorXd current_state = state;
  while (elapsed < simulation_step)
  {
    double dt = std::min(simulation_step - elapsed, integration_step);
    double th = current_state(0);
    double dt_th = current_state(1);
    double dt_th2 = dt_th * dt_th;
    double alpha = 1 / (pendulum_mass + cart_mass);
    double acc =
      (g * sin(th)
       - alpha * pendulum_mass * pendulum_length * dt_th2 * sin(2 * th) / 2
       - alpha * cos(th) * noisy_action)
      / (4 * pendulum_length / 3 - alpha * pendulum_mass * pendulum_length * std::pow(cos(th),2));
    Eigen::VectorXd next_state(4);
    next_state(0) = th + dt * dt_th;
    next_state(1) = dt_th + dt * acc;
    next_state(2) = cos(next_state(0));
    next_state(3) = sin(next_state(0));
    elapsed += dt;
    current_state = next_state;
  }
  return current_state;
}

Eigen::VectorXd
CartPoleStabilization::getAngularSuccessor(const Eigen::VectorXd &state,
                                           const Eigen::VectorXd &action,
                                           std::default_random_engine * engine) const
{
  return getFullSuccessor(angularToFull(state), action, engine).segment(0,2);
}

Eigen::VectorXd
CartPoleStabilization::getCartesianSuccessor(const Eigen::VectorXd &state,
                                             const Eigen::VectorXd &action,
                                             std::default_random_engine * engine) const
{
  Eigen::VectorXd full_successor = getFullSuccessor(cartesianToFull(state), action, engine);
  Eigen::VectorXd partial_successor(3);
  partial_successor.segment(0,2) = full_successor.segment(2,2);
  partial_successor(2) = full_successor(1);
  return partial_successor;
}

CartPoleStabilization::LearningSpace
CartPoleStabilization::detectSpace(const Eigen::VectorXd & state) const
{
  // Check if dimensions are appropriate
  switch(state.rows())
  {
    case 2: return LearningSpace::Angular;
    case 3: return LearningSpace::Cartesian;
    case 4: return LearningSpace::Full;
    default:
    {
      std::ostringstream oss;
      oss << "CartPoleStabilization::detectSpace: Unexpected dim for state: " << state.rows();
      throw std::logic_error(oss.str());
    }
  }  
}

Eigen::VectorXd CartPoleStabilization::whateverToFull(const Eigen::VectorXd & state) const
{
  switch(detectSpace(state)) {
    case LearningSpace::Angular: return angularToFull(state);
    case LearningSpace::Cartesian: return cartesianToFull(state);
    case LearningSpace::Full: return state;
  }
  throw std::runtime_error("CartPoleStabilization::whateverToFull: unknown space");
}

Eigen::VectorXd CartPoleStabilization::angularToFull(const Eigen::VectorXd & state) const
{
  Eigen::VectorXd full_state(4);
  full_state.segment(0,2) = state;
  full_state(2) = cos(state(2));
  full_state(3) = sin(state(2));
  return full_state;
}

Eigen::VectorXd CartPoleStabilization::cartesianToFull(const Eigen::VectorXd & state) const
{
  Eigen::VectorXd full_state(4);
  full_state(0) = atan2(state(3), state(2));//theta
  full_state(1) = state(2);//omega
  full_state.segment(2,2) = state.segment(0,2);//cos(theta), sin(theta)
  return full_state;
}


Eigen::VectorXd CartPoleStabilization::getStartingState(std::default_random_engine * engine) const
{
  (void) engine;
  Eigen::VectorXd state = Eigen::VectorXd::Zero(4);
  state(2) = cos(state(0));
  state(3) = sin(state(0));
  return state;
}

Json::Value CartPoleStabilization::toJson() const
{
  Json::Value v;
  v["learning_space"] = to_string(learning_space);
  return v;
}
void CartPoleStabilization::fromJson(const Json::Value & v, const std::string & dir_name)
{
  (void)dir_name;
  std::string learning_space_str;
  rhoban_utils::tryRead(v, "learning_space", &learning_space_str);
  if (learning_space_str != "") {
    learning_space = loadLearningSpace(learning_space_str);
  }
}

std::string CartPoleStabilization::getClassName() const
{
  return "cart_pole_stabilization";
}

CartPoleStabilization::LearningSpace
CartPoleStabilization::loadLearningSpace(const std::string & str)
{
  if (str == "Angular") {
    return CartPoleStabilization::LearningSpace::Angular;
  }
  if (str == "Cartesian") {
    return CartPoleStabilization::LearningSpace::Cartesian;
  }
  if (str == "Full") {
    return CartPoleStabilization::LearningSpace::Full;
  }
  std::cerr << "Failed to load learning space" << std::endl;
  throw std::runtime_error("CartPoleStabilization::loadLearningSpace: unknown learning space: '"
                           + str + "'");
}

std::string to_string(CartPoleStabilization::LearningSpace learning_space)
{
  switch(learning_space)
  {
    case CartPoleStabilization::LearningSpace::Angular: return "Angular";
    case CartPoleStabilization::LearningSpace::Cartesian: return "Cartesian";
    case CartPoleStabilization::LearningSpace::Full: return "Full";
  }
  throw std::runtime_error("to_string(CartPoleStabilization::LearningSpace): unknown learning space");
}

}
