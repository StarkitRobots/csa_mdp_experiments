#include "problems/cart_pole_stabilization.h"

#include "rosban_regression_forests/tools/random.h"

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
{
  Eigen::MatrixXd state_limits = Eigen::MatrixXd(2,2);
  state_limits(0,0) = -theta_max;
  state_limits(0,1) =  theta_max;
  state_limits(1,0) = -omega_max;
  state_limits(1,1) =  omega_max;
  Eigen::MatrixXd action_limits = Eigen::MatrixXd(1,2);
  action_limits(0,0) = -action_max;
  action_limits(0,1) =  action_max;

  setStateLimits(state_limits);
  setActionLimits(action_limits);

  generator = regression_forests::get_random_engine();
  noise_distribution = std::uniform_real_distribution<double>(-noise_max, noise_max);
}

bool CartPoleStabilization::isTerminal(const Eigen::VectorXd& state) const
{
  for (int i = 0; i < 2; i++)
  {
    if (state(i) < getStateLimits()(i,0) || state(i) > getStateLimits()(i,1))
      return true;
  }
  return false;
}

double CartPoleStabilization::getReward(const Eigen::VectorXd &src,
                                        const Eigen::VectorXd &action,
                                        const Eigen::VectorXd &result)
{
  (void)src;//Unused
  if (isTerminal(result)) {
    return -1000;
  }
  double pos_cost   = std::pow(result(0) / theta_max , 2);
  double speed_cost = std::pow(result(1)             , 2);
  double force_cost = std::pow(action(0) / action_max, 2);
  return - (pos_cost + speed_cost + force_cost);
}

Eigen::VectorXd CartPoleStabilization::getSuccessor(const Eigen::VectorXd &state,
                                                    const Eigen::VectorXd &action)
{
  // Adding noise to action
  double noisy_action = action(0) + noise_distribution(generator);
  // Integrating action with the system dynamics
  double elapsed = 0;
  Eigen::Vector2d current_state = state;
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
    Eigen::Vector2d next_state;
    next_state(0) = th + dt * dt_th;
    next_state(1) = dt_th + dt * acc;
    elapsed += dt;
    current_state = next_state;
  }
  return current_state;
}

Eigen::VectorXd CartPoleStabilization::getStartingState()
{
  return Eigen::VectorXd::Zero(2);
}
