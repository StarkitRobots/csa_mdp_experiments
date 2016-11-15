#include "policies/expert_approach.h"

namespace csa_mdp
{

const int ExpertApproach::nb_parameters = 15;

ExpertApproach::ExpertApproach()
  : type(Type::cartesian),
    current_state(State::far),
    step_max(0.04),
    max_rotate_radius(0.99),
    min_far_radius(0.72),
    radius(0.198),
    step_p(0.21),
    far_theta_p(0.26),
    rotate_theta_p(0.47),
    rotate_lateral_p(-0.044),
    near_theta_p(0.21),
    near_lateral_p(0.178),
    stop_y_near(0.10),
    max_y_near(0.35),
    wished_x(0.21),
    wished_y(0),
    target_theta_tol(0.072),//10 * M_PI / 180),
    ball_theta_tol(0.29)//10 * M_PI / 180)
{
}

void ExpertApproach::init()
{
  current_state = State::far;
}
  
Eigen::VectorXd ExpertApproach::getConfig() const
{
  Eigen::VectorXd params(nb_parameters);
  params <<
    max_rotate_radius,
    min_far_radius,
    radius,
    step_p,
    far_theta_p,
    rotate_theta_p,
    rotate_lateral_p,
    near_theta_p,
    near_lateral_p,
    stop_y_near,
    max_y_near,
    wished_x,
    wished_y,
    target_theta_tol,
    ball_theta_tol;
  return params;
}
void ExpertApproach::setConfig(Type newType, const Eigen::VectorXd& params)
{
  if (params.size() != nb_parameters) {
    throw std::logic_error(
      "ExpertApproach::setConfig Invalid parameters size");
  }
  type = newType;
  max_rotate_radius = params(0);
  min_far_radius = params(1);
  radius = params(2);
  step_p = params(3);
  far_theta_p = params(4);
  rotate_theta_p = params(5);
  rotate_lateral_p = params(6);
  near_theta_p = params(7);
  near_lateral_p = params(8);
  stop_y_near = params(9);
  max_y_near = params(10);
  wished_x = params(11);
  wished_y = params(12);
  target_theta_tol = params(13);
  ball_theta_tol = params(14);

}

Eigen::VectorXd ExpertApproach::getRawAction(const Eigen::VectorXd &state)
{
  // Properties
  double ball_x, ball_y;
  switch(type) {
    case Type::cartesian:
      ball_x = state(0);
      ball_y = state(1);
      break;
    case Type::polar:
      ball_x = cos(state(1)) * state(0);
      ball_y = sin(state(1)) * state(0);
      break;
    default:
      throw std::logic_error("Unknown type for ExpertApproach");
  }
  double ball_azimuth = atan2(ball_y,ball_x);
  double ball_distance = std::sqrt(ball_x * ball_x + ball_y * ball_y);
  double target_angle = state(2);

  // Alignements
  bool good_align_goal = std::fabs(target_angle) < target_theta_tol;
  bool good_align_ball = std::fabs(ball_azimuth) < ball_theta_tol;

  // Errors
  float x_error = ball_x - wished_x;
  float y_error = ball_y - wished_y;

  // Updating state
  if (current_state == State::far && ball_distance < min_far_radius)
  {
    current_state = State::rotate;
  }
  if (ball_distance > max_rotate_radius)
  {
    current_state = State::far;
  }
  if (current_state == State::rotate && good_align_goal && good_align_ball)
  {
    current_state = State::near;
  }
  if (current_state == State::near && ball_y > max_y_near)
  {
    current_state = State::rotate;
  }

  // Computing command
  Eigen::VectorXd wished_cmd = Eigen::VectorXd::Zero(3);
  switch(current_state)
  {
    case State::far:
      wished_cmd(0) = std::max(0.0, cos(ball_azimuth) * step_max);
      wished_cmd(2) = far_theta_p * ball_azimuth;
      break;
    case State::rotate:
      wished_cmd(0) = step_p * (ball_distance - radius);
      wished_cmd(1) = rotate_lateral_p * target_angle;
      wished_cmd(2) = rotate_theta_p * ball_azimuth;
      break;
    case State::near:
      // Turn for alignements only if goal is entirely wrong
      if (!good_align_goal)
      {
        wished_cmd(2) = near_theta_p * target_angle;
      }
      // Move forward / backward if error along y is lower than a threshold
      if (std::fabs(y_error) < stop_y_near)
      {
        wished_cmd(0) = step_p * x_error;
      }
      wished_cmd(1) = near_lateral_p * y_error;
      break;
  }

  Eigen::VectorXd delta_cmd = wished_cmd - state.segment(3,3);

  return delta_cmd;
}

Eigen::VectorXd ExpertApproach::getRawAction(const Eigen::VectorXd &state,
                                             std::default_random_engine * external_engine) const
{
  (void) state;
  (void) external_engine;
  throw std::logic_error("Impossible to retrieve an action from an expert approach without modifications (due to state memory)");
}

void ExpertApproach::to_xml(std::ostream & out) const
{
  rosban_utils::xml_tools::write<std::string>("type", to_string(type), out);
}

void ExpertApproach::from_xml(TiXmlNode * node)
{
  std::string type_str;
  rosban_utils::xml_tools::try_read<std::string>(node, "type", type_str);
  if (type_str != "") {
    type = loadType(type_str);
  }
  // Reading vector list of parameters
  std::vector<double> params_read;
  rosban_utils::xml_tools::try_read_vector<double>(node, "params", params_read);
  // If number of coefficients is appropriate, update config
  if (params_read.size() == nb_parameters)
  {
    Eigen::VectorXd new_params = Eigen::Map<Eigen::VectorXd>(params_read.data(), nb_parameters);
    setConfig(type, new_params);
  }
  // Else throw an explicit error
  else {
    std::ostringstream oss;
    oss << "ExpertApproach::from_xml: invalid number of parameters in node 'params': "
        << "read: " << params_read.size() << ", expecting: " << nb_parameters;
    throw std::runtime_error(oss.str());
  }
}

ExpertApproach::Type ExpertApproach::loadType(const std::string & type_str)
{
  if (type_str == "cartesian") return Type::cartesian;
  if (type_str == "polar") return Type::polar;
  throw std::runtime_error("Unknown ExpertApproach::Type: '" + type_str + "'");
}

std::string ExpertApproach::class_name() const
{ return "expert_approach"; }

std::string to_string(ExpertApproach::State state)
{
  switch(state)
  {
    case ExpertApproach::State::near: return "near";
    case ExpertApproach::State::far: return "far";
    case ExpertApproach::State::rotate: return "rotate";
  }
  throw std::runtime_error("Invalid ExpertApproach::State in to_string");
}

std::string to_string(ExpertApproach::Type type)
{
  switch(type)
  {
    case ExpertApproach::Type::cartesian: return "cartesian";
    case ExpertApproach::Type::polar: return "polar";
  }
  throw std::runtime_error("Invalid ExpertApproach::Type in to_string");
}

}
