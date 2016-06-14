#include "policies/expert_approach.h"

ExpertApproach::ExpertApproach()
  : type(Type::cartesian),
    current_state(State::far),
    step_max(0.04),
    max_rotate_radius(1),
    min_far_radius(0.75),
    radius(0.5),
    step_p(0.1),
    far_theta_p(0.2),
    rotate_theta_p(0.4),
    rotate_lateral_p(-0.06),
    near_theta_p(0.1),
    near_lateral_p(0.06),
    stop_y_near(0.125),
    max_y_near(0.25),
    wished_x(0.15),
    wished_y(0.0),
    target_theta_tol(15 * M_PI / 180),
    ball_theta_tol(15 * M_PI / 180)
{
}

void ExpertApproach::init()
{
  current_state = State::far;
}

Eigen::VectorXd ExpertApproach::getRawAction(const Eigen::VectorXd &state)
{
  // Properties
  double ball_x = state(0);
  double ball_y = state(1);
  double target_angle = state(2);
  double ball_azimuth = atan2(state(1),state(0));
  double ball_distance = std::sqrt(ball_x * ball_x + ball_y * ball_y);

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
