#include "policies/expert_approach.h"

#include "rosban_fa/linear_approximator.h"
#include "rosban_fa/orthogonal_split.h"

using namespace rosban_fa;

namespace csa_mdp
{

const int ExpertApproach::nb_parameters = 15;

ExpertApproach::ExpertApproach()
  : type(Type::cartesian),
    memory_state(State::far),
    lateral_kick(false),
    foot_y_offset(0.03),
    step_max(0.04),
    max_rotate_radius(1),
    min_far_radius(0.75),
    radius(0.5),
    step_p(0.1),
    far_theta_p(0.2),
    rotate_theta_p(0.3),
    rotate_lateral_p(-0.2),
    near_theta_p(0.1),
    near_lateral_p(0.06),
    stop_y_near(0.125),
    max_y_near(0.25),
    wished_x(0.22),
    wished_y(0.0),
    target_theta_tol(10 * M_PI / 180),
    ball_theta_tol(10 * M_PI / 180)
{
}

void ExpertApproach::init()
{
  memory_state = State::far;
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

Eigen::VectorXd ExpertApproach::getRawAction(const Eigen::VectorXd &state) {
  return getRawAction(state, &memory_state);
}

Eigen::VectorXd ExpertApproach::getRawAction(const Eigen::VectorXd &state,
                                             State * final_state) const
{
  State current_state = memory_state;
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

  // Errors
  float x_error = ball_x - wished_x;
  float y_error = ball_y - wished_y;

  // In lateral mode, target angle and y error change
  if (lateral_kick) {
    // If target is on the left, use right foot, else use left foot
    if (target_angle > 0) {
      target_angle -= M_PI/2;
      y_error += foot_y_offset;
    }
    else {
      target_angle += M_PI/2;
      y_error -= foot_y_offset;
    }
  }

  // Alignements
  bool good_align_goal = std::fabs(target_angle) < target_theta_tol;
  bool good_align_ball = std::fabs(ball_azimuth) < ball_theta_tol;


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

  if (final_state != nullptr) {
    *final_state = current_state;
  }

  Eigen::VectorXd action = Eigen::VectorXd::Zero(4);
  action.segment(1,3) = delta_cmd;

  return action;
}

Eigen::VectorXd ExpertApproach::getRawAction(const Eigen::VectorXd &state,
                                             std::default_random_engine * external_engine) const
{
  (void) external_engine;
  return getRawAction(state, (State *)nullptr);
}

void ExpertApproach::to_xml(std::ostream & out) const
{
  rosban_utils::xml_tools::write<std::string>("type", to_string(type), out);
  rosban_utils::xml_tools::write<bool>  ("lateral_kick" , lateral_kick , out);
  rosban_utils::xml_tools::write<double>("foot_y_offset", foot_y_offset, out);
  Eigen::VectorXd config = getConfig();
  std::vector<double> params;
  for (int i = 0; i < config.size(); i++)
  {
    params.push_back(config(i));
  }
  rosban_utils::xml_tools::write_vector<double>("params", params, out);
}

void ExpertApproach::from_xml(TiXmlNode * node)
{
  std::string type_str;
  rosban_utils::xml_tools::try_read<std::string>(node, "type", type_str);
  if (type_str != "") {
    type = loadType(type_str);
  }
  rosban_utils::xml_tools::try_read<bool>  (node, "lateral_kick" , lateral_kick );
  rosban_utils::xml_tools::try_read<double>(node, "foot_y_offset", foot_y_offset);
  // Reading vector list of parameters
  std::vector<double> params_read;
  rosban_utils::xml_tools::try_read_vector<double>(node, "params", params_read);
  // If number of coefficients is appropriate, update config
  if (params_read.size() == nb_parameters)
  {
    Eigen::VectorXd new_params = Eigen::Map<Eigen::VectorXd>(params_read.data(), nb_parameters);
    setConfig(type, new_params);
  }
  // Else throw an explicit error if number of parameters was not 0
  else if (params_read.size() != 0) {
    std::ostringstream oss;
    oss << "ExpertApproach::from_xml: invalid number of parameters in node 'params': "
        << "read: " << params_read.size() << ", expecting: " << nb_parameters;
    throw std::runtime_error(oss.str());
  }
}

// Since ExpertApproach specifies a command, default is substracting current command
Eigen::MatrixXd getDefaultCoeffs() {
  Eigen::MatrixXd coeffs = Eigen::MatrixXd::Zero(4,6);
  // Since 'action' is acceleration and not speed, then current speed is
  // removed from wished speed
  for (int dim = 1; dim < 4; dim++) {
    coeffs(dim,dim+2) = -1;
  }
  return coeffs;
}

std::unique_ptr<FATree> ExpertApproach::extractFATree() const {
  if (type != Type::polar) {
    throw std::logic_error("ExpertApproach::extractFATree: not implemented for non-polar");
  }
  // Far node parameters
  Eigen::VectorXd far_bias = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd far_coeffs = getDefaultCoeffs();
  far_bias(1) = step_max;//simplification
  far_coeffs(3,1) = far_theta_p;
  // Rotate node parameters
  Eigen::VectorXd rotate_bias = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd rotate_coeffs = getDefaultCoeffs();
  rotate_coeffs(1,0) = step_p;
  rotate_bias(1) = - step_p * radius;
  rotate_coeffs(2,2) = rotate_lateral_p;
  rotate_coeffs(3,1) = rotate_theta_p;
  // Near node parameters
  Eigen::VectorXd near_bias = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd near_coeffs = getDefaultCoeffs();
  near_coeffs(1,0) = step_p;
  near_bias(1) = -step_p * wished_x;
  near_coeffs(2,1) = near_lateral_p;//Large approximation
  near_coeffs(3,2) = near_theta_p;
  // Build approximators
  std::unique_ptr<FunctionApproximator> far_node(new LinearApproximator(far_bias, far_coeffs));
  std::unique_ptr<FunctionApproximator> rotate_node(new LinearApproximator(rotate_bias,
                                                                           rotate_coeffs));
  std::unique_ptr<FunctionApproximator> near_node(new LinearApproximator(near_bias, near_coeffs));
  // Above this distance, we consider state as far
  double far_split_radius = (min_far_radius + max_rotate_radius) / 2;
  std::unique_ptr<Split> far_split(new OrthogonalSplit(1, far_split_radius));
  // Other splits for distance 
  std::unique_ptr<Split> ball_align_split_low   (new OrthogonalSplit(1, -ball_theta_tol  ));
  std::unique_ptr<Split> ball_align_split_high  (new OrthogonalSplit(1,  ball_theta_tol  ));
  std::unique_ptr<Split> target_align_split_low (new OrthogonalSplit(2, -target_theta_tol));
  std::unique_ptr<Split> target_align_split_high(new OrthogonalSplit(2,  target_theta_tol));
  // Building complete tree
  std::vector<std::unique_ptr<FunctionApproximator>> approximators;
  std::unique_ptr<FATree> tmp;
  // 1. Near is the result of 4 splits to ensure the 4 boundaries
  // 1.1 ball_dir > -ball_theta_tol
  approximators.push_back(rotate_node->clone());
  approximators.push_back(std::move(near_node));
  tmp.reset(new FATree(std::move(ball_align_split_low), approximators));
  // 1.2 ball_dir < ball_theta_tol
  approximators.push_back(std::move(tmp));
  approximators.push_back(rotate_node->clone());
  tmp.reset(new FATree(std::move(ball_align_split_high), approximators));
  // 1.3 target_dir > - target_theta_tol
  approximators.push_back(rotate_node->clone());
  approximators.push_back(std::move(tmp));
  tmp.reset(new FATree(std::move(target_align_split_low), approximators));
  // 1.4 target_dir < target_theta_tol
  approximators.push_back(std::move(tmp));
  approximators.push_back(rotate_node->clone());
  tmp.reset(new FATree(std::move(target_align_split_high), approximators));
  // 2. Distinction between far and the rest
  approximators.push_back(std::move(tmp));
  approximators.push_back(std::move(far_node));
  tmp.reset(new FATree(std::move(far_split), approximators));
  // Return final result
  return std::move(tmp);
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
