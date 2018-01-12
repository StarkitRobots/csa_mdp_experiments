#include "policies/expert_approach.h"

#include "rosban_fa/linear_approximator.h"
#include "rosban_fa/orthogonal_split.h"

using namespace rosban_fa;

static double deg2rad(double deg) { return M_PI * deg / 180; }
static double rad2deg(double rad) { return 180 * rad / M_PI; }

namespace csa_mdp
{

const int ExpertApproach::nb_parameters = 15;

ExpertApproach::ExpertApproach()
  : type(Type::cartesian),
    memory_state(State::far),
    approach_type("classic"),
    foot_y_offset(0.08),
    step_max(0.04),
    max_rotate_radius(0.6),
    min_far_radius(0.4),
    radius(0.5),
    step_p(0.2),
    far_theta_p(0.2),
    rotate_theta_p(0.3),
    rotate_lateral_p(-0.2),
    near_theta_p(0.2),
    near_lateral_p(0.2),
    stop_y_near(0.125),
    max_y_near(0.25),
    wished_x(0.17),
    wished_y(0.0),
    target_theta_tol(30 * M_PI / 180),
    ball_theta_tol(80 * M_PI / 180)
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
  if ((approach_type == "opportunist" && std::fabs(target_angle) > lateral_threshold)
      || approach_type == "lateral") {
    // If target is on the left, use right foot, else use left foot
    // For foot_y_offset, sign is inverted because it is always specified as a positive value
    if (target_angle > 0) {
      target_angle -= right_foot_kick_dir;
      y_error -= foot_y_offset;
    }
    else {
      target_angle += right_foot_kick_dir;
      y_error += foot_y_offset;
    }
  }
  else { 
    // If target is on the left, use left foot, else use right foot
    if (target_angle > 0) {
      y_error -= foot_y_offset;
    }
    else {
      y_error += foot_y_offset;
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
      // Move forward / backward if error along y is lower than a threshold
      if (std::fabs(y_error) < stop_y_near)
      {
        wished_cmd(0) = step_p * x_error;
      }
      wished_cmd(1) = near_lateral_p * y_error;
      // Align with goals
      wished_cmd(2) = near_theta_p * target_angle;
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

Json::Value ExpertApproach::toJson() const
{
  Json::Value v;
  v["type"            ] = to_string(type) ;
  v["step_p"          ] = step_p          ;
  v["near_lateral_p"  ] = near_lateral_p  ;
  v["approach_type"   ] = approach_type   ;
  v["foot_y_offset"   ] = foot_y_offset   ;
  v["target_theta_tol"] = target_theta_tol;
  v["ball_theta_tol"  ] = ball_theta_tol  ;
  v["params"          ] = rhoban_utils::vector2Json(getConfig());
  return v;
}

void ExpertApproach::fromJson(const Json::Value & v, const std::string & dir_name)
{
  (void)dir_name;
  std::string type_str;
  rhoban_utils::tryRead(v, "type"            , &type_str        );
  rhoban_utils::tryRead(v, "step_p"          , &step_p          );
  rhoban_utils::tryRead(v, "near_theta_p"    , &near_theta_p    );
  rhoban_utils::tryRead(v, "near_lateral_p"  , &near_lateral_p  );
  rhoban_utils::tryRead(v, "foot_y_offset"   , &foot_y_offset   );
  rhoban_utils::tryRead(v, "wished_x"        , &wished_x        );
  rhoban_utils::tryRead(v, "target_theta_tol", &target_theta_tol);
  rhoban_utils::tryRead(v, "ball_theta_tol"  , &ball_theta_tol  );
  rhoban_utils::tryRead(v, "approach_type"   , &approach_type   );
  if (type_str != "") {
    type = loadType(type_str);
  }
  bool valid_type = false;
  for (const std::string & name : {"lateral", "classic", "opportunist"}) {
    if (name == approach_type) {
      valid_type = true;
      break;
    }
  }
  if (!valid_type) {
    throw std::runtime_error("ExpertApproach::fromJson: invalid approach_type: '" 
                             + approach_type + "'");
  }
  if (foot_y_offset < 0) {
    throw std::runtime_error("ExpertApproach::fromJson: foot_y_offset should always be positive");
  }
  // Read right_foot_kick_dir (from deg to rad)
  double right_foot_kick_dir_deg = rad2deg(right_foot_kick_dir);
  rhoban_utils::tryRead(v, "right_foot_kick_dir", &right_foot_kick_dir_deg);
  right_foot_kick_dir = deg2rad(right_foot_kick_dir_deg);
  // Read lateral threshold (from deg to rad)
  if (approach_type == "opportunist") {
    double lateral_threshold_deg = rad2deg(lateral_threshold);
    rhoban_utils::tryRead(v, "lateral_threshold", &lateral_threshold_deg);
    lateral_threshold = deg2rad(lateral_threshold_deg);
  }
  // Reading vector list of parameters
  Eigen::VectorXd new_params;
  rhoban_utils::tryRead<Eigen::VectorXd>(v, "params", &new_params);
  // If number of coefficients is appropriate, update config
  if (new_params.rows() == nb_parameters) {
    setConfig(type, new_params);
  }
  // Else throw an explicit error if number of parameters was not 0
  else if (new_params.rows() != 0) {
    std::ostringstream oss;
    oss << "ExpertApproach::fromJson: invalid number of parameters in node 'params': "
        << "read: " << new_params.rows() << ", expecting: " << nb_parameters;
    throw std::runtime_error(oss.str());
  }
}

// Since ExpertApproach specifies a command, default is substracting current command
Eigen::MatrixXd getDefaultCoeffs() {
  Eigen::MatrixXd coeffs = Eigen::MatrixXd::Zero(4,6);
  // Since 'action' is acceleration and not speed, then current speed is
  // removed from wished speed
  for (int dim = 1; dim < 4; dim++) {
    coeffs(dim,dim+2) = -1;// +3 for change of dim, -1 because first line is action_type
  }
  return coeffs;
}

std::unique_ptr<FATree> ExpertApproach::extractFATree() const {
  if (type != Type::polar) {
    throw std::logic_error("ExpertApproach::extractFATree: not implemented for non-polar");
  }
  if (approach_type == "lateral") {
    return extractLateralFATree();
  } else if (approach_type == "classic") {
    return extractClassicFATree();
  } else if (approach_type == "opportunist") {
    return extractOpportunistFATree();
  }
  throw std::logic_error("ExpertApproach::extractFATree: unknown type: '"
                         + approach_type + "'");
}

std::unique_ptr<FATree> ExpertApproach::extractClassicFATree() const {
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
  double near_theta_bias = atan2(foot_y_offset, wished_x);
  Eigen::VectorXd near_right_bias = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd near_left_bias = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd near_coeffs = getDefaultCoeffs();
  near_coeffs(1,0) = step_p;
  near_right_bias(1) = -step_p * wished_x;
  near_left_bias(1) = -step_p * wished_x;
  near_coeffs(2,1) = near_lateral_p;//Large approximation (sin(x) = x)
  near_right_bias(2) = near_lateral_p * near_theta_bias;
  near_left_bias(2)  = -near_lateral_p * near_theta_bias;
  near_coeffs(3,2) = near_theta_p;
  // Build approximators
  std::unique_ptr<FunctionApproximator> far_node(new LinearApproximator(far_bias, far_coeffs));
  std::unique_ptr<FunctionApproximator> rotate_node(new LinearApproximator(rotate_bias,
                                                                           rotate_coeffs));
  std::unique_ptr<FunctionApproximator> near_right_node(new LinearApproximator(near_right_bias, near_coeffs));
  std::unique_ptr<FunctionApproximator> near_left_node(new LinearApproximator(near_left_bias, near_coeffs));
  // Foot split depends on sign of ball direction
  std::unique_ptr<Split> foot_split(new OrthogonalSplit(1, 0.0));
  // Above this distance, we consider state as far
  double far_split_radius = (min_far_radius + max_rotate_radius) / 2;
  std::unique_ptr<Split> far_split(new OrthogonalSplit(0, far_split_radius));
  // Other splits for distance 
  std::unique_ptr<Split> ball_align_split_low   (new OrthogonalSplit(1, -ball_theta_tol  ));
  std::unique_ptr<Split> ball_align_split_high  (new OrthogonalSplit(1,  ball_theta_tol  ));
  std::unique_ptr<Split> target_align_split_low (new OrthogonalSplit(2, -target_theta_tol));
  std::unique_ptr<Split> target_align_split_high(new OrthogonalSplit(2,  target_theta_tol));
  // Building complete tree
  std::vector<std::unique_ptr<FunctionApproximator>> approximators;
  std::unique_ptr<FATree> tmp;
  // 0. In near state, the foot used matters
  approximators.push_back(std::move(near_right_node));// Right if below 0
  approximators.push_back(std::move(near_left_node));// Left if below 0
  tmp.reset(new FATree(std::move(foot_split),approximators));
  // 1. Near is the result of 4 splits to ensure the 4 boundaries
  // 1.1 ball_dir > -ball_theta_tol
  approximators.push_back(rotate_node->clone());
  approximators.push_back(std::move(tmp));
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

std::unique_ptr<FATree> ExpertApproach::extractLateralFATree() const {
  double theta_bias = right_foot_kick_dir;
  // Far nodes parameters
  Eigen::VectorXd far_bias = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd far_coeffs = getDefaultCoeffs();
  far_bias(1) = step_max;//simplification
  far_coeffs(3,1) = far_theta_p;
  // Rotate node parameters
  Eigen::VectorXd rotate_left_bias = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd rotate_right_bias = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd rotate_coeffs = getDefaultCoeffs();
  rotate_coeffs(1,0) = step_p;
  rotate_left_bias(1) = - step_p * radius;
  rotate_right_bias(1) = - step_p * radius;
  rotate_coeffs(2,2) = rotate_lateral_p;
  rotate_coeffs(3,1) = rotate_theta_p;
  rotate_left_bias(2)  = -rotate_lateral_p * theta_bias;
  rotate_right_bias(2) = rotate_lateral_p * theta_bias;
  // Near node parameters
  double near_theta_bias = atan2(-foot_y_offset, wished_x);
  Eigen::VectorXd near_right_bias = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd near_left_bias = Eigen::VectorXd::Zero(4);
  Eigen::MatrixXd near_coeffs = getDefaultCoeffs();
  near_coeffs(1,0)   = step_p;
  near_right_bias(1) = -step_p * wished_x;
  near_left_bias(1)  = -step_p * wished_x;
  near_coeffs(2,1)   = near_lateral_p;//Large approximation (sin(x) = x)
  near_right_bias(2) = -near_lateral_p * near_theta_bias;
  near_left_bias(2)  =  near_lateral_p * near_theta_bias;
  near_coeffs(3,2)   = near_theta_p;
  near_right_bias(3) = near_theta_p * theta_bias;
  near_left_bias(3)  = -near_theta_p * theta_bias;
  // Build approximators
  std::unique_ptr<FunctionApproximator> far_node(
    new LinearApproximator(far_bias, far_coeffs));
  std::unique_ptr<FunctionApproximator> rotate_left_node(
    new LinearApproximator(rotate_left_bias, rotate_coeffs));
  std::unique_ptr<FunctionApproximator> rotate_right_node(
    new LinearApproximator(rotate_right_bias, rotate_coeffs));
  std::unique_ptr<FunctionApproximator> near_right_node(
    new LinearApproximator(near_right_bias, near_coeffs));
  std::unique_ptr<FunctionApproximator> near_left_node(
    new LinearApproximator(near_left_bias, near_coeffs));
  // Foot split depends on sign of target angle
  std::unique_ptr<Split> foot_split(new OrthogonalSplit(2, 0.0));
  // Above this distance, we consider state as far
  double far_split_radius = (min_far_radius + max_rotate_radius) / 2;
  std::unique_ptr<Split> far_split(new OrthogonalSplit(0, far_split_radius));
  // Other splits for distance 
  std::unique_ptr<Split> ball_align_split_low (new OrthogonalSplit(1, -ball_theta_tol  ));
  std::unique_ptr<Split> ball_align_split_high(new OrthogonalSplit(1,  ball_theta_tol  ));
  std::unique_ptr<Split> target_align_right_split_low(
    new OrthogonalSplit(2, -theta_bias - target_theta_tol));
  std::unique_ptr<Split> target_align_right_split_high(
    new OrthogonalSplit(2, -theta_bias + target_theta_tol));
  std::unique_ptr<Split> target_align_left_split_low(
    new OrthogonalSplit(2, theta_bias - target_theta_tol));
  std::unique_ptr<Split> target_align_left_split_high(
    new OrthogonalSplit(2, theta_bias + target_theta_tol));
  // Building complete tree
  std::vector<std::unique_ptr<FunctionApproximator>> approximators;
  std::unique_ptr<FATree> left_side, right_side;
  // 1. Left side: Distinction between near_left and rotate_left
  // 1.1 ball_dir > -ball_theta_tol
  approximators.push_back(rotate_left_node->clone());
  approximators.push_back(std::move(near_left_node));
  left_side.reset(new FATree(ball_align_split_low->clone(), approximators));
  // 1.2 ball_dir < ball_theta_tol
  approximators.push_back(std::move(left_side));
  approximators.push_back(rotate_left_node->clone());
  left_side.reset(new FATree(ball_align_split_high->clone(), approximators));
  // 1.3 target_dir > theta_bias - target_theta_tol
  approximators.push_back(rotate_left_node->clone());
  approximators.push_back(std::move(left_side));
  left_side.reset(new FATree(std::move(target_align_left_split_low), approximators));
  // 1.4 target_dir < theta_bias + target_theta_tol
  approximators.push_back(std::move(left_side));
  approximators.push_back(rotate_left_node->clone());
  left_side.reset(new FATree(std::move(target_align_left_split_high), approximators));
  // 2. Right side: Distinction between near_right and rotate_right
  // 2.1 ball_dir > -ball_theta_tol
  approximators.push_back(rotate_right_node->clone());
  approximators.push_back(std::move(near_right_node));
  right_side.reset(new FATree(ball_align_split_low->clone(), approximators));
  // 2.2 ball_dir < ball_theta_tol
  approximators.push_back(std::move(right_side));
  approximators.push_back(rotate_right_node->clone());
  right_side.reset(new FATree(ball_align_split_high->clone(), approximators));
  // 2.3 target_dir > -theta_bias - target_theta_tol
  approximators.push_back(rotate_right_node->clone());
  approximators.push_back(std::move(right_side));
  right_side.reset(new FATree(std::move(target_align_right_split_low), approximators));
  // 2.4 target_dir < -theta_bias + target_theta_tol
  approximators.push_back(std::move(right_side));
  approximators.push_back(rotate_right_node->clone());
  right_side.reset(new FATree(std::move(target_align_right_split_high), approximators));
  // 3. Merge right and left to get the rotate and near
  std::unique_ptr<FATree> rotate_and_near;
  approximators.push_back(std::move(right_side));
  approximators.push_back(std::move(left_side));
  rotate_and_near.reset(new FATree(std::move(foot_split), approximators));
  // 4. Distinction between far and the rest
  std::unique_ptr<FATree> final_tree;
  approximators.push_back(std::move(rotate_and_near));
  approximators.push_back(std::move(far_node));
  final_tree.reset(new FATree(std::move(far_split), approximators));
  // Return final result
  return std::move(final_tree);
}

std::unique_ptr<FATree> ExpertApproach::extractOpportunistFATree() const {
  // Only two splits are required, other approaximators are based on classic
  // and lateral extraction
  std::unique_ptr<Split> lateral_split_low (new OrthogonalSplit(2, -lateral_threshold));
  std::unique_ptr<Split> lateral_split_high(new OrthogonalSplit(2,  lateral_threshold));
  // Build the final tree
  std::vector<std::unique_ptr<FunctionApproximator>> approximators;
  approximators.push_back(extractLateralFATree());
  approximators.push_back(extractClassicFATree());
  std::unique_ptr<FATree> tmp(new FATree(std::move(lateral_split_low), approximators));
  approximators.push_back(std::move(tmp));
  approximators.push_back(extractLateralFATree());
  std::unique_ptr<FATree> final_tree(new FATree(std::move(lateral_split_high), approximators));
  // Return final result
  return std::move(final_tree);
}

ExpertApproach::Type ExpertApproach::loadType(const std::string & type_str)
{
  if (type_str == "cartesian") return Type::cartesian;
  if (type_str == "polar") return Type::polar;
  throw std::runtime_error("Unknown ExpertApproach::Type: '" + type_str + "'");
}

std::string ExpertApproach::getClassName() const
{ return "ExpertApproach"; }

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
