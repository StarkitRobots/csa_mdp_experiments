#include "problems/ssl_ball_approach.h"

#include "rhoban_utils/angle.h"

#include <cmath>

namespace csa_mdp
{

/// Return the given angle in radian bounded between -PI and PI
static double normalizeAngle(double angle)
{
  return angle - 2.0*M_PI*std::floor((angle + M_PI)/(2.0*M_PI));
}

/// If norm2 of vec is lower than bound, return vec, otherwise, return
/// vec * bound / norm2(vec)
static Eigen::Vector2d boundNorm2(const Eigen::Vector2d & vec, double bound)
{
  double norm2_vec = vec.norm2();
  if (vec.norm2() > bound) {
    return vec * bound  / vec.norm2();
  }
  return vec;
}


static Eigen::Vector3d boundXYA(const Eigen::Vector3d & vec, double cart_bound, double a_bound)
{
  Eigen::Vector3d bounded_vec;
  bounded_vec.segment(0,2) = boundNorm2(vec.segment(0,2), cart_bound);
  bounded_vec(2) = std::max(-a_bound,std::min(a_bound, vec(2)));
  return bounded_vec;
}

/// Return the homogeneous transform T_r2_from_r1
/// p    : the position of R2 center in r1 referential
/// alpha: the rotation from R1 to R2 (R2.x = R1.x * cos(alpha) + R1.y * sin(theta))
static Eigen::Matrix<double,3,3> getR2FromR1(const Eigen::Vector2d & p,
                                             double alpha){
  Eigen::Matrix<double,2,2> rotation;
  rotation(0,0) = cos(alpha);
  rotation(0,1) = sin(alpha);
  rotation(1,0) = -sin(alpha);
  rotation(1,1) = cos(alpha);
  Eigen::Matrix<double,3,3> transform;
  transform.block(0,0,2,2) = rotation;
  transform.block(0,2,2,1) = -rotation * p;
  transform(2,2) = 1;
  return transform;
}


SSLBallApproach::SSLBallApproach() :
  // State limits
  max_dist(1.0),// Roughly OK, hard to have more ideas
  max_speed(0.8),// Working well: 0.8 [m/s]
  max_speed_theta(1.5),// Working well: 1.5 [rad/s]
  // Acceleration limits (Much higher could be possible
  max_acc(max_speed/ 2),// 2 s to reach full speed
  max_acc_theta(max_speed_theta / 2),// 2 s to reach full speed 
  // Kick
  kick_dir_tol(rhoban_utils::deg2rad(10)),// Roughly enough to kick from 1[m] distance
  kick_x_max(0.15),// Ideally 0.1
  kick_y_tol(0.03),// Kicker width ~0.08
  kick_speed_x_max(0.1),// No real idea
  kick_speed_y_max(0.05),// Ideally less
  kick_speed_theta_max(rhoban_utils::deg2rad(10)),
  kick_reward(0),
  // Collision
  collision_radius(0.12),// measured
  collision_reward(-200),
  // Misc
  out_of_space_reward(-200),
  dt(0.033),// Around 30 fps
  init_min_dist(0.4),
  init_max_dist(max_dist - 0.05)
{
  updateLimits();
}

void SSLBallApproach::updateLimits()
{
  Eigen::MatrixXd state_limits(6,2), action_limits(3,2);
  state_limits <<
    0, max_dist,
    -M_PI, M_PI,
    -M_PI, M_PI,
    -max_speed, max_speed,
    -max_speed, max_speed,
    -max_speed_theta, max_speed_theta;
  action_limits <<
    -max_acc, max_acc,
    -max_acc, max_acc,
    -max_acc_theta, max_acc_theta;
  setStateLimits(state_limits);
  setActionLimits({action_limits});

  // Also ensure names are valid
  setStateNames({"ball_dist", "ball_dir", "target_angle", "v_x", "v_y", "v_theta"});
  setActionsNames({{"acc_x","acc_y","acc_theta"}});
}

void SSLBallApproach::setMaxDist(double dist)
{
  max_dist = dist;
  updateLimits();
}

bool SSLBallApproach::isTerminal(const Eigen::VectorXd & state) const
{
  return isKickable(state) || isColliding(state) || isOutOfSpace(state);
}

double  SSLBallApproach::getReward(const Eigen::VectorXd & state,
                                   const Eigen::VectorXd & action,
                                   const Eigen::VectorXd & dst) const
{
  (void)state;(void)action;
  if (isKickable(dst)  ) return kick_reward;
  if (isColliding(dst) ) return collision_reward;
  if (isOutOfSpace(dst)) return out_of_space_reward;
  return -dt;//Default reward
}

Problem::Result SSLBallApproach::getSuccessor(const Eigen::VectorXd & state,
                                              const Eigen::VectorXd & action,
                                              std::default_random_engine * engine) const
{
  // Now, actions need to be (0 acc_x acc_y acc_theta)
  if (action.rows() != 4) {
    std::ostringstream oss;
    oss << "SSLBallApproach::getSuccessor: "
        << " invalid dimension for action, expecting 4 and received "
        << action.rows();
    throw std::runtime_error(oss.str());
  }
  // Here, we use 3 different basis:
  // rt : referential of the robot at time t (now)
  // rdt: referential of the robot at time dt (now+dt)
  // b  : referential of the ball (invariant to t)
  // Get the step which will be applied
  Eigen::Vector3d acc_in_rt = action.segment(1,3);
  acc_in_rt = boundXYA(acc_in_rt, max_acc, max_acc_theta);// Ensuring acc respects the bounds
  Eigen::Vector3d curr_speed_in_rt = state.segment(3,3);
  Eigen::Vector3d next_speed_in_rt = curr_speed_in_rt + acc_in_rt * dt;
  next_speed_in_rt = boundXYA(next_speed_in_rt, max_speed, max_speed_theta);
  // Using homogeneous transform
  Eigen::Vector2d ball_in_rt(getBallX(state), getBallY(state));
  Eigen::Matrix<double,3,3> b_from_rt = getR2FromR1(ball_in_rt, kick_dir);

  // TODO : update this part


  // Second part
  Eigen::Matrix<double,3,3> rdt_from_b = getR2FromR1(ball_in_rt, kick_dir);






  Eigen::Matrix<double,3,3> robot_t_to_ball = rot2d(-kick_dir);
  robot_in_robot = Eigen::Vector3d(getBallX(state), getBallY(state))
  robot_in_ball = 
  speed_in_ball
  // Apply a linear modification (from theory to 'reality')
  Eigen::VectorXd real_move = odometry.getDiffFullStep(next_cmd, engine);
  // Apply the real move
  Eigen::VectorXd next_state = state;
  // Apply rotation first
  double delta_theta = real_move(2);
  next_state(2) = normalizeAngle(state(2) - delta_theta);
  next_state(1) = normalizeAngle(state(1) - delta_theta);
  // Then, apply translation
  double ball_x = getBallX(next_state) - real_move(0);
  double ball_y = getBallY(next_state) - real_move(1);
  double new_dist = std::sqrt(ball_x * ball_x + ball_y * ball_y);
  double new_dir = atan2(ball_y, ball_x);
  next_state(0) = new_dist;
  next_state(1) = new_dir;
  // Update cmd
  next_state.segment(3,3) = next_cmd;
  Problem::Result result;
  result.successor = next_state;
  result.reward = getReward(state, action, next_state);
  result.terminal = isTerminal(next_state);
  return result;
}

Eigen::VectorXd SSLBallApproach::getStartingState(std::default_random_engine * engine) const
{
  Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
  // Creating the distribution
  std::uniform_real_distribution<double> dist_distrib(init_min_dist,
                                                      init_max_dist);
  std::uniform_real_distribution<double> viewable_distrib(-viewing_angle,
                                                          viewing_angle);
  std::uniform_real_distribution<double> angle_distrib(-M_PI, M_PI);
  // Generating random values
  double dist = dist_distrib(*engine);
  double ball_theta = viewable_distrib(*engine);
  double target_theta = angle_distrib(*engine);
  // Updating state
  state(0) = dist;
  state(1) = ball_theta;
  state(2) = target_theta;

  return state;
}

bool SSLBallApproach::isKickable(const Eigen::VectorXd & state) const
{
  double ball_x = getBallX(state);
  double ball_y = getBallY(state);
  double kick_dir = state(2);
  double speed_x = state(3);
  double speed_y = state(4);
  double speed_theta = state(5);
  return (ball_x >= 0 && ball_x <= kick_x_max)
    && (ball_y >= -kick_y_tol && ball_y <= kick_y_tol)
    && (kick_dir >= - kick_dir_tol && kick_dir <= kick_dir_tol)
    && (speed_x >= 0 &&  speed_x <= kick_speed_x_max)
    && (speed_y >= -kick_speed_y_max && speed_y <= kick_speed_y_max)
    && (speed_theta >= -kick_speed_theta_max && speed_theta <= kick_speed_theta_max);
}

bool SSLBallApproach::isColliding(const Eigen::VectorXd & state) const {
  // Robot is circular
  return state(0) < collision_radius;
}

bool SSLBallApproach::isOutOfSpace(const Eigen::VectorXd & state) const
{
  const Eigen::MatrixXd & space_limits = getStateLimits();
  for (int dim = 0; dim < state.rows(); dim++)
  {
    if (state(dim) < space_limits(dim, 0) || state(dim) > space_limits(dim, 1))
    {
      return true;
    }
  }
  return false;
}

Json::Value SSLBallApproach::toJson() const {
  throw std::logic_error("SSLBallApproach::toJson: not implemented");
}

void SSLBallApproach::fromJson(const Json::Value & v, const std::string & dir_name)
{
  std::string odometry_path;
  rhoban_utils::tryRead(v, "odometry_path", &odometry_path);

  // If coefficients have been properly read, use them
  if (odometry_path != "") {
    odometry.loadFile(dir_name + odometry_path);
  }
  // Read kicks
  auto kick_zone_builder = [](const Json::Value & v, const std::string & dir_name)
    {
      KickZone kz;
      kz.fromJson(v,dir_name);
      return kz;
    };
  rhoban_utils::tryReadVector<KickZone>(v, "kick_zones", dir_name, kick_zone_builder, &kick_zones);
  // Read internal properties
  double max_step_theta_deg(rhoban_utils::rad2deg(max_step_theta));
  double max_step_theta_diff_deg(rhoban_utils::rad2deg(max_step_theta_diff));
  rhoban_utils::tryRead(v,"max_dist"                  , &max_dist);
  rhoban_utils::tryRead(v,"max_speed"                 , &max_speed);
  rhoban_utils::tryRead(v,"max_step_theta"            , &max_step_theta_deg);
  rhoban_utils::tryRead(v,"max_step_x_diff"           , &max_step_x_diff);
  rhoban_utils::tryRead(v,"max_step_y_diff"           , &max_step_y_diff);
  rhoban_utils::tryRead(v,"max_step_theta_diff"       , &max_step_theta_diff_deg);
  rhoban_utils::tryRead(v,"kick_reward"               , &kick_reward);
  rhoban_utils::tryRead(v,"kick_terminal_speed_factor", &kick_terminal_speed_factor);
  rhoban_utils::tryRead(v,"viewing_angle"             , &viewing_angle);
  rhoban_utils::tryRead(v,"no_view_reward"            , &no_view_reward);
  rhoban_utils::tryRead(v,"collision_x_front"         , &collision_x_front);
  rhoban_utils::tryRead(v,"collision_x_back"          , &collision_x_back);
  rhoban_utils::tryRead(v,"collision_y"               , &collision_y);
  rhoban_utils::tryRead(v,"collision_reward"          , &collision_reward);
  rhoban_utils::tryRead(v,"terminal_collisions"       , &terminal_collisions);
  rhoban_utils::tryRead(v,"out_of_space_reward"       , &out_of_space_reward);
  rhoban_utils::tryRead(v,"walk_frequency"            , &walk_frequency);
  rhoban_utils::tryRead(v,"init_min_dist"             , &init_min_dist);
  rhoban_utils::tryRead(v,"init_max_dist"             , &init_max_dist);
  // Applying values which have been read in Deg:
  max_step_theta = rhoban_utils::deg2rad(max_step_theta_deg);
  max_step_theta_diff = rhoban_utils::deg2rad(max_step_theta_diff_deg);

  std::vector<std::string> kick_zone_names;
  rhoban_utils::tryReadVector(v, "kick_zone_names", &kick_zone_names);

  if (kick_zone_names.size() > 0) {
    KickModelCollection kmc;
    kmc.loadFile();
    for (const std::string & name : kick_zone_names) {
      kick_zones.push_back(kmc.getKickModel(name).getKickZone());
    }
  }

  // Update limits according to the new parameters
  updateLimits();
}

std::string SSLBallApproach::getClassName() const
{
  return "SSLBallApproach";
}

double SSLBallApproach::getBallX(const Eigen::VectorXd & state)
{
  return cos(state(1)) * state(0);
}

double SSLBallApproach::getBallY(const Eigen::VectorXd & state)
{
  return sin(state(1)) * state(0);
}

}
