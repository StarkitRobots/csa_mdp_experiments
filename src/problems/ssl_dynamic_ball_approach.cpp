#include "problems/ssl_dynamic_ball_approach.h"

#include "rhoban_utils/angle.h"

#include <cmath>
#include <iostream>
#include <random>

namespace csa_mdp
{

/// If norm of vec is lower than bound, return vec, otherwise, return
/// vec * bound / norm(vec)
static Eigen::Vector2d boundNorm(const Eigen::Vector2d & vec, double bound) {
  if (vec.norm() > bound) {
    return vec * bound  / vec.norm();
  }
  return vec;
}


static Eigen::Vector3d boundXYA(const Eigen::Vector3d & vec, double cart_bound, double a_bound) {
  Eigen::Vector3d bounded_vec;
  bounded_vec.segment(0,2) = boundNorm(vec.segment(0,2), cart_bound);
  bounded_vec(2) = std::max(-a_bound,std::min(a_bound, vec(2)));
  return bounded_vec;
}

static Eigen::Vector2d pointFromPolar(double dist, double angle_rad) {
  return Eigen::Vector2d(dist * cos(angle_rad), dist * sin(angle_rad));
}

/// Return the homogeneous transform T_r2_from_r1
/// p    : the position of R2 center in r1 referential
/// alpha: the rotation from R1 to R2 (R2.x = R1.x * cos(alpha) + R1.y * sin(theta))
static Eigen::Matrix<double,3,3> getR2FromR1(const Eigen::Vector2d & p,
                                             double alpha) {
  Eigen::Matrix<double,2,2> rotation;
  rotation(0,0) = cos(alpha);
  rotation(0,1) = sin(alpha);
  rotation(1,0) = -sin(alpha);
  rotation(1,1) = cos(alpha);
  Eigen::Matrix<double,3,3> transform;
  transform.block(0,0,2,2) = rotation;
  transform.block(0,2,2,1) = -rotation * p;
  transform(2,0) = 0;
  transform(2,1) = 0;
  transform(2,2) = 1;
  return transform;
}


SSLDynamicBallApproach::SSLDynamicBallApproach() :
  // State limits
  ball_max_dist(1.0),
  target_max_dist(2.0),
  max_robot_speed(0.8),// Working well: 0.8 [m/s]
  max_robot_speed_theta(1.5),// Working well: 1.5 [rad/s]
  max_ball_speed(0.4),// Theory: 8 [m/s]
  min_kick_dir_tol(rhoban_utils::deg2rad(5)),
  max_kick_dir_tol(rhoban_utils::deg2rad(20)),
  // Acceleration limits (Much higher could be possible)
  max_acc(max_robot_speed/ 2),// 2 s to reach full speed
  max_acc_theta(max_robot_speed_theta / 2),// 2 s to reach full speed 
  // Finish
  finish_x_limits(0.11,0.2),
  finish_y_tol(0.08),
  finish_diff_speed_x_limits(-0.05,0.2),// No real idea
  finish_diff_speed_y_max(0.1),// Ideally less
  finish_speed_theta_max(rhoban_utils::deg2rad(20)),
  // Kick
  kick_x_limits(0.09,0.12),
  kick_y_tol(0.02),// Kicker width ~0.08
  kick_diff_speed_x_limits(0.0,0.05),// No real idea
  kick_diff_speed_y_max(0.025),// Ideally less
  kick_speed_theta_max(rhoban_utils::deg2rad(5)),
  // Collision
  collision_radius(0.12),// measured
  collision_forward(0.09),// measured
  collision_reward(-200),
  // Misc
  out_of_space_reward(-200),
  dt(0.033),// Around 30 Hz
  mode(Mode::Finish),
  ball_init_min_dist(collision_radius + 0.05),
  ball_init_max_dist(ball_max_dist/2),
  target_init_max_dist(target_max_dist/2),
  // Noise model
  cart_stddev(0.02),
  angular_stddev(rhoban_utils::deg2rad(2))
{
  updateLimits();
}

void SSLDynamicBallApproach::updateLimits()
{
  Eigen::MatrixXd state_limits(10,2), action_limits(3,2);
  state_limits <<
    // Ball Position
    -ball_max_dist, ball_max_dist,
    -ball_max_dist, ball_max_dist,
    // Target Position
    -target_max_dist, target_max_dist,
    -target_max_dist, target_max_dist,
    // Robot speed
    -max_robot_speed, max_robot_speed,
    -max_robot_speed, max_robot_speed,
    -max_robot_speed_theta, max_robot_speed_theta,
    // Ball speed
    -max_ball_speed, max_ball_speed,
    -max_ball_speed, max_ball_speed,
    // Kick tolerance
    min_kick_dir_tol, max_kick_dir_tol;
  action_limits <<
    -max_acc, max_acc,
    -max_acc, max_acc,
    -max_acc_theta, max_acc_theta;
  setStateLimits(state_limits);
  setActionLimits({action_limits});

  // Also ensure names are valid
  setStateNames({"ball_x", "ball_y", "target_x", "target_y", "v_x", "v_y", "v_theta",
        "ball_speed_x", "ball_speed_y", "kick_dir_tol"});
  setActionsNames({{"acc_x","acc_y","acc_theta"}});
}

void SSLDynamicBallApproach::setMaxDist(double dist)
{
  ball_max_dist = dist;
  updateLimits();
}

bool SSLDynamicBallApproach::isTerminal(const Eigen::VectorXd & state) const
{
  bool is_success = (mode == Mode::Finish && isKickable(state))
    || (mode == Mode::Wide && isFinishState(state));
  return is_success || isColliding(state) || isOutOfSpace(state);
}

double  SSLDynamicBallApproach::getReward(const Eigen::VectorXd & state,
                                   const Eigen::VectorXd & action,
                                   const Eigen::VectorXd & dst) const
{
  (void)state;(void)action;
  if (mode == Mode::Finish && isKickable(dst)) {
    return 0;
  }
  if (mode == Mode::Wide && isFinishState(dst)) {
    return 0;
  }
  if (isColliding(dst) ) {
    return collision_reward;
  }
  if (isOutOfSpace(dst)) {
    return out_of_space_reward;
  }
  return -dt;//Default reward
}

Problem::Result SSLDynamicBallApproach::getSuccessor(const Eigen::VectorXd & state,
                                                     const Eigen::VectorXd & action,
                                                     std::default_random_engine * engine) const
{
  // Now, actions need to be (0 acc_x acc_y acc_theta)
  if (action.rows() != 4) {
    std::ostringstream oss;
    oss << "SSLDynamicBallApproach::getSuccessor: "
        << " invalid dimension for action, expecting 4 and received "
        << action.rows();
    throw std::runtime_error(oss.str());
  }
  // REFERENTIAL INFORMATIONS
  // Here, we use 2 different basis:
  // rt : referential of the robot at time 'now'
  // rdt: referential of the robot at time 'now+dt'
  // BOUNDING ACCELERATION AND SPEED TO PHYSICAL LIMITS
  Eigen::Vector3d robot_acc_in_rt = action.segment(1,3);
  robot_acc_in_rt = boundXYA(robot_acc_in_rt, max_acc, max_acc_theta);
  Eigen::Vector3d robot_curr_speed_in_rt = state.segment(4,3);
  Eigen::Vector3d robot_next_speed_in_rt = robot_curr_speed_in_rt + robot_acc_in_rt * dt;
  robot_next_speed_in_rt = boundXYA(robot_next_speed_in_rt, max_robot_speed, max_robot_speed_theta);
  // COMPUTING NOISE
  // Since variance is multiplied by dt, stddev is multiplied by sqrt(dt)
  double noise_multiplier = std::sqrt(dt);
  std::normal_distribution<double> cart_noise(0,cart_stddev * noise_multiplier);
  std::normal_distribution<double> angular_noise(0, angular_stddev * noise_multiplier);
  double noise_x = cart_noise(*engine);
  double noise_y = cart_noise(*engine);
  double noise_theta = angular_noise(*engine);
  // GETTING TRANSFORM FROM RT TO RDT
  Eigen::Vector3d robot_avg_speed_in_rt = (robot_curr_speed_in_rt + robot_next_speed_in_rt) /2;
  double next_theta_speed = robot_next_speed_in_rt(2);
  // Computing orientation change of robot
  double theta_diff = robot_avg_speed_in_rt(2) * dt + noise_theta;
  robot_avg_speed_in_rt(2) = 0;// Transforming avg_speed_in_rt to homogeneous coordinates
  Eigen::Vector3d next_pos_in_rt = Eigen::Vector3d(noise_x,noise_y,1) + robot_avg_speed_in_rt * dt;
  Eigen::Matrix<double,3,3> rdt_from_rt = getR2FromR1(next_pos_in_rt.segment(0,2), theta_diff);
  // ROLLING BALL
  Eigen::Vector4d ball_state;
  ball_state.segment(0,2) = state.segment(0,2);
  ball_state.segment(2,2) = state.segment(7,2);
  ball_state = rolling_ball_model.getNextState(ball_state, dt);
  // GETTING OBJECT FINAL POSITION
  Eigen::Vector3d ball_in_rt, target_in_rt, ball_speed_in_rt;
  ball_in_rt.segment(0,2) = ball_state.segment(0,2);
  ball_in_rt(2) = 1;// point
  target_in_rt.segment(0,2) = state.segment(2,2);
  target_in_rt(2) = 1;// point
  ball_speed_in_rt.segment(0,2) = ball_state.segment(2,2);
  ball_speed_in_rt(2) = 0;// vector
  robot_next_speed_in_rt(2) = 0;// vector
  // TRANSFORMING TO RDT REFERENTIAL
  Eigen::Vector3d ball_in_rdt, target_in_rdt, ball_speed_in_rdt, robot_next_speed_in_rdt;
  ball_in_rdt = rdt_from_rt * ball_in_rt;
  target_in_rdt = rdt_from_rt * target_in_rt;
  robot_next_speed_in_rdt = rdt_from_rt * robot_next_speed_in_rt;
  ball_speed_in_rdt = rdt_from_rt * ball_speed_in_rt;
  // Filling up successor
  Eigen::VectorXd successor(10);
  successor.segment(0,2) = ball_in_rdt.segment(0,2);
  successor.segment(2,2) = target_in_rdt.segment(0,2);
  successor.segment(4,2) = robot_next_speed_in_rdt.segment(0,2);
  successor(6) = next_theta_speed;
  successor.segment(7,2) = ball_speed_in_rdt.segment(0,2);
  successor(9) = state(9);// kick_dir_tol is fixed for each trial
  // Filling up result
  Problem::Result result;
  result.successor = successor;
  result.reward = getReward(state, action, successor);
  result.terminal = isTerminal(successor);
  return result;
}

Eigen::VectorXd SSLDynamicBallApproach::getStartingState(std::default_random_engine * engine) const
{
  switch(mode) {
    case Mode::Wide: return getWideStartingState(engine);
    case Mode::Finish: return getFinishStartingState(engine);
  }
  throw std::logic_error("SSLDynamicBallApproach::getStartingState: unknown mode");
}

Eigen::VectorXd SSLDynamicBallApproach::getWideStartingState(std::default_random_engine * engine) const
{
  // Creating the distribution
  std::uniform_real_distribution<double>
    ball_dist_distrib(ball_init_min_dist, ball_init_max_dist),
    target_dist_distrib(0, target_init_max_dist),
    ball_speed_distrib(0, max_ball_speed),
    angle_distrib(-M_PI, M_PI),
    kick_dir_tol_distrib(min_kick_dir_tol, max_kick_dir_tol);
  // Generating random values
  double ball_dist = ball_dist_distrib(*engine);
  double ball_theta = angle_distrib(*engine);
  double target_dist = target_dist_distrib(*engine);
  double target_theta = angle_distrib(*engine);
  double ball_speed = ball_speed_distrib(*engine);
  double ball_speed_theta = angle_distrib(*engine);
  double kick_dir_tol = kick_dir_tol_distrib(*engine);
  // Updating state
  Eigen::VectorXd state = Eigen::VectorXd::Zero(10);
  state.segment(0,2) = pointFromPolar(ball_dist, ball_theta);
  state.segment(2,2) = pointFromPolar(target_dist, target_theta);
  state.segment(7,2) = pointFromPolar(ball_speed, ball_speed_theta);
  state(9) = kick_dir_tol;
  return state;
}

Eigen::VectorXd SSLDynamicBallApproach::getFinishStartingState(std::default_random_engine * engine) const
{
  // Creating the distribution
  std::uniform_real_distribution<double>
    ball_x_distrib(finish_x_limits(0), finish_x_limits(1)),
    ball_y_distrib(-finish_y_tol, finish_y_tol),
    target_dist_distrib(0, target_init_max_dist),
    robot_speed_offset_x_distrib(finish_diff_speed_x_limits(0), finish_diff_speed_x_limits(1)),
    robot_speed_offset_y_distrib(-finish_diff_speed_y_max, finish_diff_speed_y_max),
    robot_speed_theta_distrib(-finish_speed_theta_max, finish_speed_theta_max),
    ball_speed_distrib(0, max_ball_speed),
    angle_distrib(-M_PI, M_PI),
    kick_dir_tol_distrib(min_kick_dir_tol, max_kick_dir_tol);
  // Generating random values
  double ball_x = ball_x_distrib(*engine);
  double ball_y = ball_y_distrib(*engine);
  double target_dist = target_dist_distrib(*engine);
  double ball_speed_norm = ball_speed_distrib(*engine);
  double ball_speed_theta = angle_distrib(*engine);
  double robot_speed_dx = robot_speed_offset_x_distrib(*engine);
  double robot_speed_dy = robot_speed_offset_y_distrib(*engine);
  double robot_speed_theta = robot_speed_theta_distrib(*engine);
  double kick_dir_tol = kick_dir_tol_distrib(*engine);
  // Target dir has to be valid at start:
  std::uniform_real_distribution<double> target_dir_distrib(-kick_dir_tol, kick_dir_tol);
  double target_dir = target_dir_distrib(*engine);
  //
  Eigen::Vector2d ball_speed = pointFromPolar(ball_speed_norm, ball_speed_theta);
  Eigen::Vector2d robot_speed = ball_speed + Eigen::Vector2d(robot_speed_dx, robot_speed_dy);
  // Updating state
  Eigen::VectorXd state = Eigen::VectorXd::Zero(10);
  state.segment(0,2) = Eigen::Vector2d(ball_x, ball_y);
  state.segment(2,2) = pointFromPolar(target_dist, target_dir);
  state.segment(4,2) = robot_speed;
  state(6) = robot_speed_theta;
  state.segment(7,2) = ball_speed;
  state(9) = kick_dir_tol;
  std::cout << "FinishStartingState: " << state.transpose() << std::endl;
  return state;
}

bool SSLDynamicBallApproach::isFinishState(const Eigen::VectorXd & state) const
{
  // Get speed difference
  const Eigen::Vector2d & robot_speed = state.segment(4,2);
  const Eigen::Vector2d & ball_speed = state.segment(7,2);
  Eigen::Vector2d speed_diff =  robot_speed - ball_speed;
  double target_dir_rad = atan2(state(3), state(2));// Result in [-pi,pi]
  // Computing conditions separately
  bool ball_x_ok = state(0) >= finish_x_limits(0) && state(0) <= finish_x_limits(1);
  bool ball_y_ok = std::fabs(state(1)) <= finish_y_tol;
  bool speed_x_ok = speed_diff(0) >= finish_diff_speed_x_limits(0)
    && speed_diff(0) <= finish_diff_speed_x_limits(1);
  bool speed_y_ok = std::fabs(speed_diff(1)) <= finish_diff_speed_y_max;
  bool speed_theta_ok = std::fabs(state(6)) <= finish_speed_theta_max;
  bool direction_ok = std::fabs(target_dir_rad) <= state(9);
  // All conditions need to be gathered
  return ball_x_ok && ball_y_ok && speed_x_ok && speed_y_ok && speed_theta_ok && direction_ok;
}

bool SSLDynamicBallApproach::isKickable(const Eigen::VectorXd & state) const
{
  // Get speed difference
  const Eigen::Vector2d & robot_speed = state.segment(4,2);
  const Eigen::Vector2d & ball_speed = state.segment(7,2);
  Eigen::Vector2d speed_diff =  robot_speed - ball_speed;
  double target_dir_rad = atan2(state(3), state(2));// Result in [-pi,pi]
  // Computing conditions separately
  bool ball_x_ok = state(0) >= kick_x_limits(0) && state(0) <= kick_x_limits(1);
  bool ball_y_ok = std::fabs(state(1)) <= kick_y_tol;
  bool speed_x_ok = speed_diff(0) >= kick_diff_speed_x_limits(0)
    && speed_diff(0) <= kick_diff_speed_x_limits(1);
  bool speed_y_ok = std::fabs(speed_diff(1)) <= kick_diff_speed_y_max;
  bool speed_theta_ok = std::fabs(state(6)) <= kick_speed_theta_max;
  bool direction_ok = std::fabs(target_dir_rad) <= state(9);
  // All conditions need to be gathered
  return ball_x_ok && ball_y_ok && speed_x_ok && speed_y_ok && speed_theta_ok && direction_ok;
}

bool SSLDynamicBallApproach::isColliding(const Eigen::VectorXd & state) const {
  // Robot is globally circular but it has a kicker 'inside' the circle
  return state(0) < collision_forward && state.segment(0,2).norm() <= collision_radius;
}

bool SSLDynamicBallApproach::isOutOfSpace(const Eigen::VectorXd & state) const
{
  bool ball_dist_ko = state.segment(0,2).norm() > ball_max_dist;
  bool target_dist_ko = state.segment(2,2).norm() > target_max_dist;
  bool robot_cart_speed_ko = state.segment(4,2).norm() > max_robot_speed;
  bool robot_theta_speed_ko = std::fabs(state(6)) > max_robot_speed_theta;
  bool ball_speed_ko = state.segment(7,2).norm() > max_ball_speed;
  bool kick_tol_ko = state(9) < min_kick_dir_tol || state(9) > max_kick_dir_tol;
  return ball_dist_ko || target_dist_ko || robot_cart_speed_ko || robot_theta_speed_ko
                                               || ball_speed_ko || kick_tol_ko;
}

Json::Value SSLDynamicBallApproach::toJson() const {
  throw std::logic_error("SSLDynamicBallApproach::toJson: not implemented");
}

static SSLDynamicBallApproach::Mode str2Mode(const std::string & str) {
  if (str == "Finish") return SSLDynamicBallApproach::Mode::Finish;
  if (str == "Wide") return SSLDynamicBallApproach::Mode::Wide;
  throw std::logic_error("SSLDynamicBallApproach: str2Mode: unknown mode: '" + str + "'");
}

void SSLDynamicBallApproach::fromJson(const Json::Value & v, const std::string & dir_name)
{
  (void)dir_name;
  // Read current mode
  std::string mode_str;
  rhoban_utils::tryRead(v,"mode", &mode_str);
  if (mode_str != "") mode = str2Mode(mode_str);
  // Read internal properties
  double max_robot_speed_theta_deg(rhoban_utils::rad2deg(max_robot_speed_theta));
  double max_acc_theta_deg(rhoban_utils::rad2deg(max_acc_theta));
  double finish_speed_theta_max_deg(rhoban_utils::rad2deg(finish_speed_theta_max));
  double kick_speed_theta_max_deg(rhoban_utils::rad2deg(kick_speed_theta_max));
  double angular_stddev_deg(rhoban_utils::rad2deg(angular_stddev));
  double min_kick_dir_tol_deg(rhoban_utils::rad2deg(min_kick_dir_tol));
  double max_kick_dir_tol_deg(rhoban_utils::rad2deg(max_kick_dir_tol));
  rhoban_utils::tryRead(v,"ball_max_dist"             , &ball_max_dist);
  rhoban_utils::tryRead(v,"target_max_dist"           , &target_max_dist);
  rhoban_utils::tryRead(v,"max_robot_speed"           , &max_robot_speed);
  rhoban_utils::tryRead(v,"max_robot_speed_theta"     , &max_robot_speed_theta_deg);
  rhoban_utils::tryRead(v,"max_ball_speed"            , &max_ball_speed);
  rhoban_utils::tryRead(v,"min_kick_dir_tol"          , &min_kick_dir_tol_deg);
  rhoban_utils::tryRead(v,"max_kick_dir_tol"          , &max_kick_dir_tol_deg);
  rhoban_utils::tryRead(v,"max_acc"                   , &max_acc);
  rhoban_utils::tryRead(v,"max_acc_theta"             , &max_acc_theta_deg);
  rhoban_utils::tryRead(v,"finish_x_limits"           , &finish_x_limits);
  rhoban_utils::tryRead(v,"finish_y_tol"              , &finish_y_tol);
  rhoban_utils::tryRead(v,"finish_diff_speed_x_limits", &finish_diff_speed_x_limits);
  rhoban_utils::tryRead(v,"finish_diff_speed_y_max"   , &finish_diff_speed_y_max);
  rhoban_utils::tryRead(v,"finish_speed_theta_max"    , &finish_speed_theta_max_deg);
  rhoban_utils::tryRead(v,"kick_x_limits"             , &kick_x_limits);
  rhoban_utils::tryRead(v,"kick_y_tol"                , &kick_y_tol);
  rhoban_utils::tryRead(v,"kick_diff_speed_x_limits"  , &kick_diff_speed_x_limits);
  rhoban_utils::tryRead(v,"kick_diff_speed_y_max"     , &kick_diff_speed_y_max);
  rhoban_utils::tryRead(v,"kick_speed_theta_max"      , &kick_speed_theta_max_deg);
  rhoban_utils::tryRead(v,"collision_radius"          , &collision_radius);
  rhoban_utils::tryRead(v,"collision_forward"         , &collision_forward);
  rhoban_utils::tryRead(v,"collision_reward"          , &collision_reward);
  rhoban_utils::tryRead(v,"out_of_space_reward"       , &out_of_space_reward);
  rhoban_utils::tryRead(v,"dt"                        , &dt);
  rhoban_utils::tryRead(v,"ball_init_min_dist"        , &ball_init_min_dist);
  rhoban_utils::tryRead(v,"ball_init_max_dist"        , &ball_init_max_dist);
  rhoban_utils::tryRead(v,"target_init_max_dist"      , &target_init_max_dist);
  rhoban_utils::tryRead(v,"cart_stddev"               , &cart_stddev);
  rhoban_utils::tryRead(v,"angular_stddev"            , &angular_stddev_deg);
  rolling_ball_model.tryRead(v,"ball_rolling_model", dir_name);
  // Applying values which have been read in Deg:
  max_robot_speed_theta = rhoban_utils::deg2rad(max_robot_speed_theta_deg);
  max_acc_theta = rhoban_utils::deg2rad(max_acc_theta_deg);
  finish_speed_theta_max = rhoban_utils::deg2rad(finish_speed_theta_max_deg);
  kick_speed_theta_max = rhoban_utils::deg2rad(kick_speed_theta_max_deg);
  angular_stddev = rhoban_utils::deg2rad(angular_stddev_deg);
  min_kick_dir_tol = rhoban_utils::deg2rad(min_kick_dir_tol_deg);
  max_kick_dir_tol = rhoban_utils::deg2rad(max_kick_dir_tol_deg);

  // Update limits according to the new parameters
  updateLimits();
}

std::string SSLDynamicBallApproach::getClassName() const
{
  return "SSLDynamicBallApproach";
}

}
