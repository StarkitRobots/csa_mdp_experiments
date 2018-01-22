#include "problems/ssl_dynamic_ball_approach.h"

#include "rhoban_utils/angle.h"

#include <cmath>
#include <iostream>
#include <random>

namespace csa_mdp
{

/// Return the given angle in radian bounded between -PI and PI
static double normalizeAngle(double angle) {
  return angle - 2.0*M_PI*std::floor((angle + M_PI)/(2.0*M_PI));
}

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
  ball_max_dist(1.0),// Roughly OK, hard to have more ideas
  max_robot_speed(0.8),// Working well: 0.8 [m/s]
  max_robot_speed_theta(1.5),// Working well: 1.5 [rad/s]
  min_kick_dir_tol(rhoban_utils::deg2rad(5)),
  max_kick_dir_tol(rhoban_utils::deg2rad(20)),
  // Acceleration limits (Much higher could be possible)
  max_acc(max_robot_speed/ 2),// 2 s to reach full speed
  max_acc_theta(max_robot_speed_theta / 2),// 2 s to reach full speed 
  // Kick
  kick_x_max(0.15),// Ideally 0.1
  kick_y_tol(0.03),// Kicker width ~0.08
  kick_diff_speed_x_max(0.1),// No real idea
  kick_diff_speed_y_max(0.05),// Ideally less
  kick_speed_theta_max(rhoban_utils::deg2rad(10)),
  // Collision
  collision_radius(0.12),// measured
  collision_reward(-200),
  // Misc
  out_of_space_reward(-200),
  dt(0.033),// Around 30 Hz
  ball_init_min_dist(collision_radius + 0.05),
  ball_init_max_dist(ball_max_dist/2),
  target_init_max_dist(target_max_dist/2),
  // Noise model
  cart_stddev(0.05),
  angular_stddev(rhoban_utils::deg2rad(5))
{
  updateLimits();
}

void SSLDynamicBallApproach::updateLimits()
{
  Eigen::MatrixXd state_limits(6,2), action_limits(3,2);
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
  return isKickable(state) || isColliding(state) || isOutOfSpace(state);
}

double  SSLDynamicBallApproach::getReward(const Eigen::VectorXd & state,
                                   const Eigen::VectorXd & action,
                                   const Eigen::VectorXd & dst) const
{
  (void)state;(void)action;
  if (isKickable(dst)) {
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
  // Creating the distribution
  std::uniform_real_distribution<double>
    ball_dist_distrib(ball_init_min_dist, ball_init_max_dist),
    target_dist_distrib(0, target_init_max_dist),
    ball_speed_distrib(ball_init_min_dist, ball_init_max_dist),
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

bool SSLDynamicBallApproach::isKickable(const Eigen::VectorXd & state) const
{
  // TODO: write it

  double ball_x = getBallX(state);
  double ball_y = getBallY(state);
  double kick_dir = state(2);
  double speed_x = state(3);
  double speed_y = state(4);
  double speed_theta = state(5);
  return (ball_x >= 0 && ball_x <= kick_x_max)
    && (ball_y >= -kick_y_tol && ball_y <= kick_y_tol)
    && (kick_dir >= - kick_dir_tol && kick_dir <= kick_dir_tol)
    && (speed_x >= 0 &&  speed_x <= kick_diff_speed_x_max)
    && (speed_y >= -kick_diff_speed_y_max && speed_y <= kick_diff_speed_y_max)
    && (speed_theta >= -kick_speed_theta_max && speed_theta <= kick_speed_theta_max);
}

bool SSLDynamicBallApproach::isColliding(const Eigen::VectorXd & state) const {
  // Robot is circular
  return state(0) < collision_radius;
}

bool SSLDynamicBallApproach::isOutOfSpace(const Eigen::VectorXd & state) const
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

Json::Value SSLDynamicBallApproach::toJson() const {
  throw std::logic_error("SSLDynamicBallApproach::toJson: not implemented");
}

void SSLDynamicBallApproach::fromJson(const Json::Value & v, const std::string & dir_name)
{
  (void)dir_name;
  // Read internal properties
  double max_robot_speed_theta_deg(rhoban_utils::rad2deg(max_robot_speed_theta));
  double max_acc_theta_deg(rhoban_utils::rad2deg(max_acc_theta));
  double kick_speed_theta_max_deg(rhoban_utils::rad2deg(kick_speed_theta_max));
  double angular_stddev_deg(rhoban_utils::rad2deg(angular_stddev));
  double min_kick_dir_tol_deg(rhoban_utils::rad2deg(min_kick_dir_tol));
  double max_kick_dir_tol_deg(rhoban_utils::rad2deg(max_kick_dir_tol));
  rhoban_utils::tryRead(v,"max_dist"                  , &max_dist);
  rhoban_utils::tryRead(v,"max_robot_speed"           , &max_robot_speed);
  rhoban_utils::tryRead(v,"max_robot_speed_theta"     , &max_robot_speed_theta_deg);
  rhoban_utils::tryRead(v,"max_ball_speed"            , &max_ball_speed);
  rhoban_utils::tryRead(v,"max_acc"                   , &max_acc);
  rhoban_utils::tryRead(v,"max_acc_theta"             , &max_acc_theta_deg);
  rhoban_utils::tryRead(v,"min_kick_dir_tol"          , &min_kick_dir_tol_deg);
  rhoban_utils::tryRead(v,"max_kick_dir_tol"          , &max_kick_dir_tol_deg);
  rhoban_utils::tryRead(v,"kick_x_max"                , &kick_x_max);
  rhoban_utils::tryRead(v,"kick_y_tol"                , &kick_y_tol);
  rhoban_utils::tryRead(v,"kick_diff_speed_x_max"     , &kick_diff_speed_x_max);
  rhoban_utils::tryRead(v,"kick_diff_speed_y_max"     , &kick_diff_speed_y_max);
  rhoban_utils::tryRead(v,"kick_speed_theta_max"      , &kick_speed_theta_max_deg);
  rhoban_utils::tryRead(v,"collision_radius"          , &collision_radius);
  rhoban_utils::tryRead(v,"collision_reward"          , &collision_reward);
  rhoban_utils::tryRead(v,"out_of_space_reward"       , &out_of_space_reward);
  rhoban_utils::tryRead(v,"dt"                        , &dt);
  rhoban_utils::tryRead(v,"ball_init_min_dist"        , &ball_init_min_dist);
  rhoban_utils::tryRead(v,"ball_init_max_dist"        , &ball_init_max_dist);
  rhoban_utils::tryRead(v,"target_init_max_dist"      , &target_init_max_dist);
  rhoban_utils::tryRead(v,"cart_stddev"               , &cart_stddev);
  rhoban_utils::tryRead(v,"angular_stddev"            , &angular_stddev_deg);
  // Applying values which have been read in Deg:
  max_robot_speed_theta = rhoban_utils::deg2rad(max_robot_speed_theta_deg);
  max_acc_theta = rhoban_utils::deg2rad(max_acc_theta_deg);
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
