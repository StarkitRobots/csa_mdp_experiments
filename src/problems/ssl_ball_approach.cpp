#include "problems/ssl_ball_approach.h"

#include "rhoban_utils/angle.h"

#include <cmath>
#include <iostream>
#include <random>

namespace csa_mdp
{

/// Return the given angle in radian bounded between -PI and PI
static double normalizeAngle(double angle)
{
  return angle - 2.0*M_PI*std::floor((angle + M_PI)/(2.0*M_PI));
}

/// If norm of vec is lower than bound, return vec, otherwise, return
/// vec * bound / norm(vec)
static Eigen::Vector2d boundNorm(const Eigen::Vector2d & vec, double bound)
{
  if (vec.norm() > bound) {
    return vec * bound  / vec.norm();
  }
  return vec;
}


static Eigen::Vector3d boundXYA(const Eigen::Vector3d & vec, double cart_bound, double a_bound)
{
  Eigen::Vector3d bounded_vec;
  bounded_vec.segment(0,2) = boundNorm(vec.segment(0,2), cart_bound);
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
  transform(2,0) = 0;
  transform(2,1) = 0;
  transform(2,2) = 1;
  return transform;
}


SSLBallApproach::SSLBallApproach() :
  // State limits
  max_dist(1.0),// Roughly OK, hard to have more ideas
  max_speed(0.8),// Working well: 0.8 [m/s]
  max_speed_theta(1.5),// Working well: 1.5 [rad/s]
  // Acceleration limits (Much higher could be possible)
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
  init_min_dist(collision_radius + 0.05),
  init_max_dist(max_dist - 0.05),
  // Noise model
  cart_stddev(0.05),
  angular_stddev(rhoban_utils::deg2rad(5))
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
  if (isKickable(dst)) {
    return kick_reward;
  }
  if (isColliding(dst) ) {
    return collision_reward;
  }
  if (isOutOfSpace(dst)) {
    return out_of_space_reward;
  }
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
  double kick_dir = state(2);
  Eigen::Vector3d acc_in_rt = action.segment(1,3);
  acc_in_rt = boundXYA(acc_in_rt, max_acc, max_acc_theta);// Ensuring acc respects the bounds
  Eigen::Vector3d curr_speed_in_rt = state.segment(3,3);
  Eigen::Vector3d next_speed_in_rt = curr_speed_in_rt + acc_in_rt * dt;
  Eigen::Vector3d avg_speed_in_rt = (curr_speed_in_rt + next_speed_in_rt) /2;
  next_speed_in_rt = boundXYA(next_speed_in_rt, max_speed, max_speed_theta);
  // Computing noise:
  // Since variance is multiplied by dt, stddev is multiplied by sqrt(dt)
  double noise_multiplier = std::sqrt(dt);
  std::normal_distribution<double> cart_noise(cart_stddev * noise_multiplier);
  std::normal_distribution<double> angular_noise(angular_stddev * noise_multiplier);
  double noise_x = cart_noise(*engine);
  double noise_y = cart_noise(*engine);
  double noise_theta = cart_noise(*engine);
  // Getting new kick_dir
  double next_speed_theta = next_speed_in_rt(2);
  double avg_speed_theta = avg_speed_in_rt(2);
  double new_kick_dir = normalizeAngle(kick_dir - avg_speed_theta * dt + noise_theta);
  // Now we do not care about speed_theta and acc_theta anymore, we can
  // transform useful vectors in homogenous 2d vectors
  avg_speed_in_rt(2) = 0;
  next_speed_in_rt(2) = 0;
  // Using homogeneous transform
  Eigen::Vector2d ball_in_rt(getBallX(state), getBallY(state));
  Eigen::Matrix<double,3,3> b_from_rt = getR2FromR1(ball_in_rt, kick_dir);
  Eigen::Vector3d next_pos_in_rt = Eigen::Vector3d(noise_x,noise_y,1) + avg_speed_in_rt * dt;
  Eigen::Vector3d next_pos_in_b = b_from_rt * next_pos_in_rt;
  Eigen::Matrix<double,3,3> rdt_from_b = getR2FromR1(next_pos_in_b.segment(0,2), -new_kick_dir);
  Eigen::Vector3d ball_in_rdt = rdt_from_b * Eigen::Vector3d(0,0,1);
  Eigen::Vector3d next_speed_in_rdt = rdt_from_b * b_from_rt * next_speed_in_rt;
  // Filling up successor
  Eigen::VectorXd successor(6);
  successor(0) = ball_in_rdt.segment(0,2).norm();
  successor(1) = atan2(ball_in_rdt(1), ball_in_rdt(0));
  successor(2) = new_kick_dir;
  successor.segment(3,2) = next_speed_in_rdt.segment(0,2);
  successor(5) = next_speed_theta;
  // Filling up result
  Problem::Result result;
  result.successor = successor;
  result.reward = getReward(state, action, successor);
  result.terminal = isTerminal(successor);
  return result;
}

Eigen::VectorXd SSLBallApproach::getStartingState(std::default_random_engine * engine) const
{
  Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
  // Creating the distribution
  std::uniform_real_distribution<double> dist_distrib(init_min_dist,
                                                      init_max_dist);
  std::uniform_real_distribution<double> angle_distrib(-M_PI, M_PI);
  // Generating random values
  double dist = dist_distrib(*engine);
  double ball_theta = angle_distrib(*engine);
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
  (void)dir_name;
  // Read internal properties
  double max_speed_theta_deg(rhoban_utils::rad2deg(max_speed_theta));
  double max_acc_theta_deg(rhoban_utils::rad2deg(max_acc_theta));
  double kick_speed_theta_max_deg(rhoban_utils::rad2deg(kick_speed_theta_max));
  double angular_stddev_deg(rhoban_utils::rad2deg(angular_stddev));
  rhoban_utils::tryRead(v,"max_dist"                  , &max_dist);
  rhoban_utils::tryRead(v,"max_speed"                 , &max_speed);
  rhoban_utils::tryRead(v,"max_speed_theta"           , &max_speed_theta_deg);
  rhoban_utils::tryRead(v,"max_acc"                   , &max_acc);
  rhoban_utils::tryRead(v,"max_acc_theta"             , &max_acc_theta_deg);
  rhoban_utils::tryRead(v,"kick_dir_tol"              , &kick_dir_tol);
  rhoban_utils::tryRead(v,"kick_x_max"                , &kick_x_max);
  rhoban_utils::tryRead(v,"kick_y_tol"                , &kick_y_tol);
  rhoban_utils::tryRead(v,"kick_speed_x_max"          , &kick_speed_x_max);
  rhoban_utils::tryRead(v,"kick_speed_y_max"          , &kick_speed_y_max);
  rhoban_utils::tryRead(v,"kick_speed_theta_max"      , &kick_speed_theta_max_deg);
  rhoban_utils::tryRead(v,"kick_reward"               , &kick_reward);
  rhoban_utils::tryRead(v,"collision_radius"          , &collision_radius);
  rhoban_utils::tryRead(v,"collision_reward"          , &collision_reward);
  rhoban_utils::tryRead(v,"out_of_space_reward"       , &out_of_space_reward);
  rhoban_utils::tryRead(v,"dt"                        , &dt);
  rhoban_utils::tryRead(v,"init_min_dist"             , &init_min_dist);
  rhoban_utils::tryRead(v,"init_max_dist"             , &init_max_dist);
  rhoban_utils::tryRead(v,"cart_stddev"               , &cart_stddev);
  rhoban_utils::tryRead(v,"angular_stddev"            , &angular_stddev_deg);
  // Applying values which have been read in Deg:
  max_speed_theta = rhoban_utils::deg2rad(max_speed_theta_deg);
  max_acc_theta = rhoban_utils::deg2rad(max_acc_theta_deg);
  kick_speed_theta_max = rhoban_utils::deg2rad(kick_speed_theta_max_deg);
  angular_stddev = rhoban_utils::deg2rad(angular_stddev_deg);

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
