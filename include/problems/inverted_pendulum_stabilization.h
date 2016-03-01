#pragma once

#include "problems/blackbox_problem.h"

#include <Eigen/Core>

#include <functional>
#include <random>
#include <vector>

/**
 * The description of this problem is given in the article:
 * "Binary Action Search for Learning Continuous-Action Control Policies"
 * (Pazis & Lagoudakis 2009)
 * The discount rate used in the article is 0.95
 */

class InvertedPendulumStabilization {
public:
  static double thetaMax;// Above this values, task is considered as failed
  static double dtThetaMax;
  static double actionMax;

  static double noise;// Uniform noise in [-noise,+noise] is applied at each step

  InvertedPendulum();

  bool isTerminal(const Eigen::Vector2d& state);
  double reward(const Eigen::Vector2d& src,
                double action,
                const Eigen::Vector2d& result);

  Eigen::Vector2d getSuccessor(const Eigen::Vector2d& state, double action);




private:
  // Problem properties
  static double integration_step;//[s]
  static double simulation_step; //[s] Also called controlStep
  static double pendulum_mass;   //[kg] Mass of the pendulum
  static double cart_mass;       //[kg] Mass of the cart
  static double pendulum_length; //[m] length of the pendulum
  static double g;//[m/s^2]
  /// State space parameters
  static double theta_max;// Above this values, task is considered as failed
  static double omega_max;
  static double action_max;
  static double action_noise;// Uniform noise in [-noise,+noise] is applied at each step
};
