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

class CartPoleStabilization : public BlackBoxProblem {
public:
  CartPoleStabilization();

  bool isTerminal(const Eigen::VectorXd & state) const override;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) override;

  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action) override;

  Eigen::VectorXd getStartingState() override;

private:
  std::default_random_engine generator;
  std::uniform_real_distribution<double> noise_distribution;

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
  static double noise_max;// Uniform noise in [-noise_max,+noise_max] is applied at each step
};
