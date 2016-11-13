#pragma once

#include "problems/blackbox_problem.h"

#include <Eigen/Core>

#include <functional>
#include <random>
#include <vector>

namespace csa_mdp
{

/// The description of this problem is given in the article:
/// "Binary Action Search for Learning Continuous-Action Control Policies"
/// (Pazis & Lagoudakis 2009)
/// The discount rate used in the article is 0.95
/// 
/// The overall state space is the following
/// 0 - theta (angular position)
/// 1 - omega (angular speed)
/// 2 - cos(theta)
/// 3 - sin(theta)
/// 
/// Two different learning spaces are proposed
/// - Angular: [theta omega]
/// - Cartesian: [cos(theta) sin(theta) omega]
/// - Full: [theta omega cos(theta) sin(theta)
class CartPoleStabilization : public BlackBoxProblem {
public:
  /// cf above
  enum class LearningSpace
  { Angular, Cartesian, Full };

  CartPoleStabilization();

  std::vector<int> getLearningDimensions() const override;

  bool isTerminal(const Eigen::VectorXd & state) const override;

  double getReward(const Eigen::VectorXd & state,
                   const Eigen::VectorXd & action,
                   const Eigen::VectorXd & dst) const override;

  // tries to match the state with the learning space if necessary
  Eigen::VectorXd getSuccessor(const Eigen::VectorXd & state,
                               const Eigen::VectorXd & action,
                               std::default_random_engine * engine) const override;

  Eigen::VectorXd getStartingState() override;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

protected:
  // Entry is dimension 4, output is dimension 4
  Eigen::VectorXd getFullSuccessor(const Eigen::VectorXd & state,
                                   const Eigen::VectorXd & action,
                                   std::default_random_engine * engine) const;

  // Entry is dimension 2, output is dimension 2
  Eigen::VectorXd getAngularSuccessor(const Eigen::VectorXd & state,
                                      const Eigen::VectorXd & action,
                                      std::default_random_engine * engine) const;

  // Entry is dimension 3, output is dimension 3
  Eigen::VectorXd getCartesianSuccessor(const Eigen::VectorXd & state,
                                        const Eigen::VectorXd & action,
                                        std::default_random_engine * engine) const;

  /// Detect the learning space or throw an exception
  LearningSpace detectSpace(const Eigen::VectorXd & state) const;

  /// Detect the learning space and convert
  Eigen::VectorXd whateverToFull(const Eigen::VectorXd & state) const;

  /// Convert a state in angular space to a state in full space
  Eigen::VectorXd angularToFull(const Eigen::VectorXd & state) const;
  /// Convert a state in cartesian space to a state in full space
  Eigen::VectorXd cartesianToFull(const Eigen::VectorXd & state) const;

private:
  LearningSpace learning_space;
  
  //TODO transform those parameters in member variables, accessible through xml

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

  LearningSpace loadLearningSpace(const std::string & str);
};

std::string to_string(CartPoleStabilization::LearningSpace learning_space);

}
