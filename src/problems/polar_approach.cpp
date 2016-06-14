#include "problems/polar_approach.h"

PolarApproach::PolarApproach() {}

bool PolarApproach::isTerminal(const Eigen::VectorXd & state) const
{
  return Approach::isTerminal(stateToCartesian(state));
}


double PolarApproach::getReward(const Eigen::VectorXd & state,
                                const Eigen::VectorXd & action,
                                const Eigen::VectorXd & dst)
{
  return Approach::getReward(stateToCartesian(state),
                             action,
                             stateToCartesian(dst));
}

Eigen::VectorXd PolarApproach::getSuccessor(const Eigen::VectorXd & state,
                                            const Eigen::VectorXd & action)
{
  Eigen::VectorXd cart_state = Approach::getSuccessor(stateToCartesian(state), action);
  return stateFromCartesian(cart_state);
}

Eigen::VectorXd PolarApproach::getStartingState()
{
  return stateFromCartesian(Approach::getStartingState());
}

std::string PolarApproach::class_name() const
{
  return "PolarApproach";
}

Eigen::VectorXd PolarApproach::stateToCartesian(const Eigen::VectorXd & polar_state)
{
  Eigen::VectorXd cart_state = polar_state;
  cart_state(0) = cos(polar_state(1)) * polar_state(0);
  cart_state(1) = sin(polar_state(1)) * polar_state(0);
  return cart_state;
}

Eigen::VectorXd PolarApproach::stateFromCartesian(const Eigen::VectorXd & cart_state)
{
  Eigen::VectorXd polar_state = cart_state;
  polar_state(0) = std::sqrt(cart_state.segment(0,2).squaredNorm());
  polar_state(1) = atan2(cart_state(1), cart_state(0));
  return polar_state;
}
