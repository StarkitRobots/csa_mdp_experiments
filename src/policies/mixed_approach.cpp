#include "policies/mixed_approach.h"

#include "rosban_csa_mdp/core/policy_factory.h"

#include "rosban_random/tools.h"

MixedApproach::MixedApproach()
  : far_dist_min(1.0),
    far_theta_p(-0.2),
    far_max_speed(0.04)
{
  engine = rosban_random::getRandomEngine();
}

Eigen::VectorXd MixedApproach::getRawAction(const Eigen::VectorXd & state)
{
  return getRawAction(state, &engine);
}
Eigen::VectorXd MixedApproach::getRawAction(const Eigen::VectorXd & state,
                             std::default_random_engine * engine) const
{
  /// First: check that a policy has been loaded
  if (!nearby_policy) {
    throw std::logic_error("MixedApproach::getRawAction: No nearby policy loaded yet");
  }
  // Import variables with explicit names
  double ball_dist = state(0);
  double ball_dir = state(1);
  Eigen::Vector3d current_speed = state.segment(3,3);
  // If ball is far, use the simple strategy
  if (ball_dist > far_dist_min)
  {
    Eigen::VectorXd wished_speed(3);
    wished_speed(0) = std::max(0.0, cos(ball_dir));
    wished_speed(1) = 0;
    wished_speed(2) = - ball_dir * far_theta_p;
    // Action is acceleration
    return wished_speed - current_speed;
  }
  // When ball is close, use the nearby policy
  return nearby_policy->getRawAction(state, engine);
};

void MixedApproach::to_xml(std::ostream & out) const
{
  (void)out;
  throw std::logic_error("MixedApproach::to_xml: unimplemented method");
}

void MixedApproach::from_xml(TiXmlNode * node)
{
  std::string path;
  csa_mdp::PolicyFactory().tryRead(node, "nearby_policy", nearby_policy);
}

std::string MixedApproach::class_name() const
{
  return "mixed_approach";
}
