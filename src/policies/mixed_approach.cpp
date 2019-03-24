#include "policies/mixed_approach.h"

#include "rhoban_csa_mdp/core/policy_factory.h"

#include "rhoban_random/tools.h"

namespace csa_mdp
{
MixedApproach::MixedApproach() : far_dist_min(1.0), far_theta_p(-0.2), far_max_speed(0.04)
{
  engine = rhoban_random::getRandomEngine();
}

Eigen::VectorXd MixedApproach::getRawAction(const Eigen::VectorXd& state)
{
  return getRawAction(state, &engine);
}
Eigen::VectorXd MixedApproach::getRawAction(const Eigen::VectorXd& state, std::default_random_engine* engine) const
{
  /// First: check that a policy has been loaded
  if (!nearby_policy)
  {
    throw std::logic_error("MixedApproach::getRawAction: No nearby policy loaded yet");
  }
  // Import variables with explicit names
  double ball_dist = state(0);
  double ball_dir = state(1);
  Eigen::Vector3d current_speed = state.segment(3, 3);
  // If ball is far, use the simple strategy
  if (ball_dist > far_dist_min)
  {
    Eigen::VectorXd wished_speed(3);
    wished_speed(0) = std::max(0.0, cos(ball_dir));
    wished_speed(1) = 0;
    wished_speed(2) = -ball_dir * far_theta_p;
    // Action is (0, acceleration)
    Eigen::VectorXd action = Eigen::VectorXd::Zero(4);
    action.segment(1, 3) = wished_speed - current_speed;
    return action;
  }
  // When ball is close, use the nearby policy
  return nearby_policy->getRawAction(state, engine);
};

Json::Value MixedApproach::toJson() const
{
  throw std::logic_error("MixedApproach::toJson: unimplemented method");
}

void MixedApproach::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  std::string path;
  csa_mdp::PolicyFactory().tryRead(v, "nearby_policy", dir_name, &nearby_policy);
}

std::string MixedApproach::getClassName() const
{
  return "MixedApproach";
}

}  // namespace csa_mdp
