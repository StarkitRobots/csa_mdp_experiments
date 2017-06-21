#include "policies/opportunist_approach.h"

namespace csa_mdp
{

OpportunistApproach::OpportunistApproach(){}

Eigen::VectorXd
OpportunistApproach::getRawAction(const Eigen::VectorXd & state,
                                  std::default_random_engine * engine) const
{
  if (options.size() == 0) {
    throw std::logic_error("OpportunistApproach::getRawAction: no available options");
  }
  double best_score = std::numeric_limits<double>::lowest();
  Eigen::VectorXd best_action;
  for (size_t option_id = 0; option_id < options.size(); option_id++) {
    double cumulated_reward = 0;
    const csa_mdp::Policy & policy = *(options[option_id].first);
    const csa_mdp::PolarApproach & problem = options[option_id].second;
    for (int rollout = 0; rollout < nb_rollouts; rollout++) {
      cumulated_reward +=
        problem.sampleRolloutReward(state, policy, horizon, 1.0, engine);
    }
    double avg_reward = cumulated_reward / nb_rollouts;
    if (avg_reward > best_score) {
      best_action = policy.getAction(state, engine);
      best_score = avg_reward;
    }
  }
  return best_action;
}

void OpportunistApproach::to_xml(std::ostream & out) const
{
  (void) out;
  throw std::logic_error("OpportunistApproach::to_xml: not implemented");
}

void OpportunistApproach::from_xml(TiXmlNode * node)
{
  // TODO: read options
  rosban_utils::xml_tools::try_read<int>(node, "nb_rollouts" , nb_rollouts);
  rosban_utils::xml_tools::try_read<int>(node, "horizon" , horizon);
 
}
std::string OpportunistApproach::class_name() const
{
  return "OpportunistApproach";
}


}
