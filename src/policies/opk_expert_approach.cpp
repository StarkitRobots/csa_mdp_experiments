#include "policies/opk_expert_approach.h"

namespace csa_mdp
{

OPKExpertApproach::OPKExpertApproach()
  : kick_power(3.0),
    field_length(9.0)
{
}

Eigen::VectorXd OPKExpertApproach::getRawAction(const Eigen::VectorXd &state)
{
  return getRawAction(state, NULL);
}

Eigen::VectorXd OPKExpertApproach::getRawAction(const Eigen::VectorXd &state,
                                                std::default_random_engine * external_engine) const
{
  /// Avoid warning due to non-random action
  (void) external_engine;
  /// Check state
  if (state.rows() != 5) {
    std::ostringstream oss;
    oss << "OPKExpertApproach: Unexpected number of rows for state: " << state.rows()
        << " (5 expected)";
    throw std::runtime_error(oss.str());
  }
  /// Retrieve basic properties
  double ball_x = state(0);
  double ball_y = state(1);
  /// Computing wished direction
  double dx = field_length / 2 - ball_x;
  double dy = - ball_y;
  double dir = atan2(dy, dx);
  /// Preparing result
  Eigen::VectorXd action(3);
  action(0) = 0;
  action(1) = dir;
  action(2) = kick_power;
  return action;
}

std::string OPKExpertApproach::class_name() const
{
  return "opk_expert_approach";
}

void OPKExpertApproach::to_xml(std::ostream & out) const
{
  rosban_utils::xml_tools::write<double>("kick_power", kick_power, out);
  rosban_utils::xml_tools::write<double>("field_length", field_length, out);
}

void OPKExpertApproach::from_xml(TiXmlNode * node)
{
  rosban_utils::xml_tools::try_read<double>(node, "kick_power", kick_power);
  rosban_utils::xml_tools::try_read<double>(node, "field_length", field_length);
}

}
