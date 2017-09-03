#include "kick_model/directed_kick.h"

#include "rosban_utils/xml_tools.h"

namespace csa_mdp
{

DirectedKick::DirectedKick()
{
  action_names.push_back("target_dir");
  action_limits = Eigen::MatrixXd(1,2);
  action_limits << -M_PI, M_PI;
}

Eigen::VectorXd DirectedKick::computeKickParameters(const Eigen::VectorXd & unused,
                                                    const Eigen::VectorXd & actions) const
{
  (void) unused; (void) actions;
  return Eigen::VectorXd();
}

double DirectedKick::computeKickDirection(const Eigen::VectorXd & unused,
                                          const Eigen::VectorXd & actions) const
{
  (void) unused;
  if (actions.rows() != 1) {
    std::ostringstream oss;
    oss << "DirectedKick::computeKickDirection: actions has invalid dimension: "
        << "(" << actions.rows() << " while expecting " << 1 << ")" << std::endl;
    throw std::logic_error(oss.str());
  }
  return actions(0);
}
void DirectedKick::to_xml(std::ostream & out) const
{
  (void)out;
}

void DirectedKick::from_xml(TiXmlNode * node)
{
  (void)node;
}

std::string DirectedKick::class_name() const
{
  return "DirectedKick";
}

}
