#include "kick_model/final_kick.h"

#include "rosban_utils/xml_tools.h"

namespace csa_mdp
{

FinalKick::FinalKick()
  : goal_x(4.5),
    max_y(1.3)
{
  action_names.push_back("target_y");
}

void FinalKick::updateActionLimits()
{
  action_limits = Eigen::MatrixXd(1,2);
  action_limits << -max_y, max_y;
}

Eigen::VectorXd FinalKick::computeKickParameters(const Eigen::VectorXd & ball_pos,
                                                 const Eigen::VectorXd & actions) const
{
  (void) ball_pos; (void) actions;
  return Eigen::VectorXd();
}

double FinalKick::computeKickDirection(const Eigen::VectorXd & ball_pos,
                                       const Eigen::VectorXd & actions) const
{
  if (actions.rows() != 1) {
    std::ostringstream oss;
    oss << "FinalKick::computeKickDirection: actions has invalid dimension: "
        << "(" << actions.rows() << " while expecting " << 1 << ")" << std::endl;
    throw std::logic_error(oss.str());
  }
  if (ball_pos.rows() < 2) {
    std::ostringstream oss;
    oss << "FinalKick::computeKickDirection: ball_pos has invalid dimension: "
        << "(" << ball_pos.rows() << " while expecting 2 or more)" << std::endl;
    throw std::logic_error(oss.str());
  }
  double target_y = actions(0);
  double dx = goal_x - ball_pos(0);
  double dy = target_y - ball_pos(1);
  return atan2(dy,dx);
}



void FinalKick::to_xml(std::ostream & out) const
{
  rosban_utils::xml_tools::write<double>("goal_x"    , goal_x    , out);
  rosban_utils::xml_tools::write<double>("max_y"     , max_y     , out);
}

void FinalKick::from_xml(TiXmlNode * node)
{
  goal_x     = rosban_utils::xml_tools::read<double>(node, "goal_x"    );
  max_y      = rosban_utils::xml_tools::read<double>(node, "max_y"     );
  updateActionLimits();
}

std::string FinalKick::class_name() const
{
  return "FinalKick";
}

}
