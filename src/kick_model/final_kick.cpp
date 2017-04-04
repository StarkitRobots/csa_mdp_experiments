#include "kick_model/final_kick.h"

#include "rosban_utils/xml_tools.h"

namespace csa_mdp
{

FinalKick::FinalKick()
  : kick_power(2.0),
    goal_x(4.5),
    max_y(1.3)
{
}

Eigen::MatrixXd FinalKick::getActionsLimits() const
{
  Eigen::MatrixXd limits(1,2);
  limits << -max_y, max_y;
  return limits;
}
std::vector<std::string> FinalKick::getActionsNames() const
{
  return {"target_y"};
}

double FinalKick::getWishedDir(double ball_x, double ball_y,
                               const Eigen::VectorXd & kick_parameters) const
{
  double target_y = kick_parameters(0);
  double dx = goal_x - ball_x;
  double dy = target_y - ball_y;
  return atan2(dy,dx);
}

void FinalKick::applyKick(double ball_start_x, double ball_start_y,
                              const Eigen::VectorXd & kick_parameters,
                              std::default_random_engine * engine,
                              double * final_ball_x, double * final_ball_y,
                              double * reward) const
{
  if (kick_parameters.rows() != 1) {
    std::ostringstream oss;
    oss << "FinalKick::applyKick: invalid size for kick_parameters: "
        << kick_parameters.rows() << " rows, expecting 1";
    throw std::logic_error(oss.str());
  }
  double kick_dir = getWishedDir(ball_start_x, ball_start_y, kick_parameters);
  applyKick(ball_start_x, ball_start_y,
            kick_power, kick_dir,
            engine, final_ball_x, final_ball_y, reward);
}

void FinalKick::to_xml(std::ostream & out) const
{
  KickModel::to_xml(out);
  rosban_utils::xml_tools::write<double>("kick_power", kick_power, out);
  rosban_utils::xml_tools::write<double>("goal_x"    , goal_x    , out);
  rosban_utils::xml_tools::write<double>("max_y"     , max_y     , out);
}

void FinalKick::from_xml(TiXmlNode * node)
{
  KickModel::from_xml(node);
  kick_power = rosban_utils::xml_tools::read<double>(node, "kick_power");
  goal_x     = rosban_utils::xml_tools::read<double>(node, "goal_x"    );
  max_y      = rosban_utils::xml_tools::read<double>(node, "max_y"     );
}

std::string FinalKick::class_name() const
{
  return "FinalKick";
}

}
