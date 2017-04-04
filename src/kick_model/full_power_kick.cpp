#include "kick_model/full_power_kick.h"

#include "rosban_utils/xml_tools.h"

namespace csa_mdp
{

Eigen::MatrixXd FullPowerKick::getActionsLimits() const
{
  Eigen::MatrixXd limits(1,2);
  limits << -M_PI, M_PI;
  return limits;
}
std::vector<std::string> FullPowerKick::getActionsNames() const
{
  return {"kick_direction"};
}

double FullPowerKick::getWishedDir(double ball_x, double ball_y,
                                   const Eigen::VectorXd & kick_parameters) const
{
  // Direction of kick is not dependent on ball position
  (void) ball_x;(void) ball_y;

  return kick_parameters(0);
}

void FullPowerKick::applyKick(double ball_start_x, double ball_start_y,
                              const Eigen::VectorXd & kick_parameters,
                              std::default_random_engine * engine,
                              double * final_ball_x, double * final_ball_y,
                              double * reward) const
{
  if (kick_parameters.rows() != 1) {
    std::ostringstream oss;
    oss << "FullPowerKick::applyKick: invalid size for kick_parameters: "
        << kick_parameters.rows() << " rows, expecting 1";
    throw std::logic_error(oss.str());
  }
  applyKick(ball_start_x, ball_start_y,
            kick_power, kick_parameters(0),
            engine, final_ball_x, final_ball_y, reward);
}

void FullPowerKick::to_xml(std::ostream & out) const
{
  KickModel::to_xml(out);
  rosban_utils::xml_tools::write<double>("kick_power", kick_power, out);
}

void FullPowerKick::from_xml(TiXmlNode * node)
{
  KickModel::from_xml(node);
  kick_power = rosban_utils::xml_tools::read<double>(node, "kick_power");
}

std::string FullPowerKick::class_name() const
{
  return "FullPowerKick";
}

}
