#include "kick_model/kick_zone.h"

using namespace rosban_utils;
using namespace xml_tools;

namespace csa_mdp
{

/**
 * Return the given angle in radian 
 * bounded between -PI and PI
 */
static double normalizeAngle(double angle)
{
  return angle - 2.0*M_PI*std::floor((angle + M_PI)/(2.0*M_PI));
}

KickZone::KickZone()
  : kick_x_min(0.12),
    kick_x_max(0.22),
    kick_y_tol(0.04),
    kick_y_offset(0.08),
    kick_theta_offset(0),
    kick_theta_tol(10 * M_PI/180)
{
}

bool KickZone::isKickable(const Eigen::Vector3d & state) const
{
  return canKickLeftFoot(state) || canKickRightFoot(state);
}

bool KickZone::canKickLeftFoot(const Eigen::Vector3d & state) const
{
  // Getting explicit names
  double ball_x = state(0);
  double ball_y = state(1);
  double theta  = state(2);
  double kick_err = normalizeAngle(theta + kick_theta_offset);
  // Check validity
  bool x_ok = ball_x > kick_x_min && ball_x < kick_x_max;
  bool y_ok = std::fabs(ball_y - kick_y_offset) < kick_y_tol;
  bool theta_ok = std::fabs(kick_err) < kick_theta_tol;
  bool kick_ok = x_ok && y_ok && theta_ok;
  return kick_ok;
}

bool KickZone::canKickRightFoot(const Eigen::Vector3d & state) const
{
  // Getting explicit names
  double ball_x = state(0);
  double ball_y = state(1);
  double theta  = state(2);
  double kick_err = normalizeAngle(theta - kick_theta_offset);
  // Check validity
  bool x_ok = ball_x > kick_x_min && ball_x < kick_x_max;
  bool y_ok = std::fabs(ball_y + kick_y_offset) < kick_y_tol;
  bool theta_ok = std::fabs(kick_err) < kick_theta_tol;
  bool kick_ok = x_ok && y_ok && theta_ok;
  return kick_ok;
}

void KickZone::to_xml(std::ostream & out) const
{
  xml_tools::write<double>("kick_x_min"       , kick_x_min       , out);
  xml_tools::write<double>("kick_x_max"       , kick_x_max       , out);
  xml_tools::write<double>("kick_y_tol"       , kick_y_tol       , out);
  xml_tools::write<double>("kick_y_offset"    , kick_y_offset    , out);
  xml_tools::write<double>("kick_theta_tol"   , kick_theta_tol   , out);
  xml_tools::write<double>("kick_theta_offset", kick_theta_offset, out);
}

void KickZone::from_xml(TiXmlNode * node)
{
  try_read<double>(node, "kick_x_min"       , kick_x_min       );
  try_read<double>(node, "kick_x_max"       , kick_x_max       );
  try_read<double>(node, "kick_y_tol"       , kick_y_tol       );
  try_read<double>(node, "kick_y_offset"    , kick_y_offset    );
  try_read<double>(node, "kick_theta_tol"   , kick_theta_tol   );
  try_read<double>(node, "kick_theta_offset", kick_theta_offset);
}

std::string KickZone::class_name() const
{
  return "KickZone";
}


}
