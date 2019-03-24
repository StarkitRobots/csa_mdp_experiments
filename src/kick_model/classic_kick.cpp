#include "kick_model/classic_kick.h"

#include <math.h>

using namespace rhoban_utils;

static double deg2rad(double deg)
{
  return M_PI * deg / 180;
}
static double rad2deg(double rad)
{
  return 180 * rad / M_PI;
}

namespace csa_mdp
{
ClassicKick::ClassicKick() : kick_power(1.0), right_kick_dir(0.0), rel_dist_stddev(0.05), dir_stddev(10 * M_PI / 180)
{
  // Setting limits, names and default value for parameters
  parameters_limits = Eigen::MatrixXd(1, 2);
  parameters_limits << -M_PI, M_PI;
  parameters_names.push_back("kick_theta_tol");
  default_parameters = Eigen::VectorXd(1);
  default_parameters(0) = 10 * M_PI / 180;
}

Eigen::Vector2d ClassicKick::applyKick(const Eigen::Vector2d& ball_pos, double kick_dir,
                                       const Eigen::VectorXd& kick_parameters, std::default_random_engine* engine) const
{
  (void)kick_parameters;
  double kick_real_dist = kick_power;
  double kick_real_dir = kick_dir;
  // If engine has been provided, apply noise
  if (engine != nullptr)
  {
    // TODO: theta_tol could be provided by kick_parameters, but kick_decision models
    //       would need to be changed
    double theta_tol = 10 * M_PI / 180;
    // Uniform noise related to kick tolerance
    std::uniform_real_distribution<double> player_dir_dist(-theta_tol, theta_tol);
    // Kick related noise
    std::normal_distribution<double> kick_dir_dist(0.0, dir_stddev);
    // Distance relative noise
    std::normal_distribution<double> kick_factor_dist(1.0, rel_dist_stddev);
    // Apply noise
    kick_real_dir += kick_dir_dist(*engine);
    kick_real_dir += player_dir_dist(*engine);
    kick_real_dist *= kick_factor_dist(*engine);
  }
  kick_real_dist *= grassModel.kickReduction(rad2deg(kick_real_dir));
  Eigen::Vector2d final_pos = ball_pos;
  final_pos(0) += kick_real_dist * cos(kick_real_dir);
  final_pos(1) += kick_real_dist * sin(kick_real_dir);
  return final_pos;
}

Eigen::Vector2d ClassicKick::getKickInSelf(const Eigen::Vector2d& ball_pos, bool right_kick) const
{
  double kick_dir = right_kick ? right_kick_dir : -right_kick_dir;
  return applyKick(ball_pos, kick_dir);
}

Json::Value ClassicKick::toJson() const
{
  Json::Value v = KickModel::toJson();
  // Using human readable values in xml
  double kick_dir_deg = rad2deg(right_kick_dir);
  double dir_stddev_deg = rad2deg(dir_stddev);
  v["kick_power"] = kick_power;
  v["right_kick_dir"] = kick_dir_deg;
  v["rel_dist_stddev"] = rel_dist_stddev;
  v["dir_stddev"] = dir_stddev_deg;
  return v;
}

void ClassicKick::fromJson(const Json::Value& v, const std::string& dir_name)
{
  KickModel::fromJson(v, dir_name);
  double kick_dir_deg, dir_stddev_deg;
  kick_power = rhoban_utils::read<double>(v, "kick_power");
  kick_dir_deg = rhoban_utils::read<double>(v, "right_kick_dir");
  rel_dist_stddev = rhoban_utils::read<double>(v, "rel_dist_stddev");
  dir_stddev_deg = rhoban_utils::read<double>(v, "dir_stddev");
  // Stored informations are [rad], but xml values are [deg]
  right_kick_dir = deg2rad(kick_dir_deg);
  dir_stddev = deg2rad(dir_stddev_deg);
}

std::string ClassicKick::getClassName() const
{
  return "ClassicKick";
}

KickModel* ClassicKick::clone() const
{
  return new ClassicKick(*this);
}

}  // namespace csa_mdp
