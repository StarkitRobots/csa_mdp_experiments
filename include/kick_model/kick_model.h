#pragma once

#include "kick_model/kick_zone.h"
#include "kick_model/grass_model.h"
#include "starkit_utils/serialization/json_serializable.h"

#include <Eigen/Core>

#include <random>

namespace csa_mdp
{
/// Contains both:
/// - The predictive model for the kick
/// - The kickable area model
///
/// Each kick might contain/use several parameters
///
/// All dimensions are in meters
class KickModel : public starkit_utils::JsonSerializable
{
public:
  KickModel();

  const KickZone& getKickZone() const;

  /// Return the limits for the parameters (empty matrix if no parameters)
  /// @see parameters
  const Eigen::MatrixXd& getParametersLimits() const;

  /// Return the name of the parameters
  /// @see parameters
  const std::vector<std::string>& getParametersNames() const;

  /// The default value for the parameters
  const Eigen::VectorXd& getDefaultParameters() const;

  /// Return the reward associated to the kick
  double getReward() const;

  /// Version of the function which uses the default values for the parameters
  /// - ball_pos: field_basis [m]
  /// - kick_dir: The desired kick direction in the field_basis [rad]
  /// - engine: if nullptr, then no random is performed
  /// @return final position of the ball with kick_dir (field_basis [m])
  Eigen::Vector2d applyKick(const Eigen::Vector2d& ball_pos, double kick_dir,
                            std::default_random_engine* engine = nullptr) const;

  /// Get average final position for the kick
  virtual Eigen::Vector2d getKickInSelf(const Eigen::Vector2d& ball_pos, bool right_kick) const = 0;

  /// Throw an error if kick_parameters size is not adapted
  /// - ball_pos: field_basis [m]
  /// - kick_dir: field_basis [rad]
  /// - kick_parameters: specific to the inner class
  /// - engine: if nullptr, then no random is performed
  /// @return final position of the ball with kick_dir (field_basis [m])
  virtual Eigen::Vector2d applyKick(const Eigen::Vector2d& ball_pos, double kick_dir,
                                    const Eigen::VectorXd& kick_parameters,
                                    std::default_random_engine* engine = nullptr) const = 0;

  /// Setting the grass model
  virtual void setGrassModel(GrassModel grassModel);

  Json::Value toJson() const override;
  void fromJson(const Json::Value& v, const std::string& dir_name) override;

  virtual KickModel* clone() const = 0;

protected:
  /// TODO: might be moved somewhere else
  /// Reward received after each kick (- average time [s])
  double kick_reward;

  /// The area from which the ball is kickable
  KickZone kick_zone;

  /// Parameters limits
  /// Each row is a different parameter, col0 is min, col1 is max
  Eigen::MatrixXd parameters_limits;

  /// Names for the parameters
  std::vector<std::string> parameters_names;

  /// Default parameters for the kick (should be initialized by the children class)
  Eigen::VectorXd default_parameters;

  /// Model of the grass
  GrassModel grassModel;
};

}  // namespace csa_mdp
