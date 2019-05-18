#pragma once

#include "starkit_utils/serialization/json_serializable.h"

#include <random>
#include <Eigen/Dense>

namespace csa_mdp
{
/// OdometryNoiseModel
///
/// Implement several odometry models for gaussian displacement noise.
class OdometryNoiseModel : public starkit_utils::JsonSerializable
{
public:
  /// Different noise model types.
  enum Type
  {
    NoiseDisable = 1,
    NoiseConstant = 2,
    NoiseProportional = 3,
    NoiseLinearSimple = 4,
    NoiseLinearFull = 5,
  };

  /// Initialization with noise model type.  Parameters default values and
  /// bounds configuration.
  OdometryNoiseModel(Type type);

  /// Change type and reset all parameters to default values
  void setType(Type t);

  /// Return current model type
  Type getType() const;

  /// Return current or default parameters for current model type.
  const Eigen::VectorXd& getParameters() const;

  /// Assign given parameters.  If given parameters does not comply with min or
  /// max bounds, a positive distance (for fitness scoring) from given parameters
  /// is given.  The internal parameters are not updated.  Else, the parameters
  /// are assigned and zero is returned.
  double setParameters(const Eigen::VectorXd& params);

  std::vector<std::string> getParametersNames() const;

  /// Return parameter normalization coefficients
  const Eigen::VectorXd& getNormalization() const;

  /// Generate a gaussian noise over [dX,dY,dTheta] given displacement from
  /// given random engine and using current model parameters.
  Eigen::Vector3d noiseGeneration(const Eigen::Vector3d& diff, std::default_random_engine& engine) const;

  /// Print current parameters on standard output
  void printParameters() const;

  virtual std::string getClassName() const override;
  virtual Json::Value toJson() const override;
  virtual void fromJson(const Json::Value& v, const std::string& dir_name);

private:
  /// Current model type
  Type _type;

  /// Displacement model parameters
  Eigen::VectorXd _params;

  /// Maximum parameter bounds
  Eigen::VectorXd _maxBounds;
};

}  // namespace csa_mdp
