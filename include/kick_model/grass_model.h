#pragma once

#include "kick_model/kick_zone.h"

#include "rhoban_utils/serialization/json_serializable.h"

#include <Eigen/Core>

#include <random>

namespace csa_mdp
{

class GrassModel : public rhoban_utils::JsonSerializable {
public:
  GrassModel();

  /// A kick reduction ratio for the given kick dir in field frame [deg]
  double kickReduction(double kick_dir) const;

  /// Setting the cone offset [deg]
  void setConeOffset(double cone_offset);

  void toJson(std::ostream & out) const override;
  void fromJson(TiXmlNode * node) override;
  std::string getClassName() const override;

protected:
  /// Influence of grass [0 to 1 factor]
  double ratio;

  /// Cone size [deg]
  double coneWidth;

  /// Cone offset [deg]
  double coneOffset;
};

}
