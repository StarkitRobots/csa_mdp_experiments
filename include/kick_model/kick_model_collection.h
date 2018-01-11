#pragma once

#include "kick_model/kick_model.h"

#include <map>
#include <memory>

namespace csa_mdp
{

class KickModelCollection : public rhoban_utils::JsonSerializable
{
public:
  KickModelCollection();

  /// throw exception if name is not found
  const KickModel & getKickModel(const std::string & name) const;

  /// Return the name of all the kicks inside the collection
  std::vector<std::string> getKickNames() const;

  /// Setting the grass cone offset [deg]
  void setGrassConeOffset(double offset);

  Json::Value toJson() const override;
  void fromJson(const Json::Value & v, const std::string & dir_name) override;
  std::string getClassName() const override;

protected:
  GrassModel grassModel;

  std::map<std::string, std::unique_ptr<KickModel>> models;

};

}
