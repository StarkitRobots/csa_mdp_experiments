#pragma once

#include "kick_model/kick_model.h"

#include <map>
#include <memory>

namespace csa_mdp
{

class KickModelCollection : public rosban_utils::Serializable
{
public:
  KickModelCollection();

  const KickModel & getKickModel(const std::string & name) const;

  void to_xml(std::ostream & out) const override;
  void from_xml(TiXmlNode * node) override;
  std::string class_name() const override;

protected:

  std::map<std::string, std::unique_ptr<KickModel>> models;

};

}
