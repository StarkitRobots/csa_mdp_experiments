#include "kick_model/kick_model_collection.h"

#include "kick_model/kick_model_factory.h"

namespace csa_mdp
{

KickModelCollection::KickModelCollection() {}

const KickModel & KickModelCollection::getKickModel(const std::string & name) const
{
  try {
    return *(models.at(name));
  }
  catch (const std::out_of_range & exc) {
    throw std::logic_error("Cannot find '" + name + "' in KickModelCollection");
  }
}

std::vector<std::string> KickModelCollection::getKickNames() const
{
  std::vector<std::string> names;
  for (const auto & entry : models) {
    names.push_back(entry.first);
  }
  return names;
}

Json::Value KickModelCollection::toJson() const
{
  throw std::logic_error("KickModelCollection::toJson: not implemented");
}

void KickModelCollection::fromJson(const Json::Value & v, const std::string & dir_name)
{
  KickModelFactory kmf;
  models = rhoban_utils::readMap<std::unique_ptr<KickModel>>(
    v, "map", dir_name,
    [&kmf](const Json::Value & v, const std::string & dir_name) {
      return kmf.build(v, dir_name);
    });

  grassModel.read(v, "grassModel", dir_name);
  for (auto & entry : models) {
    entry.second->setGrassModel(grassModel);
  }
}

void KickModelCollection::setGrassConeOffset(double offset)
{
  grassModel.setConeOffset(offset);
  for (auto & entry : models) {
    entry.second->setGrassModel(grassModel);
  }
}

std::string KickModelCollection::getClassName() const
{
  return "KickModelCollection";
}

}
