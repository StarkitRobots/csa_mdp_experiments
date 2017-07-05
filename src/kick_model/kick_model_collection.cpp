#include "kick_model/kick_model_collection.h"

#include "kick_model/kick_model_factory.h"

using namespace rosban_utils::xml_tools;

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

void KickModelCollection::to_xml(std::ostream & out) const
{
  (void) out;
  throw std::logic_error("KickModelCollection::to_xml: not implemented");
}

void KickModelCollection::from_xml(TiXmlNode * node)
{
  KickModelFactory kmf;
  models = read_map<std::unique_ptr<KickModel>>(node, "map", 
                                                [&kmf](TiXmlNode * node) {
                                                  return kmf.build(node);
                                                });
}

std::string KickModelCollection::class_name() const
{
  return "KickModelCollection";
}

}
