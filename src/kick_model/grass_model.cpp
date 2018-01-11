#include "kick_model/kick_model.h"

#include "rhoban_utils/xml_tools.h"

namespace csa_mdp
{

GrassModel::GrassModel()
  : ratio(1),
    coneWidth(0),
    coneOffset(0)
{
}

double GrassModel::kickReduction(double kick_dir) const
{
    kick_dir -= coneOffset;
    if (kick_dir < -180) kick_dir += 360;
    if (kick_dir > 180) kick_dir -= 360;

    if (fabs(kick_dir) < coneWidth/2) {
        return ratio;
    } else {
        return 1.0;
    }
}

void GrassModel::setConeOffset(double cone_offset)
{
    coneOffset = cone_offset;
}

void GrassModel::toJson(std::ostream & out) const
{
  rhoban_utils::xml_tools::write<double>("ratio", ratio, out);
  rhoban_utils::xml_tools::write<double>("coneWidth", coneWidth, out);
  rhoban_utils::xml_tools::write<double>("coneOffset", coneOffset, out);
}

void GrassModel::fromJson(TiXmlNode * node)
{
  rhoban_utils::xml_tools::try_read<double>(node, "ratio", ratio);
  rhoban_utils::xml_tools::try_read<double>(node, "coneWidth", coneWidth);  
  rhoban_utils::xml_tools::try_read<double>(node, "coneOffset", coneOffset);  
}

std::string GrassModel::getClassName() const
{
    return "GrassModel";
}

}
