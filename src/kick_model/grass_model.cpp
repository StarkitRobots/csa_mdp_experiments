#include "kick_model/kick_model.h"

#include "rosban_utils/xml_tools.h"

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

void GrassModel::to_xml(std::ostream & out) const
{
  rosban_utils::xml_tools::write<double>("ratio", ratio, out);
  rosban_utils::xml_tools::write<double>("coneWidth", coneWidth, out);
  rosban_utils::xml_tools::write<double>("coneOffset", coneOffset, out);
}

void GrassModel::from_xml(TiXmlNode * node)
{
  rosban_utils::xml_tools::try_read<double>(node, "ratio", ratio);
  rosban_utils::xml_tools::try_read<double>(node, "coneWidth", coneWidth);  
  rosban_utils::xml_tools::try_read<double>(node, "coneOffset", coneOffset);  
}

std::string GrassModel::class_name() const
{
    return "GrassModel";
}

}
