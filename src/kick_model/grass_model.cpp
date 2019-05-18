#include "kick_model/kick_model.h"

namespace csa_mdp
{
GrassModel::GrassModel() : ratio(1), coneWidth(0), coneOffset(0)
{
}

double GrassModel::kickReduction(double kick_dir) const
{
  kick_dir -= coneOffset;
  if (kick_dir < -180)
    kick_dir += 360;
  if (kick_dir > 180)
    kick_dir -= 360;

  if (fabs(kick_dir) < coneWidth / 2)
  {
    return ratio;
  }
  else
  {
    return 1.0;
  }
}

void GrassModel::setConeOffset(double cone_offset)
{
  coneOffset = cone_offset;
}

Json::Value GrassModel::toJson() const
{
  Json::Value v;
  v["ratio"] = ratio;
  v["coneWidth"] = coneWidth;
  v["coneOffset"] = coneOffset;
  return v;
}

void GrassModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  starkit_utils::tryRead(v, "ratio", &ratio);
  starkit_utils::tryRead(v, "coneWidth", &coneWidth);
  starkit_utils::tryRead(v, "coneOffset", &coneOffset);
}

std::string GrassModel::getClassName() const
{
  return "GrassModel";
}

}  // namespace csa_mdp
