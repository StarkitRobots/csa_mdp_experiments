#include <fstream>
#include <iomanip>
#include <stdexcept>

#include "odometry/odometry.h"

/**
 * Return the given angle in radian
 * bounded between -PI and PI
 */
static double AngleBound(double angle)
{
  return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
}

/**
 * Compute the oriented distance between the two given angle
 * in the range -PI/2:PI/2 radian from angleSrc to angleDst
 * (Better than doing angleDst-angleSrc)
 */
static double AngleDistance(double angleSrc, double angleDst)
{
  angleSrc = AngleBound(angleSrc);
  angleDst = AngleBound(angleDst);

  double max, min;
  if (angleSrc > angleDst)
  {
    max = angleSrc;
    min = angleDst;
  }
  else
  {
    max = angleDst;
    min = angleSrc;
  }

  double dist1 = max - min;
  double dist2 = 2.0 * M_PI - max + min;

  if (dist1 < dist2)
  {
    if (angleSrc > angleDst)
    {
      return -dist1;
    }
    else
    {
      return dist1;
    }
  }
  else
  {
    if (angleSrc > angleDst)
    {
      return dist2;
    }
    else
    {
      return -dist2;
    }
  }
}

void WriteEigenVectorToStream(std::ostream& os, const Eigen::VectorXd& vect)
{
  os << vect.size();
  for (size_t i = 0; i < (size_t)vect.size(); i++)
  {
    os << " " << std::setprecision(17) << vect(i);
  }
  os << std::endl;
}

Eigen::VectorXd ReadEigenVectorFromStream(std::istream& is)
{
  size_t size;
  is >> size;
  Eigen::VectorXd vect(size);
  for (size_t i = 0; i < size; i++)
  {
    double value;
    is >> value;
    vect(i) = value;
  }
  while (is.peek() == ' ' || is.peek() == '\n')
  {
    is.ignore();
  }

  return vect;
}

namespace csa_mdp
{
Odometry::Odometry() : Odometry(OdometryDisplacementModel::Type::DisplacementIdentity)
{
}

Odometry::Odometry(OdometryDisplacementModel::Type typeDisplacement, OdometryNoiseModel::Type typeNoise)
  : _modelDisplacement(typeDisplacement)
  , _modelNoise(typeNoise)
  , _isInitialized(false)
  , _last()
  , _state()
  , _corrected()
  , _lastDiff()
{
  // Ask reset
  reset();
}

OdometryDisplacementModel::Type Odometry::getDisplacementType() const
{
  return _modelDisplacement.getType();
}
OdometryNoiseModel::Type Odometry::getNoiseType() const
{
  return _modelNoise.getType();
}

Eigen::VectorXd Odometry::getParameters() const
{
  size_t sizeDisplacement = _modelDisplacement.getParameters().size();
  size_t sizeNoise = _modelNoise.getParameters().size();

  Eigen::VectorXd params(sizeDisplacement + sizeNoise);
  if (sizeDisplacement > 0)
  {
    params.segment(0, sizeDisplacement) = _modelDisplacement.getParameters();
  }
  if (sizeNoise > 0)
  {
    params.segment(sizeDisplacement, sizeNoise) = _modelNoise.getParameters();
  }

  return params;
}

double Odometry::setParameters(const Eigen::VectorXd& params)
{
  size_t sizeDisplacement = _modelDisplacement.getParameters().size();
  size_t sizeNoise = _modelNoise.getParameters().size();

  if ((size_t)params.size() != sizeDisplacement + sizeNoise)
  {
    throw std::logic_error("Odometry invalid parameters size: " + std::to_string(sizeDisplacement) + std::string("+") +
                           std::to_string(sizeNoise) + std::string("!=") + std::to_string(params.size()));
  }

  double error = 0.0;
  if (sizeDisplacement > 0)
  {
    error += _modelDisplacement.setParameters(params.segment(0, sizeDisplacement));
  }
  if (sizeNoise > 0)
  {
    error += _modelNoise.setParameters(params.segment(sizeDisplacement, sizeNoise));
  }
  return error;
}

std::vector<std::string> Odometry::getParametersNames() const
{
  std::vector<std::string> displacementNames, noiseNames, result;
  displacementNames = _modelDisplacement.getParametersNames();
  noiseNames = _modelNoise.getParametersNames();
  // Adding displacement names (with prefix)
  for (const std::string& name : displacementNames)
  {
    result.push_back("displacement_" + name);
  }
  // Adding noise names (with prefix)
  for (const std::string& name : noiseNames)
  {
    result.push_back("noise_" + name);
  }
  return result;
}

Eigen::VectorXd Odometry::getNormalization() const
{
  size_t sizeDisplacement = _modelDisplacement.getNormalization().size();
  size_t sizeNoise = _modelNoise.getNormalization().size();

  Eigen::VectorXd coefs(sizeDisplacement + sizeNoise);
  if (sizeDisplacement > 0)
  {
    coefs.segment(0, sizeDisplacement) = _modelDisplacement.getNormalization();
  }
  if (sizeNoise > 0)
  {
    coefs.segment(sizeDisplacement, sizeNoise) = _modelNoise.getNormalization();
  }

  return coefs;
}

void Odometry::printParameters() const
{
  _modelDisplacement.printParameters();
  _modelNoise.printParameters();
}

void Odometry::reset()
{
  _isInitialized = false;
  _state = Eigen::Vector3d(0.0, 0.0, 0.0);
  _corrected = Eigen::Vector3d(0.0, 0.0, 0.0);
  _lastDiff = Eigen::Vector3d(0.0, 0.0, 0.0);
}
void Odometry::reset(const Eigen::Vector3d& pose)
{
  _isInitialized = false;
  _state = pose;
  _corrected = pose;
  _lastDiff = Eigen::Vector3d(0.0, 0.0, 0.0);
}

void Odometry::updateFullStep(const Eigen::Vector3d& deltaPose, std::default_random_engine* engine)
{
  if (!_isInitialized)
  {
    _last = _state;
    _isInitialized = true;
  }

  // Apply displacement correction
  Eigen::Vector3d diff = getDiffFullStep(deltaPose, engine);
  // Save applied delta
  _lastDiff = deltaPose;

  // Integrate current state with given full
  // step relative displacement
  _last = _state;
  odometryInt(diff, _state);
  _corrected = _state;
}

Eigen::Vector3d Odometry::getDiffFullStep(const Eigen::Vector3d& deltaPose, std::default_random_engine* engine) const
{
  // Apply displacement correction
  Eigen::Vector3d diff = _modelDisplacement.displacementCorrection(deltaPose);
  // Apply noise generation if available
  if (engine != nullptr)
  {
    diff += _modelNoise.noiseGeneration(diff, *engine);
  }
  return diff;
}

const Eigen::Vector3d& Odometry::state() const
{
  return _corrected;
}

Eigen::Vector3d Odometry::odometryDiff(const Eigen::Vector3d& state1, const Eigen::Vector3d& state2) const
{
  // Vector in world
  double vectX = state2.x() - state1.x();
  double vectY = state2.y() - state1.y();
  double angle = AngleDistance(state1.z(), state2.z());
  // Rotation to source frame
  double vectInSrcX = vectX * cos(-state1.z()) - vectY * sin(-state1.z());
  double vectInSrcY = vectX * sin(-state1.z()) + vectY * cos(-state1.z());

  return Eigen::Vector3d(vectInSrcX, vectInSrcY, angle);
}

void Odometry::odometryInt(const Eigen::Vector3d& diff, Eigen::Vector3d& state) const
{
  // Rotation to world frame
  double vectX = diff.x() * cos(state.z()) - diff.y() * sin(state.z());
  double vectY = diff.x() * sin(state.z()) + diff.y() * cos(state.z());
  // Integration
  state.x() += vectX;
  state.y() += vectY;
  state.z() += diff.z();
  // Shrink to -PI,PI
  state.z() = AngleBound(state.z());
}

std::string Odometry::getClassName() const
{
  return "OdometryModel";
}

Json::Value Odometry::toJson() const
{
  Json::Value v;
  v["displacement"] = _modelDisplacement.toJson();
  v["noise"] = _modelNoise.toJson();
  return v;
}

void Odometry::fromJson(const Json::Value& v, const std::string& dir_name)
{
  _modelDisplacement.tryRead(v, "displacement", dir_name);
  _modelNoise.tryRead(v, "noise", dir_name);
  reset();
}

}  // namespace csa_mdp
