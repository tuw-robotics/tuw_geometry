#include "tuw_geometry/measurement_object.h"

using namespace tuw;

MeasurementObject::MeasurementObject()
    : Measurement(Type::OBJECT) 
{}

MeasurementObject::MeasurementObject (const MeasurementObject& o)
    : Measurement(o) 
{}

const std::vector<int> &tuw::MeasurementObject::ids() const
{
  return ids_;
}

const std::vector<double> &tuw::MeasurementObject::idsConfidence() const
{
  return ids_confidence_;
}

const Eigen::Matrix3d &tuw::MeasurementObject::covariance() const
{
  return covariance_;
}
