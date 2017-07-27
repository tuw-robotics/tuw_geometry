#include "tuw_geometry/measurement_object.h"

using namespace tuw;

MeasurementObject::MeasurementObject() : Measurement(Type::OBJECT)
{
}

MeasurementObject::MeasurementObject(const MeasurementObject &o)
  : Measurement(o)
  , objects_(o.objects_)
  , range_min_(o.range_min_)
  , range_max_(o.range_max_)
  , range_max_id_(o.range_max_id_)
  , view_direction_(o.view_direction_)
  , fov_horizontal_(o.fov_horizontal_)
  , fov_vertical_(o.fov_vertical_)
  , type_(o.type_)
  , sensor_type_(o.sensor_type_)
{
}

void MeasurementObject::resize(size_t n)
{
  objects_.resize(n);
}

double &MeasurementObject::range_max()
{
  return range_max_;
}

const double &MeasurementObject::range_max() const
{
  return range_max_;
}

double &tuw::MeasurementObject::range_max_id()
{
  return range_max_id_;
}

const double &tuw::MeasurementObject::range_max_id() const
{
  return range_max_id_;
}

double &MeasurementObject::range_min()
{
  return range_min_;
}

const double &MeasurementObject::range_min() const
{
  return range_min_;
}

bool MeasurementObject::empty() const
{
  return objects_.empty();
}

size_t MeasurementObject::size() const
{
  return objects_.size();
}

MeasurementObject::Object &MeasurementObject::operator[](int i)
{
  return objects_[i];
}

const MeasurementObject::Object &MeasurementObject::operator[](int i) const
{
  return objects_[i];
}

void MeasurementObject::eraseObject(int i)
{
  // if(objects_.begin() + i <= objects_.end())
  objects_.erase(objects_.begin() + i);
}

Eigen::Quaterniond &tuw::MeasurementObject::view_direction()
{
  return view_direction_;
}

const Eigen::Quaterniond &tuw::MeasurementObject::view_direction() const
{
  return view_direction_;
}

double &tuw::MeasurementObject::fov_horizontal()
{
  return fov_horizontal_;
}

const double &tuw::MeasurementObject::fov_horizontal() const
{
  return fov_horizontal_;
}

double &tuw::MeasurementObject::fov_vertical()
{
  return fov_vertical_;
}

const double &tuw::MeasurementObject::fov_vertical() const
{
  return fov_vertical_;
}

std::string &tuw::MeasurementObject::type()
{
  return type_;
}

const std::string &tuw::MeasurementObject::type() const
{
  return type_;
}

std::string &tuw::MeasurementObject::sensor_type()
{
  return sensor_type_;
}

const std::string &tuw::MeasurementObject::sensor_type() const
{
  return sensor_type_;
}
