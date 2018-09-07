#ifndef MEASUREMENT_OBJECT_H
#define MEASUREMENT_OBJECT_H

#include <tuw_geometry/measurement.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace tuw
{
class MeasurementObject;  /// Prototype
using MeasurementObjectPtr = std::shared_ptr<MeasurementObject>;
using MeasurementObjectConstPtr = std::shared_ptr<MeasurementObject const>;

// object type constants
const static std::string OBJECT_TYPE_PERSON = "person";
const static std::string OBJECT_TYPE_OBSTACLE = "obstacle";

// sensor type constants
const static std::string SENSOR_TYPE_GENERIC_LASER_2D = "laser2d";
const static std::string SENSOR_TYPE_GENERIC_LASER_3D = "laser3d";
const static std::string SENSOR_TYPE_GENERIC_MONOCULAR_VISION = "mono";
const static std::string SENSOR_TYPE_GENERIC_STEREO_VISION = "stereo";
const static std::string SENSOR_TYPE_GENERIC_RGBD = "rgbd";

class MeasurementObject : public Measurement
{
public:
  struct Object
  {
    std::vector<int> ids;
    std::vector<double> ids_confidence;
    std::vector<double> shape_variables;
    Pose2D pose2d;
    // twist
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> covariance;
    boost::posix_time::ptime stamp;
  };

  /**
   * constructor
   * @param distinguish extended classes in base versions
   **/
  MeasurementObject();

  /**
   * copy constructor
   * @param o source
   **/
  MeasurementObject(const MeasurementObject& o);

  /**
   * resizes the vector holding the object measurements
   * @param size
   **/
  void resize(size_t n);

  /**
   * returns the max range measurement
   * @return possible max measurement
   **/
  double& range_max();

  /**
   * returns the max range measurement
   * @return possible max measurement
   **/
  const double& range_max() const;

  /**
   * returns the max id range measurement
   * @return possible max id measurement
   **/
  double& range_max_id();

  /**
   * returns the max id range measurement
   * @return possible max id measurement
   **/
  const double& range_max_id() const;

  /**
   * returns the min range measurement
   * @return possible min measurement
   **/
  double& range_min();

  /**
   * returns the min range measurement
   * @return possible min measurement
   **/
  const double& range_min() const;

  /**
   * @return true on empty
   **/
  bool empty() const;

  /**
   * @return number of objects vector
   **/
  size_t size() const;

  /** Array operation
  * @param i entry to return
  * @return reference to the element
  **/
  Object& operator[](int i);

  /** Array operation const version
  * @param i entry to return
  * @return reference to the element
  **/
  const Object& operator[](int i) const;

  /** Erase object at index i
   * @param i entry to erase
   **/
  void eraseObject(int i);

  Eigen::Quaterniond& view_direction();

  const Eigen::Quaterniond& view_direction() const;

  double& fov_horizontal();

  const double& fov_horizontal() const;

  double& fov_vertical();

  const double& fov_vertical() const;

  std::string& type();

  const std::string& type() const;

  std::string& sensor_type();

  const std::string& sensor_type() const;

private:
  std::vector<Object> objects_;

  double range_min_;
  double range_max_;
  double range_max_id_;

  Eigen::Quaterniond view_direction_;
  double fov_horizontal_;
  double fov_vertical_;

  std::string type_;
  std::string sensor_type_;
};
}

#endif  // MEASUREMENT_OBJECT_H
