#ifndef MEASUREMENT_OBJECT_H
#define MEASUREMENT_OBJECT_H

#include <tuw_geometry/measurement.h>
#include <eigen3/Eigen/Dense>

namespace tuw {

class MeasurementObject;  /// Prototype
using MeasurementObjectPtr = std::shared_ptr<MeasurementObject>;
using MeasurementObjectConstPtr = std::shared_ptr<MeasurementObject const>;

class MeasurementObject : public Measurement
{
public:
    /**
     * constructor
     * @param distinguish extended classes in base versions
     **/
    MeasurementObject();
    /**
     * copy constructor
     * @param o source
     **/
    MeasurementObject(const MeasurementObject &o);
    const std::vector<int> &ids() const;
    const std::vector<double> &idsConfidence() const;
    const Eigen::Matrix3d &covariance() const;
private:
    std::vector<int> ids_;
    std::vector<double> ids_confidence_;
    Eigen::Matrix3d covariance_;
};

}

#endif // MEASUREMENT_OBJECT_H
