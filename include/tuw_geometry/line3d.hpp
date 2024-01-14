#ifndef TUW_GEOMETRY__LINE3D_HPP
#define TUW_GEOMETRY__LINE3D_HPP

#include <cstdio>

#include "tuw_geometry/polar2d.hpp"

namespace tuw
{
class Line3D;  /// Prototype
using Line3DPtr = std::shared_ptr<Line3D>;
using Line3DConstPtr = std::shared_ptr<Line3D const>;

/**
 * class to represent a 3D line as equation a*x + b*y + c*z + d = 0
 * The line has no endpoints
 **/
class Line3D : public cv::Vec4d
{
public:
  friend std::ostream & operator<<(std::ostream & os, const Line3D & o)
  {
    os << "[" << o.val[0] << ", " << o.val[1] << ", " << o.val[2] << ", " << o.val[3] << "]";
    return os;
  }
  /// constructor
  Line3D();
  /**
  * copy constructor
  * @param l equation
  **/
  Line3D(const Line3D & l);
};
using Lines3D = std::vector<Line3D>;
using Lines3DPtr = std::shared_ptr<Lines3D>;
using Lines3DConstPtr = std::shared_ptr<Lines3D const>;
}  // namespace tuw
#endif  // TUW_GEOMETRY__LINE3D_HPP
