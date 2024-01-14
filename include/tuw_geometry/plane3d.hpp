#ifndef TUW_GEOMETRY__PLANE3D_HPP
#define TUW_GEOMETRY__PLANE3D_HPP

#include <memory>
#include <opencv2/core/core.hpp>
#include <tuw_geometry/utils.hpp>

namespace tuw
{
class Plane3D;  /// Prototype
using Plane3DPtr = std::shared_ptr<Plane3D>;
using Plane3DConstPtr = std::shared_ptr<Plane3D const>;

/**
     * class to represent a 2D line as equation a*x + b*y + c = 0
     * The line has no endpoints
     **/
class Plane3D : public cv::Vec4d
{
public:
  friend std::ostream & operator<<(std::ostream & os, const Plane3D & o)
  {
    os << "[" << o.val[0] << ", " << o.val[1] << ", " << o.val[2] << ", " << o.val[3] << "]";
    return os;
  }
  /// constructor
  Plane3D();
  /**
       * copy constructor
       * @param l equation
       **/
  Plane3D(const Plane3D & plane);

  const cv::Vec3d & normal() const;

  /** computes the plane equation, based on three points
       * @param p1
       * @param p2
       * @param p3
       **/
  void create(const cv::Vec3d & p1, const cv::Vec3d & p2, const cv::Vec3d & p3);

  /** computes the plane equation, based a point on the plane and the plane normal
       * @param p point on the plane
       * @param n plane normal
       **/
  void create(const cv::Vec3d & p, const cv::Vec3d & n);

  /** Finds a line plane intersection
       * @param p1 line start
       * @param p2 line end
       * @param intersection intersection point with plane
       * @param epsilon computation tolerance
       * @return true if there is an intersection
       **/
  bool intersectionLine(
    const cv::Vec3d & p1, const cv::Vec3d & p2, cv::Vec3d & intersection,
    float epsilon = 0.00001) const;

private:
  double mult();
};
}  // namespace tuw
#endif  // TUW_GEOMETRY__PLANE3D_HPP
