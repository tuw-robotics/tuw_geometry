#ifndef TUW_GEOMETRY__POLAR2D_HPP
#define TUW_GEOMETRY__POLAR2D_HPP

#include <tuw_geometry/point2d.hpp>

#include <memory>

namespace tuw
{
class Polar2D;   /// Prototype
using Polar2DPtr = std::shared_ptr<Polar2D>;
using Polar2DConstPtr = std::shared_ptr<Polar2D const>;

/**
 * class to represent a point with rho and alpha
 **/
class Polar2D : public Point2D
{
public:
  Polar2D();
  Polar2D(const Point2D & p);
  Polar2D(double alpha, double rho);
  Polar2D(double alpha, double rho, double h);

  /**
     * angle to origin
     * @return alpha
     **/
  const double & alpha() const;
  /**
     * angle to origin
     * @return alpha
     **/
  double & alpha();
  /**
     * distance to origin
     * @return rho component
     **/
  const double & rho() const;
  /**
     * distance to origin
     * @return rho component
     **/
  double & rho();
  /**
     * @return point in cartesian space
     **/
  Point2D point() const;

private:
  using Point2D::angle;
  using Point2D::radius;
  using Point2D::x;
  using Point2D::y;
};

}  // namespace tuw
#endif  //TUW_GEOMETRY__POLAR2D_HPP
