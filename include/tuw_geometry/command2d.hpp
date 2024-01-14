#ifndef TUW_GEOMETRY__COMMAND2D_HPP
#define TUW_GEOMETRY__COMMAND2D_HPP

#include <memory>
#include <opencv2/core/core.hpp>

namespace tuw
{
class Command2D;  /// Prototype
using Command2DPtr = std::shared_ptr<Command2D>;
using Command2DConstPtr = std::shared_ptr<Command2D const>;

/**
 * class to handle motion control commands
 **/
class Command2D : public cv::Vec<double, 2>
{
public:
  /// constructor
  Command2D();
  /// constructor
  Command2D(double v, double w);
  /// copy constructor
  Command2D(const Command2D & o);

  /**
     * Stream extraction
     * @param os outputstream
     * @param o object
     * @return stream
     **/
  friend std::ostream & operator<<(std::ostream & os, const Command2D & o);

  /**
     * @return linear velocity
     **/
  double & v();
  /**
     * @return linear velocity
     **/
  const double & v() const;

  /**
     * @return angular velocity
     **/
  double & w();

  /**
     * @return angular velocity
     **/
  const double & w() const;

  /**
     * @param v linear velocity
     * @param w angular velocity
     **/
  void set(double v, double w);
};
}  // namespace tuw

std::ostream & operator<<(std::ostream & os, const tuw::Command2D & o);
#endif  //TUW_GEOMETRY__COMMAND2D_HPP
