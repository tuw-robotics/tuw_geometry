#ifndef TUW_GEOMETRY__COMMAND_HPP
#define TUW_GEOMETRY__COMMAND_HPP

#include <memory>
#include <opencv2/core/core.hpp>

namespace tuw
{
class Command;   /// Prototype
using CommandPtr = std::shared_ptr<Command>;
using CommandConstPtr = std::shared_ptr<Command const>;

/**
 * class to handle motion control commands
 **/
class Command : public cv::Vec<double, 2>
{
public:
  /// constructor
  Command();
  /// constructor
  Command(double v, double w);
  /// copy constructor
  Command(const Command & o);

  /**
     * Stream extraction
     * @param os outputstream
     * @param o object
     * @return stream
     **/
  friend std::ostream & operator<<(std::ostream & os, const Command & o);

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

std::ostream & operator<<(std::ostream & os, const tuw::Command & o);
#endif  //TUW_GEOMETRY__COMMAND_HPP
