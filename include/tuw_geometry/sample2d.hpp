#ifndef TUW_GEOMETRY__SAMPLE2D_HPP
#define TUW_GEOMETRY__SAMPLE2D_HPP

#include <tuw_geometry/pose2d.hpp>

namespace tuw
{
/**
 * Class to describe a particle used in the particle filter localization
 */
class Sample2D;
typedef std::shared_ptr<Sample2D> Sample2DPtr;
typedef std::shared_ptr<Sample2D const> Sample2DConstPtr;
class Sample2D : public Pose2D
{
  double weight_;      /// weight
  unsigned int idx_;   /// index (for debugging)

public:
  Sample2D();
  /**  copy constructor
   * @param s
   **/
  Sample2D(const Sample2D & s);
  /**  constructor
   * @param p
   * @param weight
   **/
  Sample2D(const Pose2D & p, double weight = 0);
  /**  set sample
   * @param p
   * @param weight
   **/
  void set(const Pose2D & p, double weight = 0);
  /**  set sample
   * @param s
   **/
  void set(const Sample2D & s);
  /**  set sample
   * @param s
   **/
  void set(const Sample2DPtr & s);
  /**  set sample
   * @param x
   * @param y
   * @param theta
   * @param weight
   **/
  void set(double x, double y, double theta, double weight = 0);
  /** @return idx  **/
  const unsigned int & idx() const;
  /** @return idx  **/
  unsigned int & idx();
  /** @return weight  **/
  const double & weight() const;
  /** @return weight  **/
  double & weight();
  /** Stream extraction
   * @param os outputstream
   * @param o object
   * @return stream
   **/
  friend std::ostream & operator<<(std::ostream & os, const Sample2D & o)
  {
    os << "[" << (Pose2D &) o << ", " << o.weight() << "]";
    return os;
  }

  static bool greater(const Sample2DPtr & a, const Sample2DPtr & b)
  {
    return a->weight() > b->weight();
  }

  static bool smaller(const Sample2DPtr & a, const Sample2DPtr & b)
  {
    return a->weight() < b->weight();
  }
};

}
#endif // TUW_GEOMETRY__SAMPLE2D_HPP
