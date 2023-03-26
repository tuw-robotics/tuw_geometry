#include "tuw_geometry/sample2d.hpp"

using namespace tuw;

Sample2D::Sample2D()
: Pose2D(), weight_(0) {}
Sample2D::Sample2D(const Sample2D & s)
: Pose2D(s), weight_(s.weight_) {}
Sample2D::Sample2D(const Pose2D & p, double weight)
: Pose2D(p), weight_(weight) {}

void Sample2D::set(const Pose2D & p, double weight)
{
  Pose2D::set(p), weight_ = weight;
}
void Sample2D::set(double x, double y, double theta, double weight)
{
  Pose2D::set(x, y, theta), weight_ = weight;
}
void Sample2D::set(const Sample2D & s)
{
  set(s, s.weight_);
}
void Sample2D::set(const Sample2DPtr & s)
{
  Pose2D::set(*s);
}
/** @return idx  **/
const unsigned int & Sample2D::idx() const
{
  return idx_;
}
/** @return idx  **/
unsigned int & Sample2D::idx()
{
  return idx_;
}
/** @return weight  **/
const double & Sample2D::weight() const
{
  return weight_;
}
/** @return weight  **/
double & Sample2D::weight()
{
  return weight_;
}
