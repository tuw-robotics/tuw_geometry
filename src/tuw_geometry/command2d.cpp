#include "tuw_geometry/command2d.hpp"
using namespace tuw;

Command2D::Command2D()
: cv::Vec<double, 2>(0, 0) {}

Command2D::Command2D(double v, double w)
: cv::Vec<double, 2>(v, w) {}

Command2D::Command2D(const Command2D & o)
: cv::Vec<double, 2>(o) {}

double & Command2D::v() {return this->val[0];}

const double & Command2D::v() const {return this->val[0];}

double & Command2D::w() {return this->val[1];}

const double & Command2D::w() const {return this->val[1];}

void Command2D::set(double v, double w) {this->val[0] = v, this->val[1] = w;}

std::ostream & operator<<(std::ostream & os, const tuw::Command2D & o)
{
  os << "[" << o.v() << ", " << o.w() << "]";
  return os;
}
