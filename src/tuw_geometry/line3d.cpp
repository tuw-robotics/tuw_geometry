#include "tuw_geometry/line3d.hpp"

#include "iostream"

using namespace tuw;
Line3D::Line3D() {}
Line3D::Line3D(const Line3D & l)
: cv::Vec<double, 4>(l) {}
