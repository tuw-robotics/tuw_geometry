#include "tuw_geometry/plane3d.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>

using namespace tuw;
Plane3D::Plane3D() {}

Plane3D::Plane3D(const Plane3D & plane)
: cv::Vec4d(plane) {}

void Plane3D::create(const cv::Vec3d & p1, const cv::Vec3d & p2, const cv::Vec3d & p3)
{
  cv::Vec3d & n = *((cv::Vec3d *)this);
  cv::Vec3d d2 = p2 - p1;
  cv::Vec3d d3 = p3 - p1;
  n = d2.cross(d3);
  n /= cv::norm(n);
  val[3] = -(val[0] * p1[0] + val[1] * p1[1] + val[2] * p1[2]);
}

void Plane3D::create(const cv::Vec3d & p, const cv::Vec3d & normal)
{
  cv::Vec3d & n = *((cv::Vec3d *)this);
  n = normal;
  n /= cv::norm(n);
  val[3] = -(n.dot(p));
}

const cv::Vec3d & Plane3D::normal() const {return *((cv::Vec3d *)this);}

bool Plane3D::intersectionLine(
  const cv::Vec3d & p1, const cv::Vec3d & p2, cv::Vec3d & intersection, float epsilon) const
{
  cv::Vec3d & n = *((cv::Vec3d *)this);
  cv::Vec3d v = p2 - p1;
  double denominator = v[0] * n[0] + v[1] * n[1] + v[2] * n[2];
  if (fabs(denominator) < epsilon) {
    return false;
  }
  double d = val[0] * p1[0] + val[1] * p1[1] + val[2] * p1[2] + val[3];
  double u = d / (-v[0] * n[0] - v[1] * n[1] - v[2] * n[2]);
  intersection = p1 + v * u;
  return true;
}
