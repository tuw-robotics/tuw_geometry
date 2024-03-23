#include "tuw_geometry/geo_map.hpp"
#include <iostream>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <GeographicLib/UTMUPS.hpp>

using namespace tuw;
GeoMapMetaData::GeoMapMetaData()
    : resolution(1), size(0, 0), origin(), utm_offset(0, 0, 0), utm_zone(-1), Mw2m(cv::Matx33d::eye())
{
}

void GeoMapMetaData::init(double utm_easting, double utm_northing, double altitude, int utm_zone, bool utm_northp)
{
  utm_offset[0] = utm_easting;
  utm_offset[1] = utm_northing;
  utm_offset[2] = altitude;
  this->utm_zone = utm_zone;
  this->utm_northp = utm_northp;
  utm_offset[2] = altitude;
  double sx = 1. / resolution, sy = 1. / resolution; ///< scale [m/pix]

  cv::Matx<double, 3, 3> Tw = origin.tf();   // translation visualized space [m]
  cv::Matx<double, 3, 3> Sc(sx, 0, 0, 0, sy, 0, 0, 0, 1);   // scaling
  cv::Matx<double, 3, 3> Sp(1, 0, 0, 0, -1, 0, 0, 0, 1);    // fix mirror and rotation
  Mw2m = Sp * Sc * Tw;
  
  Mm2w = Mw2m.inv();
}

void GeoMapMetaData::init(double latitude, double longitude, double altitude)
{
  int zone;
  bool northp;
  double easting, northing;
  GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, easting, northing);
  init(easting, northing, altitude, zone, northp);
}

double GeoMapMetaData::dx() const
{
  return size.width * resolution;
}
double GeoMapMetaData::dy() const
{
  return size.height * resolution;
}
std::string GeoMapMetaData::info_map() const
{
  char txt[0x1FF];
  sprintf(txt, "MapInfo: [%4dpix, %4dpix] * %6.5f m/pix = [%6.2fm, %6.2fm]; origin: [%6.2fm, %6.2fm, %4.3frad]", size.width, size.height, resolution, dx(), dy(), origin.x(), origin.y(), origin.theta());
  return txt;
}

std::string GeoMapMetaData::info_geo() const
{
  char txt[0x1FF];
  cv::Vec3d origin_world_lla = utm2lla(utm_offset);
  sprintf(txt, "GeoInfo: [%12.10f°, %12.10f°, %12.10fm] --> [%12.10fm, %12.10fm, %12.10fm] zone %d %s",
          origin_world_lla[0], origin_world_lla[1], origin_world_lla[2], utm_offset[0], utm_offset[1], utm_offset[2], utm_zone,
          (utm_northp ? "north" : "south"));
  return txt;
}

cv::Vec3d &GeoMapMetaData::lla2utm(const cv::Vec3d &src, cv::Vec3d &des) const
{
  int zone;
  double gamma, k;
  bool northp;
  GeographicLib::UTMUPS::Forward(src[0], src[1], zone, northp, des[0], des[1], gamma, k, utm_zone);
  des[2] = src[2];
  return des;
}
cv::Vec3d GeoMapMetaData::lla2utm(const cv::Vec3d &src) const
{
  cv::Vec3d des;
  return lla2utm(src, des);
}

cv::Vec3d &GeoMapMetaData::utm2lla(const cv::Vec3d &src, cv::Vec3d &des) const
{
  GeographicLib::UTMUPS::Reverse(utm_zone, utm_northp, src[0], src[1], des[0], des[1]);
  des[2] = src[2];
  return des;
}
cv::Vec3d GeoMapMetaData::utm2lla(const cv::Vec3d &src) const
{
  cv::Vec3d des;
  return utm2lla(src, des);
}

cv::Vec3d &GeoMapMetaData::utm2world(const cv::Vec3d &src, cv::Vec3d &des) const
{
  des = src - utm_offset;
  return des;
}
cv::Vec3d GeoMapMetaData::utm2world(const cv::Vec3d &src) const
{
  cv::Vec3d des;
  return utm2world(src, des);
}

cv::Vec2d &GeoMapMetaData::world2map(const cv::Vec3d &src, cv::Vec2d &des) const
{
  return world2map(reinterpret_cast<const cv::Vec2d &>(src), des);
}

cv::Vec2d GeoMapMetaData::world2map(const cv::Vec3d &src) const
{
  return world2map(reinterpret_cast<const cv::Vec2d &>(src));
}

cv::Vec2d &GeoMapMetaData::world2map(const cv::Vec2d &src, cv::Vec2d &des) const
{
  cv::Vec3d tmp = Mw2m * cv::Vec3d(src[0], src[1], 1.);
  des[0] = tmp[0], des[1] = tmp[1];
  return des;
}

cv::Vec2d GeoMapMetaData::world2map(const cv::Vec2d &src) const
{
  cv::Vec2d des;
  return world2map(src, des);
}

GeoMap::GeoMap()
{
}

cv::Point &GeoMapMetaData::g2m(const cv::Vec3d &src, cv::Point &des) const
{
  cv::Vec2d pm = world2map(utm2world(lla2utm(src)));
  des.x = std::round(pm[0]), des.y = std::round(pm[1]);
  return des;
}

cv::Point GeoMapMetaData::g2m(cv::Vec3d &src) const
{
  cv::Point des;
  return g2m(src, des);
}

cv::Vec3d &GeoMapMetaData::map2world(const cv::Vec2d &src, cv::Vec3d &des) const
{
  des = Mm2w * cv::Vec3d(src[0], src[1], 1.);
  return des;
}

cv::Vec2d &GeoMapMetaData::map2world(const cv::Vec2d &src, cv::Vec2d &des) const
{
  cv::Vec3d tmp = Mm2w * cv::Vec3d(src[0], src[1], 1.);
  des[0] = tmp[0], des[1] = tmp[1];
  return des;
}
cv::Vec2d GeoMapMetaData::map2world(const cv::Vec2d &src) const
{
  cv::Vec2d des;
  return map2world(src, des);
}

cv::Vec3d &GeoMapMetaData::world2utm(const cv::Vec3d &src, cv::Vec3d &des) const
{
  des = src + utm_offset;
  return des;
}
cv::Vec3d GeoMapMetaData::world2utm(const cv::Vec3d &src) const
{
  cv::Vec3d des;
  return world2utm(src, des);
}

cv::Vec3d &GeoMapMetaData::m2g(const cv::Vec2d &src, cv::Vec3d &des) const
{
  cv::Vec3d p_w, p_utm;
  return utm2lla(world2utm(map2world(src, p_w), p_utm), des);
}

cv::Vec3d &GeoMapMetaData::m2g(const cv::Point &src, cv::Vec3d &des) const
{
  cv::Vec2d tmp(src.x, src.y);
  cv::Vec3d p_w, p_utm;
  return utm2lla(world2utm(map2world(tmp, p_w), p_utm), des);
}

cv::Vec3d GeoMapMetaData::m2g(const cv::Point &src) const
{
  cv::Vec3d des;
  return m2g(src, des);
}