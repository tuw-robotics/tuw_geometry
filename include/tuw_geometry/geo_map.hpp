#ifndef TUW_GEO_MAP__GEO_MAP_HPP_
#define TUW_GEO_MAP__GEO_MAP_HPP_

#include <opencv2/core/core.hpp>
#include <tuw_geometry/pose2d.hpp>

namespace tuw
{
/**
 * class to hold geographic meta data for a map
 * it allows to access pixels based on geo information
 * In order to use it the GeographicLib must be installed (check the CMakeFile.txt)
 **/
class GeoMapMetaData
{
public:
  GeoMapMetaData();
  double resolution;  /// The map resolution [m/cell]
  cv::Size size;      /// Map size width and height [cells]

  /**
   * The origin of the map [m, m, rad].
   * This is the real-world pose of the cell (0,0) in the map.
  **/
  Pose2D origin;
  /**
   * inti all meta data and transformation matrix
   * @param latitude
   * @param longitude
   * @param altitude
  **/
  void init(double latitude, double longitude, double altitude);
  /**
   * inti all meta data and transformation matrix
   * @param utm_easting
   * @param utm_northing
   * @param altitude
   * @param utm_zone
   * @param utm_northp
  **/
  void init(
    double utm_easting, double utm_northing, double altitude, int utm_zone, bool utm_northp);

  /**
   * @return map information as string to print
  **/
  std::string info_map() const;
  /**
   * @return geo information as string to print
  **/
  std::string info_geo() const;
  /**
   *  @return map size width in [m]
  **/
  double dx() const;
  /**
   *  @return map size height in [m]
  **/
  double dy() const;
  /**
   * latitude longitude altitude -> utm
   * the utm map depents on the map init
   * @param src latitude longitude altitude
   * @param des utm x, y, z
   * @return utm x, y, z
  **/
  cv::Vec3d & lla2utm(const cv::Vec3d & src, cv::Vec3d & des) const;
  /**
   * latitude longitude altitude -> utm
   * the utm map depents on the map init
   * @param src latitude longitude altitude
   * @return utm x, y, z
  **/
  cv::Vec3d lla2utm(const cv::Vec3d & src) const;
  /**
   * utm -> world map [m]
   * the utm map depents on the map init
   * @param src utm x, y, z
   * @param des x, y, z
   * @return x, y, z
  **/
  cv::Vec3d & utm2world(const cv::Vec3d & src, cv::Vec3d & des) const;
  /**
   * utm -> world map [m]
   * the utm map depents on the map init
   * @param src utm x, y, z
   * @return x, y, z
  **/
  cv::Vec3d utm2world(const cv::Vec3d & src) const;
  /**
   * world [m] -> map [pix]
   * @param src x, y, z
   * @param des x, y
   * @return x, y
  **/
  cv::Vec2d & world2map(const cv::Vec3d & src, cv::Vec2d & des) const;
  /**
   * world [m] -> map [pix]
   * @param src x, y, z
   * @return x, y
  **/
  cv::Vec2d world2map(const cv::Vec3d & src) const;
  /**
   * world [m] -> map [pix]
   * @param src x, y
   * @param des x, y
   * @return x, y
  **/
  cv::Vec2d & world2map(const cv::Vec2d & src, cv::Vec2d & des) const;
  /**
   * world [m] -> map [pix]
   * @param src x, y
   * @return x, y
  **/
  cv::Vec2d world2map(const cv::Vec2d & src) const;
  /**
   * geo [latitude longitude altitude] -> map [pix]
   * @param src latitude longitude altitude
   * @param des x, y
   * @return x, y
  **/
  cv::Point & g2m(const cv::Vec3d & src, cv::Point & des) const;
  /**
   * geo [latitude longitude altitude] -> map [pix]
   * @param src latitude longitude altitude
   * @return x, y
  **/
  cv::Point g2m(cv::Vec3d & src) const;

  /**
   * map [pix] -> world [m]
   * @param src x, y
   * @param des x, y, z
   * @return x, y, z
  **/
  cv::Vec3d & map2world(const cv::Vec2d & src, cv::Vec3d & des) const;
  /**
   * map [pix] -> world [m]
   * @param src x, y
   * @param des x, y
   * @return x, y
  **/
  cv::Vec2d & map2world(const cv::Vec2d & src, cv::Vec2d & des) const;
  /**
   * map [pix] -> world [m]
   * @param src x, y
   * @return x, y
  **/
  cv::Vec2d map2world(const cv::Vec2d & src) const;
  /**
   * world [m] -> utm [m]
   * @param src x, y, z
   * @param des x, y, z
   * @return x, y, z
  **/
  cv::Vec3d & world2utm(const cv::Vec3d & src, cv::Vec3d & des) const;
  /**
   * world [m] -> utm [m]
   * @param src x, y, z
   * @return x, y, z
  **/
  cv::Vec3d world2utm(const cv::Vec3d & src) const;
  /**
   * utm [m] -> geo [latitude longitude altitude]
   * @param src x, y, z
   * @param des latitude longitude altitude
   * @return latitude longitude altitude
  **/
  cv::Vec3d & utm2lla(const cv::Vec3d & src, cv::Vec3d & des) const;
  /**
   * utm [m] -> geo [latitude longitude altitude]
   * @param src x, y, z
   * @return latitude longitude altitude
  **/
  cv::Vec3d utm2lla(const cv::Vec3d & src) const;
  /**
   * map [pix]  -> geo [latitude longitude altitude]
   * @param src x, y
   * @param des latitude longitude altitude
   * @return latitude longitude altitude
  **/
  cv::Vec3d & m2g(const cv::Vec2d & src, cv::Vec3d & des) const;
  /**
   * map [pix]  -> geo [latitude longitude altitude]
   * @param src x, y
   * @param des latitude longitude altitude
   * @return latitude longitude altitude
  **/
  cv::Vec3d & m2g(const cv::Point & src, cv::Vec3d & des) const;
  /**
   * map [pix]  -> geo [latitude longitude altitude]
   * @param src x, y
   * @return latitude longitude altitude
  **/
  cv::Vec3d m2g(const cv::Point & src) const;

  /**
   * utm offset to map
   * @return x, y, z
  **/
  cv::Vec3d utm() {return utm_offset;}
  /**
   * utm zone
   * @return utm zone id
  **/
  int zone() {return utm_zone;}

  /**
   * Northern Hemisphere
   * @return true if map is on northern hemisphere
  **/
  bool is_north() {return utm_northp;}

  /**
   * Southern Hemisphere
   * @return true if map is on southern hemisphere
  **/
  bool is_south() {return !utm_northp;}

private:
  cv::Vec3d utm_offset;  /// utm offset to map
  int utm_zone;          /// utm zone id
  bool utm_northp;       /// true if map is on northern hemisphere
  cv::Matx33d Mw2m;      /// Matrix world to map
  cv::Matx33d Mm2w;      /// Matrix map to world
};

}  // namespace tuw
#endif  // TUW_GEO_MAP__GEO_MAP_HPP_
