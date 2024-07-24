#ifndef TUW_GEOMETRY__GEO_HANDLER_HPP
#define TUW_GEOMETRY__GEO_HANDLER_HPP

#include <GeographicLib/UTMUPS.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <tuw_geometry/map_handler.hpp>
#include <tuw_geometry/pose2d.hpp>

namespace tuw
{

/**
   * class to hold world file data
   * A world file is a six line plain text sidecar file used by geographic information systems (GIS)
   * to georeference raster map images. The file specification was introduced by Esri
   * This version has an extension to also read and store an UTM z value and a map offset.
   **/
class WorldFile
{
public:
  WorldFile()
  : resolution_x(.0),
    rotation_y(.0),
    rotation_x(.0),
    coordinate_x(.0),
    coordinate_y(.0),
    coordinate_z(.0),
    origin_x(.0),
    origin_y(.0)
  {
  }

  double resolution_x;  /// pixel size in the x-direction in map units/pixel
  double rotation_y;    /// rotation about y-axis
  double rotation_x;    /// rotation about x-axis
  double resolution_y;  /// pixel size in the y-direction in map units, almost always negative
  double coordinate_x;  /// x-coordinate of the center of the upper left pixel
  double coordinate_y;  /// y-coordinate of the center of the upper left pixel
  double coordinate_z;  /// z-coordinate of the center of the upper left pixel (if exits)
  double origin_x;      /// origin of the map default upper left pixel (if exits)
  double origin_y;      /// origin of the map default upper left pixel (if exits)

  /**
     * writes a world file with six lines
     * @param filename
     * @return true on error otherwise false
     **/
  bool write_jgw(const std::string & filename)
  {
    std::ofstream datei(filename);
    if (datei.is_open()) {
      // write six lines
      datei << std::fixed << std::setprecision(8) << resolution_x << std::endl;
      datei << std::fixed << std::setprecision(8) << rotation_y << std::endl;
      datei << std::fixed << std::setprecision(8) << rotation_x << std::endl;
      datei << std::fixed << std::setprecision(8) << resolution_y << std::endl;
      datei << std::fixed << std::setprecision(8) << coordinate_x << std::endl;
      datei << std::fixed << std::setprecision(8) << coordinate_y << std::endl;
      datei << std::fixed << std::setprecision(8) << coordinate_z << std::endl;
      datei << std::fixed << std::setprecision(8) << origin_x << std::endl;
      datei << std::fixed << std::setprecision(8) << origin_y << std::endl;
    }
  }
  /**
     * reads a world file with six lines
     * @param filename
     * @return true on error otherwise false
     **/
  bool read_jgw(const std::string & filename)
  {
    std::ifstream geo_info_file(filename.c_str());
    // Check if the file is open
    if (!geo_info_file.is_open()) {
      std::cerr << "Error opening the file!" << std::endl;
      return true;  // Return an error code
    }

    // Read six lines
    double num;
    if (geo_info_file >> num) {
      resolution_x = num;  /// Line 1
    }
    if (geo_info_file >> num) {
      rotation_y = num;  /// Line 2
    }
    if (geo_info_file >> num) {
      rotation_x = num;  /// Line 3
    }
    if (geo_info_file >> num) {
      resolution_y = num;  /// Line 4
    }
    if (geo_info_file >> num) {
      coordinate_x = num;  /// Line 5
    }
    if (geo_info_file >> num) {
      coordinate_y = num;  /// Line 6
    }
    try {
      if (geo_info_file >> num) {
        coordinate_z = num;  /// Line 7
      }
      if (geo_info_file >> num) {
        origin_x = num;  /// Line 8
      }
      if (geo_info_file >> num) {
        origin_y = num;  /// Line 9
      }
    } catch (std::ifstream::failure & e) {
    }

    geo_info_file.close();
    return false;
  }
};

/**
   * class to hold geographic meta data for a map
   * it allows to access pixels based on geo information
   * In order to use it the GeographicLib must be installed (check the CMakeFile.txt)
   **/
class GeoHdl : public MapHdl
{
  cv::Vec3d utm_;  /// utm offset to map origin
  double gamma_;   /// meridian convergence at point [degree]
  int zone_;       /// utm zone id
  bool northp_;    /// true if map is on northern hemisphere

public:
  GeoHdl() {}

  /**
     * @return geo information as string to print
     **/
  std::string info_geo() const
  {
    char txt[0x1FF];
    cv::Vec3d lla = utm2lla(utm_);
    sprintf(
      txt,
      "GeoInfo: [%12.10f°, %12.10f°, %12.10fm] --> [%12.10fm, %12.10fm, %12.10fm] zone %d %s, "
      "origin [%5.2fm, %5.2fm]",
      lla[0], lla[1], lla[2], utm_[0], utm_[1], utm_[2], zone_, (northp_ ? "north" : "south"),
      origin_x(), origin_y());
    return txt;
  }
  /**
     * used to initialize the figure
     * @param canvas_size pixel size of the canvas [pix]
     * @param resolution resolution size of a pixel [m/pix]
     * @param origin origin
     * @param lla  latitude longitude altitude of origin
     **/
  void init(cv::Size canvas_size, double resolution, cv::Point2d origin, cv::Vec3d lla)
  {
    double gamma, k;
    GeographicLib::UTMUPS::Forward(
      lla[0], lla[1], this->zone_, this->northp_, this->utm_[0], this->utm_[1], gamma, k);
    this->gamma_ = gamma_ * M_PI / 180.0;
    this->utm_[2] = lla[3];
    MapHdl::init(canvas_size, resolution, origin, this->gamma_);
  }

  /**
     * used to initialize the figure
     * @param canvas_size pixel size of the canvas [pix]
     * @param resolution resolution size of a pixel [m/pix]
     * @param origin origin
     * @param utm  easting northing altitude of origin
     * @param zone  utm zone
     * @param northp  true if map is on northern hemisphere
     **/
  void init(
    cv::Size canvas_size, double resolution, cv::Point2d origin, cv::Vec3d utm, int zone,
    bool northp)
  {
    double lat, lon, gamma, k;
    GeographicLib::UTMUPS::Reverse(zone, northp, utm[0], utm[1], lat, lon, gamma, k);
    this->utm_ = utm;
    this->gamma_ = gamma_ * M_PI / 180.0;
    this->zone_ = zone;
    this->northp_ = northp;
    MapHdl::init(canvas_size, resolution, origin, this->gamma_);
  }

  /**
     * latitude longitude altitude -> utm
     * the utm map depents on the map init
     * @param src latitude longitude altitude
     * @param des utm x, y, z
     * @return utm x, y, z
     **/
  cv::Vec3d & lla2utm(const cv::Vec3d & src, cv::Vec3d & des) const
  {
    int zone;
    double gamma, k;
    bool northp;
    GeographicLib::UTMUPS::Forward(
      src[0], src[1], zone, northp, des[0], des[1], gamma, k, this->zone_);
    des[2] = src[2];
    return des;
  }
  /**
     * latitude longitude altitude -> utm
     * the utm map depents on the map init
     * @param src latitude longitude altitude
     * @return utm x, y, z
     **/
  cv::Vec3d lla2utm(const cv::Vec3d & src) const
  {
    cv::Vec3d des;
    return lla2utm(src, des);
  }

  /**
     * latitude longitude altitude -> map [pix]
     * the utm map depents on the map init
     * @param src latitude longitude altitude
     * @return map x, y
     **/
  cv::Point & lla2map(const cv::Vec3d & src, cv::Point & des) const
  {
    cv::Vec3d utm;
    lla2utm(src, utm);
    cv::Vec3d world;
    utm2world(utm, world);
    return this->w2m(world[0], world[1]).to(des);
  }
  /**
     * latitude longitude altitude -> map [pix]
     * the utm map depents on the map init
     * @param src latitude longitude altitude
     * @return map x, y
     **/
  cv::Point lla2map(const cv::Vec3d & src) const
  {
    cv::Point des;
    return lla2map(src, des);
  }
  /**
     * latitude longitude altitude -> world [m]
     * the utm map depents on the map init
     * @param src latitude longitude altitude
     * @return world x, y, z
     **/
  cv::Vec3d & lla2world(const cv::Vec3d & src, cv::Vec3d & des) const
  {
    cv::Vec3d utm;
    lla2utm(src, utm);
    return utm2world(utm, des);
  }
  /**
     * latitude longitude altitude -> world [m]
     * the utm map depents on the map init
     * @param src latitude longitude altitude
     * @return world x, y, z
     **/
  cv::Vec3d lla2world(const cv::Vec3d & src) const
  {
    cv::Vec3d des;
    return lla2world(src, des);
  }

  /**
     * utm -> world map [m]
     * the utm map depents on the map init
     * @param src utm x, y, z
     * @param des x, y, z
     * @return x, y, z
     **/
  cv::Vec3d & utm2world(const cv::Vec3d & src, cv::Vec3d & des) const
  {
    des = src - utm_;
    return des;
  }
  /**
     * utm -> world map [m]
     * the utm map depents on the map init
     * @param src utm x, y, z
     * @return x, y, z
     **/
  cv::Vec3d utm2world(const cv::Vec3d & src) const
  {
    cv::Vec3d des;
    return utm2world(src, des);
  }
  /**
     * world map [m] -> utm
     * the utm map depents on the map init
     * @param des world x, y, z
     * @param src utm x, y, z
     * @return x, y, z
     **/
  cv::Vec3d & world2utm(const cv::Vec3d & src, cv::Vec3d & des) const
  {
    des = src + utm_;
    return des;
  }
  /**
     * world map [m] -> utm
     * the utm map depents on the map init
     * @param des world x, y, z
     * @param src utm x, y, z
     * @return x, y, z
     **/
  cv::Vec3d world2utm(const cv::Vec3d & src) const
  {
    cv::Vec3d des;
    return world2utm(src, des);
  }

  /**
     * world [m] -> map [pix]
     * @param src x, y
     * @param des x, y
     * @return x, y
     **/
  cv::Vec2d & world2map(const cv::Vec2d & src, cv::Vec2d & des) const
  {
    cv::Vec3d tmp = w2m(cv::Vec3d(src[0], src[1], 1.));
    des[0] = tmp[0], des[1] = tmp[1];
    return des;
  }
  /**
     * world [m] -> ma
     * p [pix]
     * @param src x, y
     * @return x, y
     **/
  cv::Vec2d world2map(const cv::Vec2d & src) const
  {
    cv::Vec2d des;
    return world2map(src, des);
  }

  /**
     * utm [m] -> geo [latitude longitude altitude]
     * @param src x, y, z
     * @param des latitude longitude altitude
     * @return latitude longitude altitude
     **/
  cv::Vec3d & utm2lla(const cv::Vec3d & src, cv::Vec3d & des) const
  {
    GeographicLib::UTMUPS::Reverse(zone_, northp_, src[0], src[1], des[0], des[1]);
    des[2] = src[2];
    return des;
  }
  /**
     * utm [m] -> geo [latitude longitude altitude]
     * @param src x, y, z
     * @return latitude longitude altitude
     **/
  cv::Vec3d utm2lla(const cv::Vec3d & src) const
  {
    cv::Vec3d des;
    return utm2lla(src, des);
  }

  /**
     * map [pix] -> latitude longitude altitude
     * the utm map depents on the map init
     * @param src  map x, y [pix]
     * @param des  latitude longitude altitude
     * @return latitude longitude altitude
     **/
  cv::Vec3d & map2lla(const cv::Point & src, cv::Vec3d & des) const
  {
    tuw::Point2D p_map(src.x, src.y);
    tuw::Point2D p_world;
    m2w(p_map, p_world);
    cv::Vec3d utm = world2utm(cv::Vec3d(p_world.x(), p_world.y(), 0));
    utm2lla(utm, des);
    return des;
  }
  /**
     * map [pix] -> latitude longitude altitude
     * the utm map depents on the map init
     * @param src  map x, y [pix]
     * @return latitude longitude altitude
     **/
  cv::Vec3d map2lla(const cv::Point & src) const
  {
    cv::Vec3d des;
    return map2lla(src, des);
  }
  /**
   * utm offset to map
   * @return x, y, z
  **/
  cv::Vec3d utm() {return utm_;}
  /**
   * utm zone
   * @return utm zone id
  **/
  int zone() {return zone_;}

  /**
   * Northern Hemisphere
   * @return true if map is on northern hemisphere
  **/
  bool is_north() {return northp_;}

  /**
   * Southern Hemisphere
   * @return true if map is on southern hemisphere
  **/
  bool is_south() {return !northp_;}
};
}  // namespace tuw
#endif  // TUW_GEOMETRY__GEO_HANDLER_HPP
