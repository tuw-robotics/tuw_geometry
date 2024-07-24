#ifndef TUW_GEOMETRY__MAP_HANDLER_HPP
#define TUW_GEOMETRY__MAP_HANDLER_HPP
#include <opencv2/core/core_c.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tuw_geometry/pose2d.hpp>

namespace tuw
{
class MapHdl;  /// Prototype
using MapHdlPtr = std::shared_ptr<MapHdl>;
using MapHdlConstPtr = std::shared_ptr<MapHdl const>;

/**
 * class to handle transformations form world system [meter] to image system [pixel]
 **/
class MapHdl
{
  cv::Matx33d Mw2m_;                                 ///< transformation world to map
  cv::Matx33d Mm2w_;                                 ///< transformation map to world
  int width_pixel_, height_pixel_;                   ///< dimensions of the canvas in pixel
  double min_x_, max_x_, min_y_, max_y_, rotation_;  ///< area and rotation of the visualized space
  double dx_, dy_;                                   ///< dimension of the visualized space [m]
  double ox_, oy_;                                   ///< image offset [pix]
  double mx_, my_;                                   ///< offset of the visualized space [m]
  double sx_, sy_;                                   ///< scale [pix/m]

  void init();  ///< initializes the transformation matrices

public:
  //special class member functions
  MapHdl();
  virtual ~MapHdl() = default;
  MapHdl(const MapHdl &) = default;
  MapHdl & operator=(const MapHdl &) = default;
  MapHdl(MapHdl &&) = default;
  MapHdl & operator=(MapHdl &&) = default;

  /**
     * used to initialize the figure
     * @param width_pixel pixel size of the canvas
     * @param height_pixel pixel size of the canvas
     * @param Mw2m transformation world to map
     **/
  void init(int width_pixel, int height_pixel, cv::Matx33d Mw2m);

  /**
   * @return map information as string to print
  **/
  std::string info_map() const;

  /**
     * used to initialize the figure
     * @param canvas_size pixel size of the canvas [pix]
     * @param resolution resolution size of a pixel [m/pix]
     * @param origin origin in relative to the top left [m]
     * @param rotation origin in relative to the top left [m]
     **/
  void init(cv::Size canvas_size, double resolution, cv::Point2d origin, double rotation = 0.);

  /**
     * used to initialize the figure
     * @param width_pixel pixel size of the canvas
     * @param height_pixel pixel size of the canvas
     * @param min_x minimal x of the visualized space
     * @param max_x maximal x of the visualized space
     * @param min_y minimal y of the visualized space
     * @param max_y maximal y of the visualized space
     * @param rotation rotation of the visualized spaces
     * @param enforce_positive_axis on true it will check that min_y < max_y and min_x < max_x and corrects if needed
     **/
  void init(
    int width_pixel, int height_pixel, double min_x, double max_x, double min_y, double max_y,
    double rotation = 0, bool enforce_positive_axis = true);

  /**
     * used to initialize the figure based on a ROS nav_msgs/MapMetaData
     * @param T nav_msgs/MapMetaData
     **/
  template<typename T>
  void init(const T & metadata)
  {
    width_pixel_ = metadata.width, height_pixel_ = metadata.height;
    dx_ = metadata.resolution * (double)metadata.width;
    dy_ = metadata.resolution * (double)metadata.height;
    sx_ = 1.0 / metadata.resolution;
    sy_ = 1.0 / metadata.resolution;
    ox_ = 0.;
    oy_ = 0.;
    double roll = 0, pitch = 0, yaw = 0;
    QuaternionToEuler(metadata.origin.orientation, roll, pitch, yaw);
    rotation_ = -yaw;
    rotation_ = 0;
    double ca = cos(rotation_), sa = sin(rotation_);
    mx_ = metadata.origin.position.x;
    my_ = metadata.origin.position.y;
    cv::Matx<double, 3, 3> Tw(1, 0, -mx_, 0, 1, -my_, 0, 0, 1);  // translation
    cv::Matx<double, 3, 3> Sc(sx_, 0, 0, 0, sy_, 0, 0, 0, 1);    // scaling
    cv::Matx<double, 3, 3> R(ca, -sa, 0, sa, ca, 0, 0, 0, 1);    // rotation
    Mw2m_ = R * Sc * Tw;
    Mm2w_ = Mw2m_.inv();
    Point2D p = m2w(width_pixel_, height_pixel_);
    min_y_ = mx_;
    min_x_ = my_;
    max_x_ = p.x();
    max_y_ = p.y();
  }

  /**
     *  @returns true if the figure is initialized
     **/
  bool initialized();

  /**
     * @return transformation matrix from the visualization space to image space (world -> map)
     **/
  const cv::Matx33d & Mw2m() const;
  /**
     * @return transformation matrix from the image space to visualization space (map -> world)
     **/
  const cv::Matx33d & Mm2w() const;

  /**
     * transforms a point from the visualization space to image space (world -> map)
     * @param src point in visualization space (world)
     * @return point in image space (map [pixel])
     **/
  Point2D w2m(const Point2D & src) const;
  /**
     * transforms a point from the visualization space to image space (world -> map)
     * @param x x coordinate in visualization space (world) eg. [m]
     * @param y y coordinate in visualization space (world) eg. [m]
     * @return point in image space  eg. [pixel]
     **/
  Point2D w2m(double x, double y) const;
  /**
     * transforms a point from the visualization space to image space (world -> map)
     * @param src point in visualization space (world)
     * @param des point in image space (map [pixel])
     * @return reference to des
     **/
  Point2D & w2m(const Point2D & src, Point2D & des) const;
  /**
     * transforms a point from the visualization space to image space (world -> map)
     * @param src point in visualization space (world)
     * @param des point in image space (map [pixel])
     * @return reference to des
     **/
  cv::Point2d & w2m(const cv::Point2d & src, cv::Point2d & des) const;
  /**
     * transforms a point from the visualization space to image space (world -> map)
     * @param src point in visualization space (world)
     * @return point in image space (map [pixel])
     **/
  cv::Point w2m(const cv::Point2d & src) const;
  /**
     * transforms a point from the visualization space to image space (world -> map)
     * @param src point in visualization space (world)
     * @param des point in image space (map [pixel])
     * @return reference to des
     **/
  cv::Point & w2m(const cv::Point2d & src, cv::Point & des) const;
  /**
     * transforms a point from the image space to visualization space (map -> world)
     * @param src point in image space (map [pixel])
     * @return point in visualization space (world)
     **/
  Point2D m2w(const Point2D & src) const;
  /**
     * transforms a point from the image space to visualization space (map -> world)
     * @param x x coordinate in image space  eg. [pixel]
     * @param y y coordinate in image space  eg. [pixel]
     * @return point in visualization space (world) eg. [m]
     **/
  Point2D m2w(double x, double y) const;
  /**
     * transforms a point from the image space to visualization space (map -> world)
     * @param src point in image space (map [pixel])
     * @param des  point in visualization space (world)
     * @return reference to des
     **/
  Point2D & m2w(const Point2D & src, Point2D & des) const;
  /**
     * transforms a point from the image space to visualization space (map -> world)
     * @param src point in image space (map [pixel])
     * @param des  point in visualization space (world)
     * @return reference to des
     **/
  cv::Point2d & m2w(const cv::Point2d & src, cv::Point2d & des) const;
  /**
     * transforms a point from the image space to visualization space (map -> world)
     * @param src point in image space (map [pixel])
     * @return point in visualization space (world)
     **/
  cv::Point2d m2w(const cv::Point & src) const;
  /**
     * transforms a point from the image space to visualization space (map -> world)
     * @param src point in image space (map [pixel])
     * @param des  point in visualization space (world)
     * @return reference to des
     **/
  cv::Point2d & m2w(const cv::Point & src, cv::Point2d & des) const;

  /**
     * @return canvas (image) width
     **/
  int width() const;
  /**
     * @return canvas (image) height
     **/
  int height() const;
  /**
     * @return canvas (image) size
     **/
  cv::Size size() const;
  /**
   * @return offest to the the zero x in [m]
   **/
  double origin_x() const;
  /**
   * @return offest to the the zero y in [m]
   **/
  double origin_y() const;
  /**
     * @return computed x resolution
     **/
  double resolution_x() const;
  /**
     * @return computed x scale
     **/
  double scale_x() const;
  /**
   * @return computed y scale
   **/
  double scale_y() const;
  /**
     * @return computed y resolution
     **/
  double resolution_y() const;
  /**
     * @return minimal x of the visualized space
     **/
  double min_x() const;
  /**
     * @return maximal x of the visualized space
     **/
  double max_x() const;
  /**
     * @return minimal y of the visualized space
     **/
  double min_y() const;
  /**
     * @return maximal y of the visualized space
     **/
  double max_y() const;
  /**
    * returns a distance measure scaled
    * @param v value to be scaledt
    * @return distance
    **/
  double scale_w2m(double v) const;
  /**
    * returns information about the maps metadata
    * @param format using printf format
    * @return string
    **/
  std::string infoHeader() const;
};

}  // namespace tuw
#endif  // TUW_GEOMETRY__MAP_HANDLER_HPP
