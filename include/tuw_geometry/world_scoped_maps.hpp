#ifndef TUW_GEOMETRY__WORLD_SCOPED_MAPS_HPP
#define TUW_GEOMETRY__WORLD_SCOPED_MAPS_HPP
#include <opencv2/core/core_c.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tuw_geometry/map_handler.hpp>
#include <tuw_geometry/pose2d.hpp>

namespace tuw
{
class WorldScopedMaps;  /// Prototype
using WorldScopedMapsPtr = std::shared_ptr<WorldScopedMaps>;
using WorldScopedMapsConstPtr = std::shared_ptr<WorldScopedMaps const>;

/**
 * class to visualize information using OpenCV matrices
 **/
class WorldScopedMaps : public MapHdl
{
public:
  //special class member functions
  WorldScopedMaps();
  virtual ~WorldScopedMaps() = default;
  WorldScopedMaps(const WorldScopedMaps &) = default;
  WorldScopedMaps & operator=(const WorldScopedMaps &) = default;
  WorldScopedMaps(WorldScopedMaps &&) = default;
  WorldScopedMaps & operator=(WorldScopedMaps &&) = default;

  /**
     * draws a line given in the visualization space (meter, ....) into a pixel map
     * @param map opencv matrix
     * @param p0 start point
     * @param p1 end point
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
  template<typename T>
  void line(
    T & map, const Point2D & p0, const Point2D & p1, const cv::Scalar & color, int thickness = 1,
    int lineType = cv::LINE_AA) const
  {
    cv::line(map, w2m(p0).cv(), w2m(p1).cv(), color, thickness, lineType);
  }
  /**
     * draws a circle given in the visualization space (meter, ....) into a pixel map
     * @param map opencv matrix
     * @param p location
     * @param radius radius
     * @param color color --> @see opencv
     * @param thickness line thickness --> @see opencv
     * @param lineType line type --> @see opencv
     **/
  template<typename T>
  void circle(
    T & map, const Point2D & p, int radius, const cv::Scalar & color, int thickness = 1,
    int lineType = cv::LINE_AA) const
  {
    cv::circle(map, w2m(p).cv(), radius, color, thickness, lineType);
  }

  /**
     * return a copy of the value located at p in the visual space (meter, ....)
     * @param map opencv matrix
     * @param p location
     **/
  template<typename T>
  cv::Scalar_<T> get(cv::Mat_<T> & map, const Point2D & p) const
  {
    return map.at(w2m(p).cv());
  }
};

}  // namespace tuw
#endif  // TUW_GEOMETRY__WORLD_SCOPED_MAPS_HPP
