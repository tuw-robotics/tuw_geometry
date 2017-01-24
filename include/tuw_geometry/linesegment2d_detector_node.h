#ifndef LINESEGMENT2D_DETECTOR_NODE_H
#define LINESEGMENT2D_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tuw_geometry/measurement_laser.h>
#include <tuw_geometry/linesegment2d_detector.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_geometry/Linesegment2DDetectorConfig.h>

namespace tuw
{

class Linesegment2DDetectorNode : public LineSegment2DDetector
{
public:
    Linesegment2DDetectorNode();
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_laser_; /// Subscriber to the laser measurements
  ros::Publisher line_pub_;
  MeasurementLaserPtr measurement_laser_;  /// laser measurements
  std::vector<Point2D> measurement_local_scanpoints_; /// laser beam endpoints for line detection
  std::vector<LineSegment2D> measurement_linesegments_;    /// detected line segments in sensor coordinates
  std::string laser_frame_;
  dynamic_reconfigure::Server<tuw_geometry::Linesegment2DDetectorConfig> reconfigure_server_; /// parameter server stuff general use
  dynamic_reconfigure::Server<tuw_geometry::Linesegment2DDetectorConfig>::CallbackType reconfigure_fnc_; /// parameter server stuff general use
  void callbackConfig(tuw_geometry::Linesegment2DDetectorConfig &config, uint32_t level); /// callback function on incoming parameter changes for general use
  void callbackLaser(const sensor_msgs::LaserScan &_laser);
};

};

#endif // LINESEGMENT2D_DETECTOR_NODE_H
