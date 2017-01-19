#ifndef LINESEGMENT2D_DETECTOR_NODE_H
#define LINESEGMENT2D_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tuw_geometry/measurement_laser.h>
#include <tuw_geometry/linesegment2d_detector.h>

namespace tuw
{

class Linesegment2DDetectorNode : public LineSegment2DDetector
{
public:
  Linesegment2DDetectorNode();
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_laser_; /// Subscriber to the laser measurements
  ros::Publisher line_pub_;
  MeasurementLaserPtr measurement_laser_;  /// laser measurements
  std::vector<Point2D> measurement_local_scanpoints_; /// laser beam endpoints for line detection
  std::vector<LineSegment2D> measurement_linesegments_;    /// detected line segments in sensor coordinates
  void callbackLaser(const sensor_msgs::LaserScan &_laser);
};

};

#endif // LINESEGMENT2D_DETECTOR_NODE_H