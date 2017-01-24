#include "tuw_geometry/linesegment2d_detector_node.h"

#include <tuw_geometry_msgs/LineSegment.h>
#include <tuw_geometry_msgs/LineSegments.h>

using namespace tuw;

Linesegment2DDetectorNode::Linesegment2DDetectorNode() : nh_private_("~")
{
  sub_laser_ = nh_.subscribe("scan", 1000, &Linesegment2DDetectorNode::callbackLaser, this);
  line_pub_ = nh_.advertise<tuw_geometry_msgs::LineSegments>("line_segments", 1000);
  measurement_laser_ = std::make_shared<tuw::MeasurementLaser>();

  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  nh_private_.param("laser_frame", laser_frame_, std::string("laser"));

  reconfigure_fnc_ = boost::bind(&Linesegment2DDetectorNode::callbackConfig, this, _1, _2);
  reconfigure_server_.setCallback(reconfigure_fnc_);
}

void Linesegment2DDetectorNode::callbackConfig(tuw_geometry::Linesegment2DDetectorConfig& config, uint32_t level)
{
  config_.threshold_split_neighbor = config.line_dection_split_neighbor;
  config_.threshold_split = config.line_dection_split_threshold;
  config_.min_length = config.line_dection_min_length;
  config_.min_points_per_line = config.line_dection_min_points_per_line;
  config_.min_points_per_unit = config.line_dection_min_points_per_unit;
}

void Linesegment2DDetectorNode::callbackLaser(const sensor_msgs::LaserScan& _laser)
{
  int nr = (_laser.angle_max - _laser.angle_min) / _laser.angle_increment;
  measurement_laser_->range_max() = _laser.range_max;
  measurement_laser_->range_min() = _laser.range_min;
  measurement_laser_->resize(nr);
  measurement_laser_->stamp() = _laser.header.stamp.toBoost();
  for (int i = 0; i < nr; i++)
  {
    MeasurementLaser::Beam& beam = measurement_laser_->operator[](i);
    beam.length = _laser.ranges[i];
    beam.angle = _laser.angle_min + (_laser.angle_increment * i);
    beam.end_point.x() = cos(beam.angle) * beam.length;
    beam.end_point.y() = sin(beam.angle) * beam.length;
  }

  measurement_local_scanpoints_.resize(measurement_laser_->size());
  for (size_t i = 0; i < measurement_laser_->size(); i++)
  {
    measurement_local_scanpoints_[i] = measurement_laser_->operator[](i).end_point;
  }
  measurement_linesegments_.clear();
  start(measurement_local_scanpoints_, measurement_linesegments_);

  // publish found line segments
  tuw_geometry_msgs::LineSegment line_segment_msg;
  tuw_geometry_msgs::LineSegments line_segments_msg;
  for (int i = 0; i < measurement_linesegments_.size(); i++)
  {
    line_segment_msg.p0.x = measurement_linesegments_[i].p0().x();
    line_segment_msg.p0.y = measurement_linesegments_[i].p0().y();
    line_segment_msg.p0.z = 0;
    line_segment_msg.p1.x = measurement_linesegments_[i].p1().x();
    line_segment_msg.p1.y = measurement_linesegments_[i].p1().y();
    line_segment_msg.p1.z = 0;
    line_segments_msg.segments.push_back(line_segment_msg);
  }
  // set header information
  line_segments_msg.header.frame_id = laser_frame_;
  line_segments_msg.header.stamp = ros::Time::now();

  line_pub_.publish(line_segments_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "linesegment2d_detector_node");

  Linesegment2DDetectorNode detector_node;

  ros::spin();

  return 0;
}