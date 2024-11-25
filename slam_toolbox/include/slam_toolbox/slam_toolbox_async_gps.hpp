/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
 * Copyright Work Modifications (c) 2019, Steve Macenski
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#ifndef SLAM_TOOLBOX_SLAM_TOOLBOX_ASYNC_GPS_H_
#define SLAM_TOOLBOX_SLAM_TOOLBOX_ASYNC_GPS_H_

#include <memory>
#include <deque>
#include "slam_toolbox/slam_toolbox_common.hpp"
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread.hpp>

namespace slam_toolbox
{

class AsynchronousGPSSlamToolbox : public SlamToolbox
{
public:
  AsynchronousGPSSlamToolbox(ros::NodeHandle& nh);
  ~AsynchronousGPSSlamToolbox();

protected:
  virtual void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) override final;
  virtual bool deserializePoseGraphCallback(
    slam_toolbox_msgs::DeserializePoseGraph::Request& req,
    slam_toolbox_msgs::DeserializePoseGraph::Response& resp) override final;

private:
  void absolutePoseCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void globalOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);  
  bool shouldProcessAbsolutePose();
  bool isValidCovariance(Eigen::Matrix<double, 6, 6>& cov);
  karto::LocalizedRangeScan* addGpsScan(
    karto::LaserRangeFinder* laser,
    const sensor_msgs::LaserScan::ConstPtr& scan,
    karto::Pose2& pose,
    const karto::GPSMeasurement* gps);
  
  // Subscribers
  ros::Subscriber absolute_pose_sub_, global_odom_sub_;
  
  // Parameters
  int absolute_pose_rate_;
  double sync_threshold_;
  int pose_count_;

  bool received_first_gps_;
  bool first_scan_processed_;
  bool has_global_odom_;

  nav_msgs::Odometry latest_global_odom_;
  
  // Buffer for recent absolute poses
  std::deque<nav_msgs::Odometry::ConstPtr> absolute_pose_buffer_;
  boost::mutex buffer_mutex_;
  
  static constexpr size_t MAX_BUFFER_SIZE = 100;
  static constexpr double DEFAULT_COVARIANCE_THRESHOLD = 0.2;  // 10cm std dev threshold
};

} // namespace slam_toolbox

#endif // SLAM_TOOLBOX_SLAM_TOOLBOX_ASYNC_GPS_H_
