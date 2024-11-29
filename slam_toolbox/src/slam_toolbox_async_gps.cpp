#include "slam_toolbox/slam_toolbox_async_gps.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
AsynchronousGPSSlamToolbox::AsynchronousGPSSlamToolbox(ros::NodeHandle& nh)
    : SlamToolbox(nh), 
    pose_count_(0), 
    received_first_gps_(false), 
    first_scan_processed_(false), 
    has_global_odom_(false), 
    gps_manager_(std::make_unique<GPSEstimationManager>())
/*****************************************************************************/
{
    // Get parameters
    nh.param("absolute_pose_rate", absolute_pose_rate_, 10);
    nh.param("sync_threshold", sync_threshold_, 0.1);
    std::string absolute_pose_topic;
    nh.param<std::string>("absolute_pose_topic", absolute_pose_topic, "/odometry/gps");
    processor_type_ = PROCESS_FIRST_NODE;

    // Subscribe to absolute pose topic
    absolute_pose_sub_ = nh.subscribe(
            absolute_pose_topic, 10,
            &AsynchronousGPSSlamToolbox::absolutePoseCallback, this);

    global_odom_sub_ = nh.subscribe(
            "/odometry/filtered/global", 10,
            &AsynchronousGPSSlamToolbox::globalOdomCallback, this);

    ROS_INFO("Initialized GPS-assisted SLAM - waiting for first valid GPS message...");

    loadPoseGraphByParams(nh);
}

/*****************************************************************************/
AsynchronousGPSSlamToolbox::~AsynchronousGPSSlamToolbox()
/*****************************************************************************/
{
}

/*****************************************************************************/
void AsynchronousGPSSlamToolbox::globalOdomCallback(
        const nav_msgs::Odometry::ConstPtr& msg)
/*****************************************************************************/
{
    latest_global_odom_ = *msg;
    has_global_odom_ = true;
}

/*****************************************************************************/
void AsynchronousGPSSlamToolbox::absolutePoseCallback(
        const nav_msgs::Odometry::ConstPtr& msg)
/*****************************************************************************/
{
    boost::mutex::scoped_lock lock(buffer_mutex_);

    absolute_pose_buffer_.push_back(msg);
    if (absolute_pose_buffer_.size() > MAX_BUFFER_SIZE)
    {
        absolute_pose_buffer_.pop_front();
    }

    // Check if this is a valid GPS message and we haven't received one yet
    if (!received_first_gps_)
    {
        received_first_gps_ = true;
        ROS_INFO("Received first valid GPS measurement - SLAM system now active");
    }
}

/*****************************************************************************/
void AsynchronousGPSSlamToolbox::laserCallback(
        const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
    // Drop scans until we get first GPS
    if (!received_first_gps_)
    {
        ROS_DEBUG_THROTTLE(1.0, "Waiting for first valid GPS measurement before starting SLAM...");
        return;
    }

    // Get odometry pose
    karto::Pose2 pose;
    if (!pose_helper_->getOdomPose(pose, scan->header.stamp))
    {
        ROS_WARN_THROTTLE(5., "Failed to compute odom pose");
        return;
    }

    // Ensure the laser can be used
    karto::LaserRangeFinder* laser = getLaser(scan);
    if (!laser)
    {
        ROS_WARN_THROTTLE(5., "Failed to create laser device for %s; discarding scan",
                          scan->header.frame_id.c_str());
        return;
    }

    // Find matching absolute pose if needed
    karto::PointGps gps;
    bool has_gps = false;

    // Always try to get GPS for the first scan
    if (!first_scan_processed_ || shouldProcessAbsolutePose())
    {
        boost::mutex::scoped_lock lock(buffer_mutex_);
        // Find closest timestamp match in buffer
        
        double time_diff = std::abs(
                (scan->header.stamp - latest_global_odom_.header.stamp).toSec());
        if (time_diff < sync_threshold_)
        {
            // Combine position and orientation covariances into one 3x3 matrix
            Eigen::Matrix<double, 6, 6> cov;
            cov.setZero();

            // set covariance from latest_global_odom_?
            
            // Now validate the combined covariance
            if (isValidCovariance(latest_global_odom_.pose.covariance))
            {
                gps.SetX(latest_global_odom_.pose.pose.position.x);
                gps.SetY(latest_global_odom_.pose.pose.position.y);
                gps.SetYaw(tf2::getYaw(latest_global_odom_.pose.pose.orientation) + M_PI);
                has_gps = true;
            }
        }

        // If this is supposed to be the first scan but we don't have GPS, wait
        if (!first_scan_processed_ && !has_gps)
        {
            ROS_DEBUG_THROTTLE(1.0, "Waiting for synchronized GPS measurement for first scan...");
            return;
        }
    }

    // Use appropriate scan function based on GPS availability
    if (has_gps)
    {
        addGpsScan(laser, scan, pose, &gps);
    }
    else if (first_scan_processed_)
    { // Only allow non-GPS scans after first scan
        addGpsScan(laser, scan, pose, nullptr);
    }

    // Mark first scan as processed if we got here
    if (!first_scan_processed_ && has_gps)
    {
        first_scan_processed_ = true;
        ROS_INFO("First scan processed with GPS constraint");
    }

    pose_count_++;
}

/*****************************************************************************/
karto::LocalizedRangeScan* AsynchronousGPSSlamToolbox::addGpsScan(
        karto::LaserRangeFinder* laser,
        const sensor_msgs::LaserScan::ConstPtr& scan,
        karto::Pose2& odom_pose,
        const karto::PointGps* gps)
/*****************************************************************************/
{
    // get our localized range scan
    karto::LocalizedRangeScan* range_scan = getLocalizedRangeScan(laser, scan, odom_pose);

    // Add the localized range scan to the mapper
    boost::mutex::scoped_lock lock(smapper_mutex_);
    bool processed = false, update_reprocessing_transform = false;
    karto::Matrix3 covariance;
    covariance.SetToIdentity();

    // For first scan with GPS, set the initial pose from global odometry
    if (processor_type_ == PROCESS_FIRST_NODE && has_global_odom_)
    {
        // Create pose from global odometry
        karto::Pose2 global_pose(
                latest_global_odom_.pose.pose.position.x,
                latest_global_odom_.pose.pose.position.y,
                tf2::getYaw(latest_global_odom_.pose.pose.orientation));

        // Set the pose
        range_scan->SetOdometricPose(global_pose);
        range_scan->SetCorrectedPose(global_pose);

        ROS_INFO("Initializing SLAM at global pose: (%0.2f, %0.2f), theta=%0.2f",
                 global_pose.GetX(), global_pose.GetY(), global_pose.GetHeading());

        processed = smapper_->getMapper()->Process(range_scan, &covariance);
        processor_type_ = PROCESS;
        update_reprocessing_transform = true;
    }
    else if (processor_type_ == PROCESS)
    {
        if (gps)
        {
            range_scan->SetGpsReading(*gps);
        }
        processed = smapper_->getMapper()->Process(range_scan, &covariance);
    }
    else
    {
        ROS_FATAL("SlamToolbox: No valid processor type set! Exiting.");
        exit(-1);
    }

    // if successfully processed, create odom to map transformation
    // and add our scan to storage
    if (processed)
    {
        if (enable_interactive_mode_)
        {
            scan_holder_->addScan(*scan);
        }

        setTransformFromPoses(
                range_scan->GetCorrectedPose(),
                odom_pose,
                scan->header.stamp,
                update_reprocessing_transform);
        dataset_->Add(range_scan);
        publishPose(
                range_scan->GetCorrectedPose(),
                covariance,
                scan->header.stamp);
    }
    else
    {
        delete range_scan;
        range_scan = nullptr;
    }

    return range_scan;
}

/*****************************************************************************/
bool AsynchronousGPSSlamToolbox::isValidCovariance(
        boost::array<double, 36> cov)
/*****************************************************************************/
{
    // Check if position covariance is below threshold
    // Covariance is in row-major order, indices 0 and 7 are x and y variances
    return (std::sqrt(cov[0]) < DEFAULT_COVARIANCE_THRESHOLD &&
            std::sqrt(cov[7]) < DEFAULT_COVARIANCE_THRESHOLD &&
            std::sqrt(cov[35]) < DEFAULT_COVARIANCE_THRESHOLD);
}

/*****************************************************************************/
bool AsynchronousGPSSlamToolbox::shouldProcessAbsolutePose()
/*****************************************************************************/
{
    return (pose_count_ % absolute_pose_rate_) == 0;
}

/*****************************************************************************/
bool AsynchronousGPSSlamToolbox::deserializePoseGraphCallback(
        slam_toolbox_msgs::DeserializePoseGraph::Request& req,
        slam_toolbox_msgs::DeserializePoseGraph::Response& resp)
/*****************************************************************************/
{
    if (req.match_type == procType::LOCALIZE_AT_POSE)
    {
        ROS_ERROR("Requested a localization deserialization in non-localization mode.");
        return false;
    }
    return SlamToolbox::deserializePoseGraphCallback(req, resp);
}

} // namespace slam_toolbox
