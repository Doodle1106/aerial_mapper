/*
 *    Filename: main-dense-pcl.cc
 *  Created on: Jun 25, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

// SYSTEM
#include <string>

// NON-SYSTEM
#include <aerial-mapper-dense-pcl/stereo.h>
#include <aerial-mapper-io/aerial-mapper-io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>

// ROS RELATED
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/duration.h>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "../../aerial_mapper_dense_pcl/include/aerial-mapper-dense-pcl/common.h"

using namespace std;

DEFINE_string(data_directory, "",
              "Directory to poses, images, and calibration file.");
DEFINE_string(
    filename_camera_rig, "",
    "Name of the camera calibration file. (intrinsics). File ending: .yaml");
DEFINE_string(filename_poses, "",
              "Name of the file that contains positions and orientations for "
              "every camera in the global/world frame, i.e. T_G_B");
DEFINE_string(prefix_images, "",
              "Prefix of the images to be loaded, e.g. 'images_'");
DEFINE_int32(dense_pcl_use_every_nth_image, 10,
             "Only use every n-th image in the densification process");
DEFINE_bool(use_BM, true,
            "Use BM Blockmatching if true. Use SGBM (=Semi-Global-) "
            "Blockmatching if false.");


stereo::Stereo* pStereo;
Pose Twb;

void PoseCallback(const geometry_msgs::PoseStamped& local_pose)
{
    aslam::Quaternion q(local_pose.pose.orientation.w,
                        local_pose.pose.orientation.x,
                        local_pose.pose.orientation.y,
                        local_pose.pose.orientation.z);
    Eigen::Vector3d t(local_pose.pose.position.x,
                      local_pose.pose.position.y,
                      local_pose.pose.position.z);

    aslam::Transformation T(q, t);
    Twb = T;
}

void FetchImageCallback(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    cv_bridge::CvImageConstPtr cv_ptrRight;

    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat imLeftRect, imRightRect;

    /*
        const Pose& T_G_B, const Image& image_raw,
                      AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud,
                      std::vector<int>* point_cloud_intensities
    */
    AlignedType<std::vector, Eigen::Vector3d>::type* point_cloud;
    std::vector<int>* point_cloud_intensities;

    pStereo->addFrame(Twb, imLeftRect, point_cloud, point_cloud_intensities);

}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  ros::init(argc, argv, "main_dense_pcl");

  // Parse input parameters.
  const std::string& base = FLAGS_data_directory;
  const std::string& filename_camera_rig = FLAGS_filename_camera_rig;
  const std::string& filename_poses = FLAGS_filename_poses;
  const std::string& filename_images = base + FLAGS_prefix_images;

  LOG(INFO) << "Loading camera rig from file.";
  io::AerialMapperIO io_handler;
  const std::string& filename_camera_rig_yaml = base + filename_camera_rig;
  std::shared_ptr<aslam::NCamera> ncameras =
      io_handler.loadCameraRigFromFile(filename_camera_rig_yaml);
  CHECK(ncameras);

  LOG(INFO) << "Loading body poses from file.";
  Poses T_G_Bs;
  const std::string& path_filename_poses = base + filename_poses;
  io::PoseFormat pose_format = io::PoseFormat::Standard;
  io_handler.loadPosesFromFile(pose_format, path_filename_poses, &T_G_Bs);

  LOG(INFO) << "Loading images from file.";
  size_t num_poses = T_G_Bs.size();
  Images images;
  io_handler.loadImagesFromFile(filename_images, num_poses, &images);

  stereo::Settings settings_dense_pcl;
  settings_dense_pcl.use_every_nth_image = FLAGS_dense_pcl_use_every_nth_image;
  LOG(INFO) << "Perform dense reconstruction using planar rectification.";
  stereo::BlockMatchingParameters block_matching_params;
  block_matching_params.use_BM = FLAGS_use_BM;
  stereo::Stereo stereo(ncameras, settings_dense_pcl, block_matching_params);
  AlignedType<std::vector, Eigen::Vector3d>::type point_cloud;

  //////////////////////////////////////////////////
  // stereo.addFrames(T_G_Bs, images, &point_cloud);
  //////////////////////////////////////////////////

  
  pStereo = &stereo;

  ros::init(argc, argv, "aerial_mapper_ros");
  ros::NodeHandle nh;
  
  string left_topic = "/gi/simulation/left/image_raw";
  string right_topic = "/gi/simulation/right/image_raw";
  message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, left_topic, 10);
  message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, right_topic, 10);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
  sync.setMaxIntervalDuration(ros::Duration(0.01));
  sync.registerCallback(boost::bind(FetchImageCallback, _1, _2));

  ros::Subscriber drone_local_position = nh.subscribe("/mavros/local_position/pose", 5, PoseCallback);

  ros::spin();
    
  return 0;
}
