/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */

#ifndef _VISUALIZERECTS_H
#define _VISUALIZERECTS_H

#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include "autoware_msgs/ProjectionMatrix.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/PointsImage.h"

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#define __APP_NAME__ "visualize_rects"

  std::string input_topic_;
  ros::Subscriber subscriber_detected_objects_;
  image_transport::Subscriber subscriber_image_;

  message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *detection_filter_subscriber_;
  message_filters::Subscriber<sensor_msgs::Image> *image_filter_subscriber_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> *lidar_filter_subscriber_;

  ros::Publisher publisher_image_;
  ros::Publisher publisher_lidar_frustum_;

  cv::Mat image_;
  std_msgs::Header image_header_;

  typedef
  message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
    autoware_msgs::DetectedObjectArray> SyncPolicyT;

  message_filters::Synchronizer<SyncPolicyT>
    *detections_synchronizer_;

  typedef
  message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
    autoware_msgs::DetectedObjectArray> SyncPolicyT_cam_lidar;

  typedef pcl::PointXYZI PointT;

  message_filters::Synchronizer<SyncPolicyT_cam_lidar>
    *detections_synchronizer_lidar_;

  pcl::PointCloud<PointT> to_Frustum_points_3D(pcl::PointCloud<PointT> Points_3D,autoware_msgs::DetectedObjectArray in_objects);

  cv::Point2d project_lidarpoint2imagepoint(PointT Point_3D);

  void projection_callback(autoware_msgs::ProjectionMatrix msg);
  void intrinsic_callback(sensor_msgs::CameraInfo msg);
  void resetMatrix();
  void initMatrix(const cv::Mat& cameraExtrinsicMat);
  autoware_msgs::PointsImage pointcloud2_to_image(const sensor_msgs::PointCloud2ConstPtr& pointcloud2,
                                                const cv::Mat& cameraExtrinsicMat, const cv::Mat& cameraMat,
                                                const cv::Mat& distCoeff, const cv::Size& imageSize);

  void
  SyncedDetectionsCallback(
    const sensor_msgs::Image::ConstPtr &in_image_msg,
    const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections);

  void
  SyncedDetectionsCallback_lidar_cam(
    const sensor_msgs::PointCloud2::ConstPtr &in_lidar_msg,
    const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects);


  bool IsObjectValid(const autoware_msgs::DetectedObject &in_object);

  cv::Mat ObjectsToRects(cv::Mat in_image, const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects);

#endif  // _VISUALIZERECTS_H
