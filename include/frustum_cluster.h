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

#include <iostream>
#include <limits>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include <vector_map/vector_map.h>
#include <tf/tf.h>

#include <yaml-cpp/yaml.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>

#if (CV_MAJOR_VERSION == 3)
#include "gencolors.cpp"
#else
#include <opencv2/contrib/contrib.hpp>
#endif

#include "cluster.h"

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

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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

  ros::Publisher _pub_cluster_cloud;
  ros::Publisher _pub_ground_cloud;
  ros::Publisher _centroid_pub;
  ros::Publisher _pub_clusters_message;
  ros::Publisher _pub_points_lanes_cloud;
  ros::Publisher _pub_detected_objects;
  std_msgs::Header _velodyne_header;
  std::string _output_frame;

  static bool _velodyne_transform_available;
  static bool _downsample_cloud;
  static bool _pose_estimation;
  static double _leaf_size;
  static int _cluster_size_min;
  static int _cluster_size_max;
  static const double _initial_quat_w = 1.0;

  static bool _remove_ground;  // only ground

  static bool _using_sensor_cloud;
  static bool _use_diffnormals;

  static bool _keep_lanes;
  static double _keep_lane_left_distance;
  static double _keep_lane_right_distance;

  static double _remove_points_upto;
  static double _cluster_merge_threshold;
  static double _clustering_distance;

  static bool _use_gpu;
  static std::chrono::system_clock::time_point _start, _end;

  std::vector<std::vector<geometry_msgs::Point>> _way_area_points;
  std::vector<cv::Scalar> _colors;
  pcl::PointCloud<pcl::PointXYZ> _sensor_cloud;
  visualization_msgs::Marker _visualization_marker;

  static bool _use_multiple_thres;
  std::vector<double> _clustering_distances;
  std::vector<double> _clustering_ranges;

  tf::StampedTransform *_transform;
  tf::StampedTransform *_velodyne_output_transform;
  tf::TransformListener *_transform_listener;
  tf::TransformListener *_vectormap_transform_listener;

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

  void publishCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr);
  void publishColorCloud(const ros::Publisher *in_publisher,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr);
  void publishCentroids(const ros::Publisher *in_publisher, const autoware_msgs::Centroids &in_centroids,
                      const std::string &in_target_frame, const std_msgs::Header &in_header);
  void publishCloudClusters(const ros::Publisher *in_publisher, const autoware_msgs::CloudClusterArray &in_clusters,
                          const std::string &in_target_frame, const std_msgs::Header &in_header);
  void segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                       autoware_msgs::Centroids &in_out_centroids, autoware_msgs::CloudClusterArray &in_out_clusters);
  void checkAllForMerge(std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                      float in_merge_threshold);
  void mergeClusters(const std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                   std::vector<size_t> in_merge_indices, const size_t &current_index,
                   std::vector<bool> &in_out_merged_clusters);
  void checkClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr> &in_clusters,
                       std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices,
                       double in_merge_threshold);
  std::vector<ClusterPtr> clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                        autoware_msgs::Centroids &in_out_centroids,
                                        double in_max_cluster_distance);
  void publishDetectedObjects(const autoware_msgs::CloudClusterArray &in_clusters);

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
