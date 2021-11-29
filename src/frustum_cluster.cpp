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

#include "frustum_cluster.h"

bool init_matrix = false;
cv::Mat cameraExtrinsicMat;
cv::Mat cameraMat;
cv::Mat distCoeff;
cv::Size imageSize;
cv::Mat invRt, invTt;

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "frustum_cluster");
  ros::NodeHandle private_nh_("~");
  ros::NodeHandle node_handle_;

  ros::NodeHandle nh;

  std::string image_src_topic;
  std::string object_src_topic;
  std::string image_out_topic;
  std::string lidar_src_topic;
  std::string lidar_frustum_topic;

  std::string camera_info_topic_str;
  std::string projection_matrix_topic;

  private_nh_.param<std::string>("projection_matrix_topic", projection_matrix_topic, "/projection_matrix");
  private_nh_.param<std::string>("camera_info_topic", camera_info_topic_str, "/camera_info");

  private_nh_.param<std::string>("image_src", image_src_topic, "/image_raw");
  private_nh_.param<std::string>("object_src", object_src_topic, "/detection/image_detector/objects");
  private_nh_.param<std::string>("lidar_src", lidar_src_topic, "/velodyne_points");
  private_nh_.param<std::string>("lidar_frustum_out", lidar_frustum_topic, "/lidar_frustum_points");
  private_nh_.param("use_multiple_thres", _use_multiple_thres, false);

  //get namespace from topic
  std::string ros_namespace = image_src_topic;
  std::size_t found_pos = ros_namespace.rfind("/");//find last / from topic name to extract namespace
  std::cout << ros_namespace << std::endl;
  if (found_pos!=std::string::npos)
    ros_namespace.erase(found_pos, ros_namespace.length()-found_pos);
  std::cout << ros_namespace << std::endl;


  image_filter_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(private_nh_,
                                                                                 image_src_topic,
                                                                                             1);

  lidar_filter_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(private_nh_,
                                                                                 lidar_src_topic,
                                                                                             1);

  detection_filter_subscriber_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(private_nh_,
                                                                                                     object_src_topic,
                                                                                             1);

  detections_synchronizer_lidar_=
      new message_filters::Synchronizer<SyncPolicyT_cam_lidar>(SyncPolicyT_cam_lidar(10),
                                                   *lidar_filter_subscriber_,
                                                   *detection_filter_subscriber_);

  detections_synchronizer_lidar_->registerCallback(
    boost::bind(SyncedDetectionsCallback_lidar_cam, _1, _2));

  publisher_lidar_frustum_=node_handle_.advertise<sensor_msgs::PointCloud2>(
    lidar_frustum_topic, 1);

  ros::Subscriber projection = nh.subscribe<autoware_msgs::ProjectionMatrix>(projection_matrix_topic, 1, projection_callback); 
  ros::Subscriber intrinsic = nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic_str, 1, intrinsic_callback);

  // ros::Subscriber Imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 100, imu_handler);//rt3000


//*********************************************************************************************//
//*********************************************************************************************//
  tf::StampedTransform transform;
  tf::TransformListener listener;
  tf::TransformListener vectormap_tf_listener;

  _vectormap_transform_listener = &vectormap_tf_listener;
  _transform = &transform;
  _transform_listener = &listener;

  #if (CV_MAJOR_VERSION == 3)
    generateColors(_colors, 255);
  #else
    cv::generateColors(_colors, 255);
  #endif

  _pub_cluster_cloud = nh.advertise<sensor_msgs::PointCloud2>("/points_cluster", 1);
  _centroid_pub = nh.advertise<autoware_msgs::Centroids>("/cluster_centroids", 1);

  _pub_clusters_message = nh.advertise<autoware_msgs::CloudClusterArray>("/detection/lidar_detector/cloud_clusters", 1);
  _pub_detected_objects = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);

  node_handle_.param("cluster_size_min", _cluster_size_min, 5);
  node_handle_.param("cluster_size_max", _cluster_size_max, 100000);
  node_handle_.param("pose_estimation", _pose_estimation, false);
  node_handle_.param("cluster_merge_threshold", _cluster_merge_threshold, 1.5);
  node_handle_.param<std::string>("output_frame", _output_frame, "velodyne");
  node_handle_.param("clustering_distance", _clustering_distance, 0.75);
  std::string str_distances;
  std::string str_ranges;
  node_handle_.param("clustering_distances", str_distances, std::string("[0.5,1.1,1.6,2.1,2.6]"));
  node_handle_.param("clustering_ranges", str_ranges, std::string("[15,30,45,60]"));

  YAML::Node distances = YAML::Load(str_distances);
  YAML::Node ranges = YAML::Load(str_ranges);
  size_t distances_size = distances.size();
  size_t ranges_size = ranges.size();
  for (size_t i_distance = 0; i_distance < distances_size; i_distance++)
  {
    _clustering_distances.push_back(distances[i_distance].as<double>());
    std::cout<<distances[i_distance].as<double>()<<std::endl;
  }
  for (size_t i_range = 0; i_range < ranges_size; i_range++)
  {
    _clustering_ranges.push_back(ranges[i_range].as<double>());
    std::cout<<ranges[i_range].as<double>()<<std::endl;
  }
//*********************************************************************************************//
//*********************************************************************************************//

  ros::spin();

  return 0;

}

std::vector<ClusterPtr> clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                        autoware_msgs::Centroids &in_out_centroids,
                                        double in_max_cluster_distance = 0.5)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // create 2d pc
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
  // make it flat
  for (size_t i = 0; i < cloud_2d->points.size(); i++)
  {
    cloud_2d->points[i].z = 0;
  }

  if (cloud_2d->points.size() > 0)
    tree->setInputCloud(cloud_2d);

  std::vector<pcl::PointIndices> cluster_indices;

  // perform clustering on 2d cloud
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(in_max_cluster_distance);  //
  ec.setMinClusterSize(_cluster_size_min);
  ec.setMaxClusterSize(_cluster_size_max);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_2d);
  ec.extract(cluster_indices);
  // use indices on 3d cloud

  /////////////////////////////////
  //---  3. Color clustered points
  /////////////////////////////////
  unsigned int k = 0;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::vector<ClusterPtr> clusters;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);//coord + color
  // cluster
  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    ClusterPtr cluster(new Cluster());
    cluster->SetCloud(in_cloud_ptr, it->indices, _velodyne_header, k, (int) _colors[k].val[0],
                      (int) _colors[k].val[1],
                      (int) _colors[k].val[2], "", _pose_estimation);
    clusters.push_back(cluster);

    k++;
  }
  // std::cout << "Clusters: " << k << std::endl;
  return clusters;
}

void checkClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr> &in_clusters,
                       std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices,
                       double in_merge_threshold)
{
  // std::cout << "checkClusterMerge" << std::endl;
  pcl::PointXYZ point_a = in_clusters[in_cluster_id]->GetCentroid();
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    if (i != in_cluster_id && !in_out_visited_clusters[i])
    {
      pcl::PointXYZ point_b = in_clusters[i]->GetCentroid();
      double distance = sqrt(pow(point_b.x - point_a.x, 2) + pow(point_b.y - point_a.y, 2));
      if (distance <= in_merge_threshold)
      {
        in_out_visited_clusters[i] = true;
        out_merge_indices.push_back(i);
        // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
        checkClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices, in_merge_threshold);
      }
    }
  }
}

void mergeClusters(const std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                   std::vector<size_t> in_merge_indices, const size_t &current_index,
                   std::vector<bool> &in_out_merged_clusters)
{
  // std::cout << "mergeClusters:" << in_merge_indices.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB> sum_cloud;
  pcl::PointCloud<pcl::PointXYZ> mono_cloud;
  ClusterPtr merged_cluster(new Cluster());
  for (size_t i = 0; i < in_merge_indices.size(); i++)
  {
    sum_cloud += *(in_clusters[in_merge_indices[i]]->GetCloud());
    in_out_merged_clusters[in_merge_indices[i]] = true;
  }
  std::vector<int> indices(sum_cloud.points.size(), 0);
  for (size_t i = 0; i < sum_cloud.points.size(); i++)
  {
    indices[i] = i;
  }

  if (sum_cloud.points.size() > 0)
  {
    pcl::copyPointCloud(sum_cloud, mono_cloud);
    merged_cluster->SetCloud(mono_cloud.makeShared(), indices, _velodyne_header, current_index,
                             (int) _colors[current_index].val[0], (int) _colors[current_index].val[1],
                             (int) _colors[current_index].val[2], "", _pose_estimation);
    out_clusters.push_back(merged_cluster);
  }
}

void checkAllForMerge(std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                      float in_merge_threshold)
{
  // std::cout << "checkAllForMerge" << std::endl;
  std::vector<bool> visited_clusters(in_clusters.size(), false);
  std::vector<bool> merged_clusters(in_clusters.size(), false);
  size_t current_index = 0;
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    if (!visited_clusters[i])
    {
      visited_clusters[i] = true;
      std::vector<size_t> merge_indices;
      checkClusterMerge(i, in_clusters, visited_clusters, merge_indices, in_merge_threshold);
      mergeClusters(in_clusters, out_clusters, merge_indices, current_index++, merged_clusters);
    }
  }
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    // check for clusters not merged, add them to the output
    if (!merged_clusters[i])
    {
      out_clusters.push_back(in_clusters[i]);
    }
  }

  // ClusterPtr cluster(new Cluster());
}

void segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                       autoware_msgs::Centroids &in_out_centroids, autoware_msgs::CloudClusterArray &in_out_clusters)
{
  // cluster the pointcloud according to the distance of the points using different thresholds (not only one for the
  // entire pc)
  // in this way, the points farther in the pc will also be clustered

  // 0 => 0-15m d=0.5
  // 1 => 15-30 d=1
  // 2 => 30-45 d=1.6
  // 3 => 45-60 d=2.1
  // 4 => >60   d=2.6

  std::vector<ClusterPtr> all_clusters;

  if (!_use_multiple_thres)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZ current_point;
      current_point.x = in_cloud_ptr->points[i].x;
      current_point.y = in_cloud_ptr->points[i].y;
      current_point.z = in_cloud_ptr->points[i].z;

      cloud_ptr->points.push_back(current_point);
    }

    all_clusters =
        clusterAndColor(cloud_ptr, out_cloud_ptr, in_out_centroids, _clustering_distance);

  } 

  else
  {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(2);
    for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      cloud_segments_array[i] = tmp_cloud;
    }

    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZ current_point;
      current_point.x = in_cloud_ptr->points[i].x;
      current_point.y = in_cloud_ptr->points[i].y;
      current_point.z = in_cloud_ptr->points[i].z;

      float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

      if (origin_distance < _clustering_ranges[0])
      {
        cloud_segments_array[0]->points.push_back(current_point);
      }
      else
      {
        cloud_segments_array[1]->points.push_back(current_point);
      }

      // if (origin_distance < _clustering_ranges[0])
      // {
      //   cloud_segments_array[0]->points.push_back(current_point);
      // }
      // else if (origin_distance < _clustering_ranges[1])
      // {
      //   cloud_segments_array[1]->points.push_back(current_point);

      // }

      // else if (origin_distance < _clustering_ranges[2])
      // {
      //   cloud_segments_array[2]->points.push_back(current_point);

      // }else if (origin_distance < _clustering_ranges[3])
      // {
      //   cloud_segments_array[3]->points.push_back(current_point);

      // }else
      // {
      //   cloud_segments_array[4]->points.push_back(current_point);
      // }

    }

    std::vector<ClusterPtr> local_clusters;
    for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
    {

      local_clusters = clusterAndColor(
          cloud_segments_array[i], out_cloud_ptr, in_out_centroids, _clustering_distances[i]);
      all_clusters.insert(all_clusters.end(), local_clusters.begin(), local_clusters.end());
    }
  }




  // Clusters can be merged or checked in here
  //....
  // check for mergable clusters
  std::vector<ClusterPtr> mid_clusters;
  std::vector<ClusterPtr> final_clusters;

  if (all_clusters.size() > 0)
    checkAllForMerge(all_clusters, mid_clusters, _cluster_merge_threshold);
  else
    mid_clusters = all_clusters;

  if (mid_clusters.size() > 0)
    checkAllForMerge(mid_clusters, final_clusters, _cluster_merge_threshold);
  else
    final_clusters = mid_clusters;

    // Get final PointCloud to be published
    for (unsigned int i = 0; i < final_clusters.size(); i++)
    {
      *out_cloud_ptr = *out_cloud_ptr + *(final_clusters[i]->GetCloud());

      jsk_recognition_msgs::BoundingBox bounding_box = final_clusters[i]->GetBoundingBox();
      geometry_msgs::PolygonStamped polygon = final_clusters[i]->GetPolygon();
      jsk_rviz_plugins::Pictogram pictogram_cluster;
      pictogram_cluster.header = _velodyne_header;

      // PICTO
      pictogram_cluster.mode = pictogram_cluster.STRING_MODE;
      pictogram_cluster.pose.position.x = final_clusters[i]->GetMaxPoint().x;
      pictogram_cluster.pose.position.y = final_clusters[i]->GetMaxPoint().y;
      pictogram_cluster.pose.position.z = final_clusters[i]->GetMaxPoint().z;
      tf::Quaternion quat(0.0, -0.7, 0.0, 0.7);
      tf::quaternionTFToMsg(quat, pictogram_cluster.pose.orientation);
      pictogram_cluster.size = 4;
      std_msgs::ColorRGBA color;
      color.a = 1;
      color.r = 1;
      color.g = 1;
      color.b = 1;
      pictogram_cluster.color = color;
      pictogram_cluster.character = std::to_string(i);
      // PICTO

      // pcl::PointXYZ min_point = final_clusters[i]->GetMinPoint();
      // pcl::PointXYZ max_point = final_clusters[i]->GetMaxPoint();
      pcl::PointXYZ center_point = final_clusters[i]->GetCentroid();
      geometry_msgs::Point centroid;
      centroid.x = center_point.x;
      centroid.y = center_point.y;
      centroid.z = center_point.z;
      bounding_box.header = _velodyne_header;
      polygon.header = _velodyne_header;

      if (final_clusters[i]->IsValid())
      {

        in_out_centroids.points.push_back(centroid);

        autoware_msgs::CloudCluster cloud_cluster;
        final_clusters[i]->ToROSMessage(_velodyne_header, cloud_cluster);
        in_out_clusters.clusters.push_back(cloud_cluster);
      }
    }
}

autoware_msgs::PointsImage pointcloud2_to_image(const sensor_msgs::PointCloud2ConstPtr& pointcloud2,
                                                const cv::Mat& cameraExtrinsicMat, const cv::Mat& cameraMat,
                                                const cv::Mat& distCoeff, const cv::Size& imageSize)
{
  int w = imageSize.width;
  int h = imageSize.height;

  autoware_msgs::PointsImage msg;

  msg.header = pointcloud2->header;

  msg.intensity.assign(w * h, 0);
  msg.distance.assign(w * h, 0);
  msg.min_height.assign(w * h, 0);
  msg.max_height.assign(w * h, 0);

  uintptr_t cp = (uintptr_t)pointcloud2->data.data();

  msg.max_y = -1;
  msg.min_y = h;

  msg.image_height = imageSize.height;
  msg.image_width = imageSize.width;
  if (!init_matrix)
  {
    initMatrix(cameraExtrinsicMat);
  }

  cv::Mat point(1, 3, CV_64F);
  cv::Point2d imagepoint;
  for (uint32_t y = 0; y < pointcloud2->height; ++y)
  {
    for (uint32_t x = 0; x < pointcloud2->width; ++x)
    {
      float* fp = (float*)(cp + (x + y * pointcloud2->width) * pointcloud2->point_step);
      double intensity = fp[4];
      for (int i = 0; i < 3; i++)
      {
        point.at<double>(i) = invTt.at<double>(i);
        for (int j = 0; j < 3; j++)
        {
          point.at<double>(i) += double(fp[j]) * invRt.at<double>(j, i);
        }
      }

      if (point.at<double>(2) <= 1)
      {
        continue;
      }

      double tmpx = point.at<double>(0) / point.at<double>(2);
      double tmpy = point.at<double>(1) / point.at<double>(2);
      double r2 = tmpx * tmpx + tmpy * tmpy;
      double tmpdist =
          1 + distCoeff.at<double>(0) * r2 + distCoeff.at<double>(1) * r2 * r2 + distCoeff.at<double>(4) * r2 * r2 * r2;

      imagepoint.x =
          tmpx * tmpdist + 2 * distCoeff.at<double>(2) * tmpx * tmpy + distCoeff.at<double>(3) * (r2 + 2 * tmpx * tmpx);
      imagepoint.y =
          tmpy * tmpdist + distCoeff.at<double>(2) * (r2 + 2 * tmpy * tmpy) + 2 * distCoeff.at<double>(3) * tmpx * tmpy;
      imagepoint.x = cameraMat.at<double>(0, 0) * imagepoint.x + cameraMat.at<double>(0, 2);
      imagepoint.y = cameraMat.at<double>(1, 1) * imagepoint.y + cameraMat.at<double>(1, 2);

      int px = int(imagepoint.x + 0.5);
      int py = int(imagepoint.y + 0.5);
      if (0 <= px && px < w && 0 <= py && py < h)
      {
        int pid = py * w + px;
        if (msg.distance[pid] == 0 || msg.distance[pid] > point.at<double>(2))
        {
          msg.distance[pid] = float(point.at<double>(2) * 100);
          msg.intensity[pid] = float(intensity);

          msg.max_y = py > msg.max_y ? py : msg.max_y;
          msg.min_y = py < msg.min_y ? py : msg.min_y;
        }
        if (0 == y && pointcloud2->height == 2)  // process simultaneously min and max during the first layer
        {
          float* fp2 = (float*)(cp + (x + (y + 1) * pointcloud2->width) * pointcloud2->point_step);
          msg.min_height[pid] = fp[2];
          msg.max_height[pid] = fp2[2];
        }
        else
        {
          msg.min_height[pid] = -1.25;
          msg.max_height[pid] = 0;
        }
      }
    }
  }

  return msg;
}


cv::Point2d project_lidarpoint2imagepoint(PointT Point_3D)
{
    cv::Point2d imagepoint;
    cv::Mat invR = cameraExtrinsicMat(cv::Rect(0, 0, 3, 3)).t();
    cv::Mat invT = -invR * (cameraExtrinsicMat(cv::Rect(3, 0, 1, 3)));
    cv::Mat point(1, 3, CV_64F);

    point.at<double>(0) = double(Point_3D.x);//x of velodyne point
    point.at<double>(1) = double(Point_3D.y);//y of velodyne point
    point.at<double>(2) = double(Point_3D.z);//z of velodyne point

    point = point * invR.t() + invT.t();

    if (point.at<double>(2) <= 0)
    {
      imagepoint.x=0;
      imagepoint.y=0;
      return imagepoint;
    }

    double tmpx = point.at<double>(0) / point.at<double>(2);//x/z
    double tmpy = point.at<double>(1) / point.at<double>(2);//y/z

    double r2 = tmpx * tmpx + tmpy * tmpy;
    double tmpdist =
        1 + distCoeff.at<double>(0) * r2 + distCoeff.at<double>(1) * r2 * r2 + distCoeff.at<double>(4) * r2 * r2 * r2;

    
    imagepoint.x =
        tmpx * tmpdist + 2 * distCoeff.at<double>(2) * tmpx * tmpy + distCoeff.at<double>(3) * (r2 + 2 * tmpx * tmpx);
    imagepoint.y =
        tmpy * tmpdist + distCoeff.at<double>(2) * (r2 + 2 * tmpy * tmpy) + 2 * distCoeff.at<double>(3) * tmpx * tmpy;
    imagepoint.x = cameraMat.at<double>(0, 0) * imagepoint.x + cameraMat.at<double>(0, 2);
    imagepoint.y = cameraMat.at<double>(1, 1) * imagepoint.y + cameraMat.at<double>(1, 2);

    return imagepoint;
}

void publishCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = _velodyne_header;
  in_publisher->publish(cloud_msg);
}

void publishColorCloud(const ros::Publisher *in_publisher,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = _velodyne_header;
  in_publisher->publish(cloud_msg);
}

void publishCentroids(const ros::Publisher *in_publisher, const autoware_msgs::Centroids &in_centroids,
                      const std::string &in_target_frame, const std_msgs::Header &in_header)
{
  if (in_target_frame != in_header.frame_id)
  {
    autoware_msgs::Centroids centroids_transformed;
    centroids_transformed.header = in_header;
    centroids_transformed.header.frame_id = in_target_frame;
    for (auto i = centroids_transformed.points.begin(); i != centroids_transformed.points.end(); i++)
    {
      geometry_msgs::PointStamped centroid_in, centroid_out;
      centroid_in.header = in_header;
      centroid_in.point = *i;
      try
      {
        _transform_listener->transformPoint(in_target_frame, ros::Time(), centroid_in, in_header.frame_id,
                                            centroid_out);

        centroids_transformed.points.push_back(centroid_out.point);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("publishCentroids: %s", ex.what());
      }
    }
    in_publisher->publish(centroids_transformed);
  } else
  {
    in_publisher->publish(in_centroids);
  }
}

void publishDetectedObjects(const autoware_msgs::CloudClusterArray &in_clusters)
{
  autoware_msgs::DetectedObjectArray detected_objects;
  detected_objects.header = in_clusters.header;

  for (size_t i = 0; i < in_clusters.clusters.size(); i++)
  {
    autoware_msgs::DetectedObject detected_object;
    detected_object.header = in_clusters.header;
    detected_object.label = "unknown";
    detected_object.score = 1.;
    detected_object.space_frame = in_clusters.header.frame_id;
    detected_object.pose = in_clusters.clusters[i].bounding_box.pose;
    detected_object.dimensions = in_clusters.clusters[i].dimensions;
    detected_object.pointcloud = in_clusters.clusters[i].cloud;
    detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
    detected_object.valid = true;

    detected_objects.objects.push_back(detected_object);
  }
  _pub_detected_objects.publish(detected_objects);
}

void publishCloudClusters(const ros::Publisher *in_publisher, const autoware_msgs::CloudClusterArray &in_clusters,
                          const std::string &in_target_frame, const std_msgs::Header &in_header)
{
  if (in_target_frame != in_header.frame_id)
  {
    autoware_msgs::CloudClusterArray clusters_transformed;
    clusters_transformed.header = in_header;
    clusters_transformed.header.frame_id = in_target_frame;
    for (auto i = in_clusters.clusters.begin(); i != in_clusters.clusters.end(); i++)
    {
      autoware_msgs::CloudCluster cluster_transformed;
      cluster_transformed.header = in_header;
      try
      {
        _transform_listener->lookupTransform(in_target_frame, _velodyne_header.frame_id, ros::Time(),
                                             *_transform);
        pcl_ros::transformPointCloud(in_target_frame, *_transform, i->cloud, cluster_transformed.cloud);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->min_point, in_header.frame_id,
                                            cluster_transformed.min_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->max_point, in_header.frame_id,
                                            cluster_transformed.max_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->avg_point, in_header.frame_id,
                                            cluster_transformed.avg_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->centroid_point, in_header.frame_id,
                                            cluster_transformed.centroid_point);

        cluster_transformed.dimensions = i->dimensions;
        cluster_transformed.eigen_values = i->eigen_values;
        cluster_transformed.eigen_vectors = i->eigen_vectors;

        cluster_transformed.convex_hull = i->convex_hull;
        cluster_transformed.bounding_box.pose.position = i->bounding_box.pose.position;
        if(_pose_estimation)
        {
          cluster_transformed.bounding_box.pose.orientation = i->bounding_box.pose.orientation;
        }
        else
        {
          cluster_transformed.bounding_box.pose.orientation.w = _initial_quat_w;
        }
        clusters_transformed.clusters.push_back(cluster_transformed);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("publishCloudClusters: %s", ex.what());
      }
    }
    in_publisher->publish(clusters_transformed);
    publishDetectedObjects(clusters_transformed);
  } else
  {
    in_publisher->publish(in_clusters);
    publishDetectedObjects(in_clusters);
  }
}

pcl::PointCloud<PointT> to_Frustum_points_3D(pcl::PointCloud<PointT> Points_3D,autoware_msgs::DetectedObjectArray in_objects)
{

  visualization_msgs::MarkerArray  bounding_boxes;

  visualization_msgs::Marker box;

  box.lifetime = ros::Duration(1);
  box.header = in_objects.header;
  box.header.frame_id="velodyne";
  
  box.type = visualization_msgs::Marker::CUBE;
  box.action = visualization_msgs::Marker::ADD;
  box.ns =  "/box_markers";
  box.id = 0;

  // box.scale = object.dimensions;

  box.scale.x = 5;
  box.scale.y = 1.8;
  box.scale.z = 1.5;

  box.pose.position.x =0 ;
  box.pose.position.y =0 ;
  box.pose.position.z =0 ;


  box.color.r = 0.0;
  box.color.g = 1.0;
  box.color.b = 0.0;
  box.color.a = 1.0;

  bounding_boxes.markers.push_back(box);

  int w = imageSize.width;
  int h = imageSize.height;

  if (!init_matrix)
  {
    initMatrix(cameraExtrinsicMat);
  }

  pcl::PointCloud<PointT> Frustum_points_3D;

  std::vector<pcl::PointCloud<PointT>> points2cluster;

  PointT Point_3D;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_total_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  autoware_msgs::CloudClusterArray cloud_clusters_total;


  for (auto const &object: in_objects.objects)
  {
      pcl::PointCloud<PointT> points_inter;
      pcl::PointXYZ point_xyz;
      if (IsObjectValid(object)  && object.label=="car" )
      {
        cv::Rect rect;
        rect.x = object.x;
        rect.y = object.y;
        rect.width = object.width;
        rect.height = object.height;
        pcl::PointCloud<pcl::PointXYZ>::Ptr bb_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        autoware_msgs::Centroids centroids;
        autoware_msgs::CloudClusterArray cloud_clusters;

        for (int i = 0; i < Points_3D.size(); ++i)
        {
          cv::Point2d imagepoint;
          Point_3D=Points_3D[i];
          point_xyz.x=Point_3D.x;
          point_xyz.y=Point_3D.y;
          point_xyz.z=Point_3D.z;

          imagepoint=project_lidarpoint2imagepoint(Point_3D);
          
          double r1=imagepoint.x*imagepoint.x+imagepoint.y*imagepoint.y;

          int px = int(imagepoint.x + 0.5);
          int py = int(imagepoint.y + 0.5);

          // if (imagepoint.x >= 0 && imagepoint.x < imageSize.width && imagepoint.y >= 0 && imagepoint.y < imageSize.height && r1>0.1 )
          // {

          // }

          if (px>=rect.tl().x  && px<=rect.br().x &&  py>=rect.tl().y  && py<=rect.br().y )
          {
            Frustum_points_3D.push_back(Point_3D);
            points_inter.push_back(Point_3D);

            bb_cloud_ptr->points.push_back(point_xyz);
          }   

        }

        if (!bb_cloud_ptr->empty())
        {
          segmentByDistance(bb_cloud_ptr, colored_clustered_cloud_ptr, centroids,
                  cloud_clusters);

          if (!colored_clustered_cloud_ptr->empty())
          {
            for (int i = 0; i < colored_clustered_cloud_ptr->points.size(); ++i)
            {
              colored_clustered_cloud_total_ptr->points.push_back(colored_clustered_cloud_ptr->points[i]);
            }

            for (auto i = cloud_clusters.clusters.begin(); i != cloud_clusters.clusters.end(); i++)
            {
              cloud_clusters_total.clusters.push_back(*i);
            }

          }

        }
      }

      if (!points_inter.empty())
      {
        points2cluster.push_back(points_inter);
      }
  }

  if (!colored_clustered_cloud_total_ptr->empty())
  {
    publishColorCloud(&_pub_cluster_cloud, colored_clustered_cloud_total_ptr);
    // centroids.header = _velodyne_header;
    // publishCentroids(&_centroid_pub, centroids, _output_frame, _velodyne_header);
    cloud_clusters_total.header = _velodyne_header;
    publishCloudClusters(&_pub_clusters_message, cloud_clusters_total, _output_frame, _velodyne_header);
  }

  if (!points2cluster.empty())
  {
    /* code */
  }  

  return Frustum_points_3D;
}

void resetMatrix()
{
  init_matrix = false;
}

void initMatrix(const cv::Mat& cameraExtrinsicMat)
{
  invRt = cameraExtrinsicMat(cv::Rect(0, 0, 3, 3));
  cv::Mat invT = -invRt.t() * (cameraExtrinsicMat(cv::Rect(3, 0, 1, 3)));
  invTt = invT.t();
  init_matrix = true;
}

void projection_callback(autoware_msgs::ProjectionMatrix msg)
{
  cameraExtrinsicMat = cv::Mat(4, 4, CV_64F);
  for (int row = 0; row < 4; row++)
  {
    for (int col = 0; col < 4; col++)
    {
      cameraExtrinsicMat.at<double>(row, col) = msg.projection_matrix[row * 4 + col];
    }
  }
  resetMatrix();
}


void intrinsic_callback(sensor_msgs::CameraInfo msg)
{
  imageSize.height = msg.height;
  imageSize.width = msg.width;

  cameraMat = cv::Mat(3, 3, CV_64F);
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      cameraMat.at<double>(row, col) = msg.K[row * 3 + col];
    }
  }

  distCoeff = cv::Mat(1, 5, CV_64F);
  for (int col = 0; col < 5; col++)
  {
    distCoeff.at<double>(col) = msg.D[col];
  }
  resetMatrix();
}


void
SyncedDetectionsCallback_lidar_cam(
  const sensor_msgs::PointCloud2::ConstPtr &in_lidar_msg,
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects)
{
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
  pcl::fromROSMsg(*in_lidar_msg, *cloud);

  _velodyne_header = in_lidar_msg->header;

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  if (cameraExtrinsicMat.empty() || cameraMat.empty() || distCoeff.empty() || imageSize.height == 0 ||
      imageSize.width == 0)
  {
    ROS_INFO("[points2image]Looks like camera_info or projection_matrix are not being published.. Please check that "
             "both are running..");
    return;
  }

  pcl::PointCloud<PointT> points_inBB=to_Frustum_points_3D(*cloud,*in_objects);

  if (!points_inBB.empty())
  {
    /*publishing the points in frustum*/
    sensor_msgs::PointCloud2 points_inBB_;
    pcl::toROSMsg(points_inBB, points_inBB_);
    points_inBB_.header.stamp = in_lidar_msg->header.stamp;
    points_inBB_.header.frame_id = in_lidar_msg->header.frame_id;
    publisher_lidar_frustum_.publish(points_inBB_);
    /**/
  }


}


void
SyncedDetectionsCallback(
  const sensor_msgs::Image::ConstPtr &in_image_msg,
  const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects)
{
  try
  {
    image_ = cv_bridge::toCvShare(in_image_msg, "bgr8")->image;
    cv::Mat drawn_image;
    drawn_image = ObjectsToRects(image_, in_objects);
    sensor_msgs::ImagePtr drawn_msg = cv_bridge::CvImage(in_image_msg->header, "bgr8", drawn_image).toImageMsg();
    publisher_image_.publish(drawn_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("[%s] Could not convert from '%s' to 'bgr8'.", __APP_NAME__, in_image_msg->encoding.c_str());
  }
}

cv::Mat
ObjectsToRects(cv::Mat in_image, const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects)
{
  cv::Mat final_image = in_image.clone();
  for (auto const &object: in_objects->objects)
  {
    if (IsObjectValid(object))
    {
      cv::Rect rect;
      rect.x = object.x;
      rect.y = object.y;
      rect.width = object.width;
      rect.height = object.height;

      if (rect.x+rect.width >= in_image.cols)
        rect.width = in_image.cols -rect.x - 1;

      if (rect.y+rect.height >= in_image.rows)
        rect.height = in_image.rows -rect.y - 1;

      //draw rectangle
      cv::rectangle(final_image,
                    rect,
                    cv::Scalar(244,134,66),
                    4,
                    CV_AA);

      //draw label
      std::string label = "";
      if (!object.label.empty() && object.label != "unknown")
      {
        label = object.label;
      }
      int font_face = cv::FONT_HERSHEY_DUPLEX;
      double font_scale = 1.5;
      int thickness = 1;

      int baseline=0;
      cv::Size text_size = cv::getTextSize(label,
                                          font_face,
                                          font_scale,
                                          thickness,
                                          &baseline);
      baseline += thickness;

      cv::Point text_origin(object.x - text_size.height,object.y);

      cv::rectangle(final_image,
                    text_origin + cv::Point(0, baseline),
                    text_origin + cv::Point(text_size.width, -text_size.height),
                    cv::Scalar(0,0,0),
                    CV_FILLED,
                    CV_AA,
                    0);

      cv::putText(final_image,
                  label,
                  text_origin,
                  font_face,
                  font_scale,
                  cv::Scalar::all(255),
                  thickness,
                  CV_AA,
                  false);

    }
  }
  return final_image;
}//ObjectsToBoxes

bool IsObjectValid(const autoware_msgs::DetectedObject &in_object)
{
  if (!in_object.valid ||
      in_object.width < 0 ||
      in_object.height < 0 ||
      in_object.x < 0 ||
      in_object.y < 0
    )
  {
    return false;
  }
  return true;
}//end IsObjectValid