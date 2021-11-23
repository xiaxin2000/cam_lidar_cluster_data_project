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

  ros::spin();

  return 0;

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


pcl::PointCloud<PointT> to_Frustum_points_3D(pcl::PointCloud<PointT> Points_3D,autoware_msgs::DetectedObjectArray in_objects)
{

  int w = imageSize.width;
  int h = imageSize.height;

  if (!init_matrix)
  {
    initMatrix(cameraExtrinsicMat);
  }

  pcl::PointCloud<PointT> Frustum_points_3D;

  for (int i = 0; i < Points_3D.size(); ++i)
  {
    cv::Point2d imagepoint;
    PointT Point_3D=Points_3D[i];

    imagepoint=project_lidarpoint2imagepoint(Point_3D);
    
    double r1=imagepoint.x*imagepoint.x+imagepoint.y*imagepoint.y;

    int px = int(imagepoint.x + 0.5);
    int py = int(imagepoint.y + 0.5);

    if (imagepoint.x >= 0 && imagepoint.x < imageSize.width && imagepoint.y >= 0 && imagepoint.y < imageSize.height && r1>0.1 )
    {

      for (auto const &object: in_objects.objects)
      {
          if (IsObjectValid(object)  && object.label=="car" )
          {
            cv::Rect rect;
            rect.x = object.x;
            rect.y = object.y;
            rect.width = object.width;
            rect.height = object.height;

            if (px>=rect.tl().x  && px<=rect.br().x &&  py>=rect.tl().y  && py<=rect.br().y )
            {
              Frustum_points_3D.push_back(Point_3D);
            }

          }
      }

    }   

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