#include "include/CustomMsg.h"
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>

using namespace std;

string file_path;

string bag_file;
string lidar_topic;
string pcd_file;

bool is_custom_msg;
double limit_y;
int bag_num = 1;

void save_points_from_bag( string bag_file, const string pcd_file)
{
  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  std::fstream file_;
  file_.open(bag_file, ios::in);
  if (!file_) {
    std::string msg = "Loading the rosbag " + bag_file + " failue";
    ROS_ERROR_STREAM(msg.c_str());
    return ;
  }
  ROS_INFO("Loading the rosbag %s", bag_file.c_str());
  rosbag::Bag bag;
  try {
    bag.open(bag_file, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return ;
  }
  std::vector<string> lidar_topic_vec;
  // lidar_topic_vec.push_back(lidar_topic);
  lidar_topic_vec.push_back("/hk_camera/image_color");
  lidar_topic_vec.push_back("/hk_camera_info");
  lidar_topic_vec.push_back("/livox/lidar");
  
  std::string output_bag_file =  bag_file.insert(bag_file.size() - 4, "_s");
  ROS_INFO("new bag is: %s", output_bag_file.c_str());

  rosbag::Bag output_bag;
  output_bag.open(output_bag_file, rosbag::bagmode::Write);

  rosbag::View view(bag, rosbag::TopicQuery(lidar_topic_vec));

  int cnts = 0;
  for (const rosbag::MessageInstance &m : view)
  {
    // ROS_INFO("msg is: %s", m.getTopic().c_str());

    // 保存前面几帧相机内参
    if ( m.getTopic() == "/hk_camera_info" && cnts++ < 10)
    {
      output_bag.write(m.getTopic(), m.getTime(), m);
      continue;
    }

    // 保存前面几帧图像
    if (m.getTopic() == "/hk_camera/image_color" && cnts++ < 10)
    {
      output_bag.write(m.getTopic(), m.getTime(), m);
      sensor_msgs::Image::ConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
      std::string image_file_name = output_bag_file.substr(0, output_bag_file.length() - 6) + ".png";
      ROS_INFO_ONCE("img file is: %s", image_file_name.c_str());

      cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image_msg, "bgr8");
      cv::imwrite(image_file_name, cv_image->image);
      continue;
    }

    if (m.getTopic() == "/livox/lidar")
    {
      pcl::PointCloud<pcl::PointXYZI> one_cloud;

      livox_ros_driver::CustomMsg livox_cloud_msg =
          *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type

      for (uint i = 0; i < livox_cloud_msg.point_num; ++i)
      {
        pcl::PointXYZI p;
        p.x = livox_cloud_msg.points[i].x;
        p.y = livox_cloud_msg.points[i].y;
        p.z = livox_cloud_msg.points[i].z;
        p.intensity = livox_cloud_msg.points[i].reflectivity;

        // if (p.intensity > 120.0)
        //   continue;

        if (p.x > 8.0 and p.x < 150.0 and std::fabs(p.y) < limit_y)
        {
          one_cloud.points.emplace_back(p);
          // one_cloud.points[pc_cnts++] = p;
        }
      }

      one_cloud.is_dense = false;
      one_cloud.width = one_cloud.points.size();
      one_cloud.height = 1;
      // ROS_INFO_STREAM(one_cloud.points.size());

      sensor_msgs::PointCloud2 pointcloud_msg;
      pcl::toROSMsg(one_cloud, pointcloud_msg);
      pointcloud_msg.header.frame_id = "map";
      pointcloud_msg.header.stamp = m.getTime();
      output_bag.write("/livox_pc2", m.getTime(), pointcloud_msg);

      //  --------------------------------------------------------------------
      if (is_custom_msg)
      {
        livox_ros_driver::CustomMsg livox_cloud_msg =
            *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type
        for (uint i = 0; i < livox_cloud_msg.point_num; ++i)
        {
          pcl::PointXYZI p;
          p.x = livox_cloud_msg.points[i].x;
          p.y = livox_cloud_msg.points[i].y;
          p.z = livox_cloud_msg.points[i].z;
          p.intensity = livox_cloud_msg.points[i].reflectivity;

          // if (p.intensity > 120.0)
          // {
          //   continue;
          // }

          // if( p.x > 5.0  and p.x < 150.0 and  std::fabs( p.y ) < limit_y )
          if (p.x > 8.0 and p.x < 150.0 and std::fabs(p.y) < limit_y)
          {
            output_cloud.points.push_back(p);
          }
        }
      }
      else
      {
        sensor_msgs::PointCloud2 livox_cloud;
        livox_cloud = *(m.instantiate<sensor_msgs::PointCloud2>()); // message
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(livox_cloud, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, cloud);
        for (uint i = 0; i < cloud.size(); ++i)
        {
          if (cloud.points[i].intensity > 120.0)
          {
            continue;
          }

          if (cloud.points[i].x > 1.0 and cloud.points[i].x < 15.0 and std::fabs(cloud.points[i].y) < limit_y)
          {
            output_cloud.points.push_back(cloud.points[i]);
          }
        }
      }
      //  --------------------------------------------------------------------
    }
  }

  bag.close();
  output_bag.close();

  output_cloud.is_dense = false;
  output_cloud.width = output_cloud.points.size();
  output_cloud.height = 1;
  pcl::io::savePCDFileASCII(pcd_file, output_cloud);
  string msg = "Sucessfully save point cloud to pcd file: " + pcd_file;
  ROS_INFO_STREAM(msg.c_str());

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidarCamCalib");
  ros::NodeHandle nh;
  nh.param<string>("file_path", file_path, "");
  // nh.param<string>("pcd_file", pcd_file, "");
  nh.param<string>("lidar_topic", lidar_topic, "/livox/lidar");
  nh.param<bool>("is_custom_msg", is_custom_msg, false);
  nh.param<double>("limit_y", limit_y, 10.0);
  nh.param<int>("bag_num", bag_num, 5 );

  for (int i = 0; i < bag_num ; i++)
  {
    string bag_file = file_path + std::to_string(i) + ".bag";
    string pcd_file = file_path + std::to_string(i) + ".pcd";
    string msg = "bag file: " + bag_file;
    ROS_INFO_STREAM(msg.c_str());
    msg = "pcd file: " + pcd_file;
    ROS_INFO_STREAM(msg.c_str());

    save_points_from_bag(bag_file, pcd_file);
  }
  
  return 0;

}