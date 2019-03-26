/*
   * PCL Example using ROS and CPP
   */
  #include "lidar_receive/PangoCloud.h"
  #include <future>
  #include <thread>
  #include <boost/thread/thread.hpp>
  #include <sys/time.h>

  // Include the ROS library
  #include <ros/ros.h>
  #include <sensor_msgs/PointCloud2.h>

  // Include pcl
  #include <pcl_conversions/pcl_conversions.h>
  #include <pcl/visualization/cloud_viewer.h>
  #include <pcl/common/common_headers.h>
  #include <pcl/point_cloud.h>
  #include <pcl/point_types.h>
  #include <pcl_ros/transforms.h>
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Vector3f trans_vec_A{0,0,20};
  Eigen::Translation<float,3> translation_A(trans_vec_A);

// auto nothing_for = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX());

// Eigen::Quaternionf test = static_cast<Eigen::Quaternionf> (nothing_for);

  Eigen::Quaternionf rotation_B = static_cast<Eigen::Quaternionf> (Eigen::AngleAxisf(30, Eigen::Vector3f::UnitX())) ;
  Eigen::Quaternionf rotation_C = static_cast<Eigen::Quaternionf> (Eigen::AngleAxisf(40, Eigen::Vector3f::UnitY())) ;
  Eigen::Quaternionf rotation_D= static_cast<Eigen::Quaternionf> (Eigen::AngleAxisf(50, Eigen::Vector3f::UnitZ()));
  Eigen::Vector3f trans_vec_E{0,0,0};
  Eigen::Translation<float,3> translation_E(trans_vec_E);
  Eigen::Transform<float,3,Eigen::Affine> combined = 
      translation_A * rotation_B * rotation_C * rotation_D * translation_E;
struct timeval timestart;

void pcl_init()
{
  ROS_INFO("Enter pcl_init");
  pcl::visualization::CloudViewer viewer("Simple lidar view");
  
  while(!viewer.wasStopped())
  {
    viewer.showCloud(temp_cloud);
    boost::this_thread::sleep (boost::posix_time::microseconds (1000));
  }
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    if(temp_cloud->empty())
    {
      ROS_INFO("converted cloud is empty");
    }
    gettimeofday(&timestart, NULL);
    // double timeofsec = static_cast<double>(timestart.tv_sec + static_cast<double>(timestart.tv_usec)/1000000);
    auto timeofsec = static_cast<int>(timestart.tv_sec);
    // ROS_INFO("time is %d", timeofsec);
    if(timeofsec % 10 == 0)
    {
        pcl::transformPointCloud (*temp_cloud, *temp_cloud, combined);
    }
  }

  int main (int argc, char** argv)
  {
    // Initialize the ROS Node "roscpp_pcl_example"
    ros::init (argc, argv, "roscpp_pcl_example");
    ros::NodeHandle nh;
    std::thread pcl_thd(pcl_init);
    ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());
    
    ros::Subscriber sub = nh.subscribe("/pandar_points", 1, cloud_cb);
    ros::spin();
    pcl_thd.join();
    return 0;
    
  }
