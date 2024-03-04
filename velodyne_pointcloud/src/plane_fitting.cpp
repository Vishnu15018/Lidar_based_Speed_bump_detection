#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl/registration/icp.h>
#include <math.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16MultiArray.h>
#include <octomap/octomap.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/PointIndices.h>
#include <ctime>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include<ctime>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <velodyne_pointcloud/organized_cloudXYZIR.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <velodyne_pointcloud/point_types.h>


int count=1;
ros::Publisher pub_;
ros::Publisher pub_plane;
//static double distance;


// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;
std::ofstream logfile;
// const int size_array = 11;
float thr1 =0.036;
float thr2=0.080;

double severity_thresholds[] = {1.0, 2.0, 3.0, 4.0, 5.0};
double no_frame=0,gremoval_time=0;
  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  { std::clock_t start = std::clock();
    // Stop the clock
    logfile <<"Threshold thr1: "<<thr1<<"Threshold2 thr2"<<thr2<<std::endl;
    std::cout<<"inside callback"<<endl;
    ros::Time timestamp = msg->header.stamp;

    // Print the timestamp
    ROS_INFO("Received PointCloud2 message with timestamp: %f", timestamp.toSec());
    logfile <<"timestamp : "<<timestamp.toSec()<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

     // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // Define strip size
    float strip_size = 0.25; // Example value, you can adjust this as needed
    
    // Calculate number of rows and columns
    int y_strip=2;
    int x_strip=10;
    int rows = ceil(x_strip / strip_size);
    int cols = ceil((y_strip * 2) / strip_size);
    logfile<<"x_strip: "<<x_strip<<"y_strip"<<y_strip<<std::endl;
    logfile<<"Strip_size: "<<strip_size<<std::endl;

    // Plane segmenation 

    // Filter the PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& point : cloud->points) {
        if (point.x >= 0 && point.x < x_strip &&
            point.y > -y_strip && point.y < y_strip &&
            point.z < -1.1) {
            filtered_cloud->points.push_back(point);
        }
    }

    // Fit a Plane using RANSAC
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01); // Adjust as needed
    seg.setInputCloud(filtered_cloud);
    seg.segment(*inliers, *coefficients);

    // Publish the Plane on a Topic
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(*coefficients, ros_coefficients);
    pub_plane.publish(ros_coefficients);
// for publishing the filtered data
no_frame++;
// Publish the filtered point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(filtered_cloud);
    extract.setIndices(inliers);
    extract.filter(*inlier_cloud);


  //  finding the distance from the plane and classifying it

  Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    float plane_d = coefficients->values[3];
    std::vector<float> distances;
    for (const auto& point : filtered_cloud->points) {
        Eigen::Vector3f point_vector(point.x, point.y, point.z);
        float distance = fabs(plane_normal.dot(point_vector) + plane_d) / plane_normal.norm();
        distances.push_back(distance);
    }


    // Classify points based on distance to the plane
    std::vector<int> count_label(6, 0);;

    pcl::PointCloud<pcl::PointXYZI>::Ptr classified_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    classified_cloud->points.resize(filtered_cloud->points.size());
    for (size_t i = 0; i < distances.size(); ++i) {
        float distance = distances[i];
        int label = 0;
        std::cout<<"distance:"<<distance<<std::endl;
        for (int j = 0; j < sizeof(severity_thresholds) / sizeof(severity_thresholds[0]); ++j) {
            if (distance*100 >= severity_thresholds[j]) {
                label = j + 1; // Labels start from 1
            }
        }

            count_label[label]=count_label[label] +1;
        classified_cloud->points[i].x = filtered_cloud->points[i].x;
        classified_cloud->points[i].y = filtered_cloud->points[i].y;
        classified_cloud->points[i].z = filtered_cloud->points[i].z;
        classified_cloud->points[i].intensity = static_cast<float>(label);
    }
    for(int i=0 ; i< count_label.size();i++){
      std::cout<<"label : "<<i<< " count is: "<<count_label[i]<<std::endl;
    }
    // Publish the classified point cloud
    sensor_msgs::PointCloud2 classified_msg;
    pcl::toROSMsg(*classified_cloud, classified_msg);
    classified_msg.header = msg->header;
    pub_.publish(classified_msg);



// sensor_msgs::PointCloud2 output_msg;
// pcl::toROSMsg(*inlier_cloud, output_msg);
// output_msg.header = msg->header;
// pub_.publish(output_msg);



std::cout << "Plane coefficients: ";
    for (size_t i = 0; i < coefficients->values.size(); ++i) {
        std::cout << coefficients->values[i] << " ";
    }
    std::cout << std::endl;


    std::clock_t end = std::clock();

    // Compute the elapsed time
    double elapsed_time = static_cast<double>(end - start) / CLOCKS_PER_SEC;


    // clearing the array 


    
    // Output the elapsed time
    logfile<<"Execution time: " << elapsed_time << " seconds" << std::endl;
    std::cout << "Execution time: " << elapsed_time << " seconds" << std::endl;

  }

  
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "path_hole_detection");
  ros::NodeHandle nh_ground_removal_new;
  double distance;
  nh_ground_removal_new.getParam("/ground_removal_new/distance_threshold",distance);
  // std::cout<<"Value of distance "<<distance<<"\n";
  //n.getParam("/main/x1",x1);
  std::cout<<"Hai"<<endl;
  ros::Subscriber sub_ = nh_ground_removal_new.subscribe("/velodyne_points", 1, callback);
  int x=1;
  std::string m1 ="Row_wise";
  std::string filename = "/home/lidar/Desktop/vishnu/output_txts/log_file_path_hole_" +m1 + std::to_string(thr1) + "_" + std::to_string(thr2)+ "_"+std::to_string(x) + ".txt";
  logfile.open(filename);
    if (!logfile.is_open()) {
        ROS_ERROR("Failed to open log file");
        return 1;
    }
  pub_ = nh_ground_removal_new.advertise<sensor_msgs::PointCloud2> ("/test_topic", 1);
  //pub_ = nh_ground_removal_new.advertise<PointCloud>("/published_topic", 1);
  pub_plane = nh_ground_removal_new.advertise<pcl_msgs::ModelCoefficients>("plane_model", 1);

  ros::spin();
   logfile.close();
  return 0;
}



// so co author we have plane coefficients right , find the normal distance from each point to the plane , and classify plane as if normal distance is less than 1 then label 0 , elsif distance 1 to2 label it as 1, else if distance is (2-3 ) as 2 , else if (3-4 ) 3 and (4-5) as 4, grater than 5 label it as 5. and color the point cloud according to this labelling techninique, and publkish this cloud .. regarding the data I am using is pathhole detection data, these labels will signifiy the pathhole severity?