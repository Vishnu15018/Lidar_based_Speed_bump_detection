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

#include <velodyne_pointcloud/organized_cloudXYZIR.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <velodyne_pointcloud/point_types.h>


int count=1;
ros::Publisher pub_;
//static double distance;


// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;
std::ofstream logfile;
const int size_array = 11;
std::array<std::vector<float>, size_array> z_values;
double no_frame=0,gremoval_time=0;
  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

  pcl::PointCloud<pcl::PointXYZ> cloud1;

    pcl::fromROSMsg (*msg, cloud1);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
*cloud=cloud1;
  ros::Time::init();
  ros::Time begin= ros::Time::now();
   std::cout<<begin<<std::endl;

   pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
// int count_groud=0;

 std::cout << "Received point cloud with " << cloud->size() << " points." << std::endl;
// ////////////////////////////////////////////////////////
float xmax,xmin,ymax,ymin;
for (size_t i = 0; i < cloud->points.size (); ++i)
   {
	
	if (i==0){
		xmax=cloud->points[i].x;
		xmin=cloud->points[i].x;
		ymax=cloud->points[i].y;
		ymin=cloud->points[i].y;
		}
	else {if (xmax<=cloud->points[i].x)
		xmax=cloud->points[i].x;
	     if (xmin>=cloud->points[i].x)
		xmin=cloud->points[i].x;
      if (ymax<=cloud->points[i].y)
		ymax=cloud->points[i].y;
	     if (ymin>=cloud->points[i].y)
		ymin=cloud->points[i].y;}
      }

std::cout<<"range of x valoues "<<xmin<<" to" <<xmax<<std::endl;
std::cout<<"range of y valoues"<<ymin<<" to "<<ymax<<std::endl;
//   pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud_filtered ;
// for (int i=0 ;i< cloud->points.size (); ++i){
//     if ((cloud->points[i].y >=-2 && cloud->points[i].y <=2)) {
//         if (cloud->points[i].x >=0 && cloud->points[i].x<=2){
//           z_values[0].push_back(cloud->points[i].z);
//         }
//         else if (cloud->points[i].x >=3 && cloud->points[i].x<=5) {
//             z_values[1].push_back(cloud->points[i].z);
//         } else if (cloud->points[i].x >=6&& cloud->points[i].x<=8) {
//             z_values[2].push_back(cloud->points[i].z);
//         } else if (cloud->points[i].x >=9 && cloud->points[i].x<=11) {
//             z_values[3].push_back(cloud->points[i].z);
//         } else {
//             continue; // Skip points that do not fall within any range
//         }
//         // std::cout<<cloud->points[i].x<<"y"<<cloud->points[i].y<<"z"<<cloud->points[i].z<<endl;
        
//     }
// }

//  for (int i=0 ; i < cloud->points.size ();++i){
//     if (cloud->points[i].x>0 && (cloud->points[i].y >=-1 && cloud->points[i].y <=1)){
//             int index= std::abs(static_cast<int>(cloud->points[i].x));
//             // std::cout<<"x: "<<cloud->points[i].x<<endl;
//             // std::cout<<index<<endl;
//            if (index>=0 and index<size_array){
//                 z_values[index].push_back(cloud->points[i].z);
//         cloud_filtered.push_back(cloud->points[i]);
          
//     }
//     }

//  }
// no_frame++;
// //   std::cout<<"average ground removal time: "<< (gremoval_time/no_frame)<<std::endl;
// std::cout << "Total no of frames: "<<no_frame<<std::endl;
// ros::Time timestamp = msg->header.stamp;
// logfile << " frame: " << no_frame << std::endl;
// for (int i = 0; i < size_array; ++i) {
//         float sum = 0.0;
//         for (const auto& z : z_values[i]) {
//             sum +=z;
//         }
//         float average = z_values[i].empty() ? 0.0 : sum / z_values[i].size();
//         ROS_INFO_STREAM("Average z value for points with x in range " << (i) << " to " << (i +1) << ": " << average);
//         logfile << timestamp << "Average z value for points with x in range " << i << " to " << i+1 << ": " << average << std::endl;
        
//     }
   for (auto& point_old : cloud->points)
    {   pcl::PointXYZRGB point;
        // Check if x-coordinate is between 4 and 6
        if (point_old.x>0 && (point_old.y >=-1 && point_old.y <=1))
        {
        if (point_old.x >= 4 && point_old.x <= 5)
        {
            // Set RGB values to highlight the area (e.g., green)
            point.x=point_old.x;
            point.y=point_old.y;
            point.z=point_old.z;
            point.r = 0;
            point.g = 255;
            point.b = 0;
        }
        else if (point_old.x >5 && point_old.x <= 6)
        {
            // Set RGB values to highlight the area (e.g., green)
            point.x=point_old.x;
            point.y=point_old.y;
            point.z=point_old.z;
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }
        else if (point_old.x >6 && point_old.x <= 7)
        {
            // Set RGB values to highlight the area (e.g., green)
            point.x=point_old.x;
            point.y=point_old.y;
            point.z=point_old.z;
            point.r = 0;
            point.g = 0;
            point.b = 255;
        } 
        else if (point_old.x >7 && point_old.x <= 8)
        {
            // Set RGB values to highlight the area (e.g., green)
            point.x=point_old.x;
            point.y=point_old.y;
            point.z=point_old.z;
            point.r = 0;
            point.g = 255;
            point.b = 255;
        } 
        else if (point_old.x >8 && point_old.x <= 9)
        {
            // Set RGB values to highlight the area (e.g., green)
            point.x=point_old.x;
            point.y=point_old.y;
            point.z=point_old.z;
            point.r = 255;
            point.g = 0;
            point.b = 255;
        } 
        else if (point_old.x >9 && point_old.x <= 10)
        {
            // Set RGB values to highlight the area (e.g., green)
            point.x=point_old.x;
            point.y=point_old.y;
            point.z=point_old.z;
            point.r = 0;
            point.g = 120;
            point.b = 255;
        } 
        else if (point_old.x >10 && point_old.x <11)
        {
            // Set RGB values to highlight the area (e.g., green)
            point.x=point_old.x;
            point.y=point_old.y;
            point.z=point_old.z;
            point.r = 0;
            point.g = 90;
            point.b = 255;
        } 
        //   else{
        //     point.x=point_old.x;
        //     point.y=point_old.y;
        //     point.z=point_old.z;
        //     point.r = 255;
        //     point.g = 255;
        //     point.b = 255;

        // }
        }
      
        else{
            point.x=point_old.x;
            point.y=point_old.y;
            point.z=point_old.z;
            point.r = 255;
            point.g = 255;
            point.b = 255;

        }
            // Set RGB values to default color (e.g., white)

        cloud_filtered.push_back(point);
    }
  


sensor_msgs::PointCloud2 output_msg;
pcl::toROSMsg(cloud_filtered, output_msg);
output_msg.header = msg->header;
std::cout<<"Header"<<endl;
std::cout<<msg->header<<endl;

    // Publish the filtered point cloud
    pub_.publish(output_msg);

 
}
  
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "ground_removal_ring");
  ros::NodeHandle nh_ground_removal_new;
  double distance;
  nh_ground_removal_new.getParam("/ground_removal_new/distance_threshold",distance);
  std::cout<<"Value of distance "<<distance<<"\n";
  //n.getParam("/main/x1",x1);
  ros::Subscriber sub_ = nh_ground_removal_new.subscribe("/velodyne_points", 1, callback);
  
  logfile.open("/home/lidar/Desktop/vishnu/log_file_test2.txt");
    if (!logfile.is_open()) {
        ROS_ERROR("Failed to open log file");
        return 1;
    }
  pub_ = nh_ground_removal_new.advertise<sensor_msgs::PointCloud2> ("/test_topic2", 1);
  //pub_ = nh_ground_removal_new.advertise<PointCloud>("/published_topic", 1);

  ros::spin();
   logfile.close();
  return 0;
}



