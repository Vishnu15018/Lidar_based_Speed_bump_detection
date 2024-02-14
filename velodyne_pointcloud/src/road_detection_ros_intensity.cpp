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
ros::Publisher pub;
//static double distance;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;


double no_frame=0,gremoval_time=0;
  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud_filtered;
  //Sensor msgs to pointcloud2 pointer
 // pcl::PointCloud<pcl::PointXYZI> cloud1;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud1;
  pcl::fromROSMsg (*msg, cloud1);
 



//-----------------------------------Ground Removed---------------------------------------


  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud2 (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
  *cloud2=cloud1;



  ros::Time::init();
  ros::Time begin= ros::Time::now();
   std::cout<<begin<<std::endl;

pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
int count_groud=0;

pcl::PointCloud<pcl::PointXYZI> cloud3 ;
////////////////////////////////////////////////////////
float xmax,xmin,ymax,ymin;
for (size_t i = 0; i < cloud2->points.size (); ++i)
   {
	
	 if (cloud2->points[i].x>-100 && cloud2->points[i].y>-100  && cloud2->points[i].y<100)
    {
        pcl::PointXYZI point;
        point.x = cloud2->points[i].x;
        point.y = cloud2->points[i].y;
        point.z = cloud2->points[i].z;
        point.intensity=cloud2->points[i].intensity;
        cloud3.push_back(point);
     }  
   }
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
//pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
*cloud=cloud3;



pcl::PointCloud<pcl::PointXYZRGB> road_cloud;
for (int i = 0; i < cloud->points.size(); i++)

{
if (cloud->points[i].intensity<10 && cloud->points[i].z<-1)
  {
        uint8_t r = 0, g = 0, b = 255;
     
     pcl::PointXYZRGB point = pcl::PointXYZRGB(r, g, b);
     point.x = cloud->points[i].x;
     point.y = cloud->points[i].y;
     point.z = cloud->points[i].z;
     road_cloud.push_back(point);
  }
}




    /**************publish*************************/
  pcl::PCLPointCloud2Ptr left_curb_cloudptr(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(road_cloud, *left_curb_cloudptr);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_msgs;
  pcl::fromPCLPointCloud2(*left_curb_cloudptr, cloud_msgs);
  sensor_msgs::PointCloud2 rosCloud_left;
  pcl::toROSMsg(cloud_msgs, rosCloud_left);
  rosCloud_left.header.frame_id = "velodyne";

  //.... do something with the input and generate the output...
  pub.publish(rosCloud_left);





  ros::Time time_at_end= ros::Time::now();
  std::cout<<time_at_end<<std::endl;
  std::cout<<"time for ground removal "<<time_at_end-begin<<std::endl;
  gremoval_time+=(time_at_end-begin).toSec();
  no_frame++;
  std::cout<<"average ground removal time: "<< (gremoval_time/no_frame)<<std::endl;
  std::cout << "Total no of frames: "<<no_frame<<std::endl;

//------------------------------------------------------------------------------------------
 
   
   

  
  }
  
  
  
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "ground_removal_ring");
  ros::NodeHandle nh;


  ros::Subscriber sub_ = nh.subscribe("/velodyne_points", 1, callback);
  
  pub = nh.advertise<sensor_msgs::PointCloud2>("/road_topic", 1);

  ros::spin();

  return 0;
}



