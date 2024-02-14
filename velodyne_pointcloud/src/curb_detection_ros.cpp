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
	
	 if (cloud2->points[i].x>4 && cloud2->points[i].x<8 && cloud2->points[i].y>-14 && cloud2->points[i].y<14)
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

std::vector <float> negative_y_normals[70];
std::vector <float> positive_y_normals[70];

float negative_y_avgz[70]={ 0 };
float positive_y_avgz[70]={ 0 };


for (int i = 0; i < cloud->points.size(); i++)


{
if (cloud->points[i].y<0)
  {
   int temp=int((0-cloud->points[i].y)/0.2); 
   int n=negative_y_normals[temp].size();
   negative_y_normals[temp].push_back(cloud->points[i].z);
   negative_y_avgz[temp]=(n*negative_y_avgz[temp]+cloud->points[i].z)/(n+1);
   }
  if (cloud->points[i].y>=0)
  {
  int temp=cloud->points[i].y/0.2; 
   int n=positive_y_normals[temp].size();
  positive_y_normals[temp].push_back(cloud->points[i].z); 
  positive_y_avgz[temp]=(n*positive_y_avgz[temp]+cloud->points[i].z)/(n+1);
  }
}

pcl::PointCloud<pcl::PointXYZRGB> left_curb_cloud;

for (int i=0;i<70;i++)
{
//std::cout<<positive_y_avgz[i]<<std::endl;
if (positive_y_avgz[i+1]>positive_y_avgz[i]+0.1)
  {
  
  for (int t = 0; t < cloud->points.size(); t++)
  {
  if ( cloud->points[t].y>i*0.2 && cloud->points[t].y<(i+1)*0.2)
     {
     uint8_t r = 255, g = 255, b = 255;
     std::cout<<"x "<<cloud->points[t].x<<std::endl;
     pcl::PointXYZRGB point = pcl::PointXYZRGB(r, g, b);
     point.x = cloud->points[t].x;
     point.y = cloud->points[t].y;
     point.z = cloud->points[t].z;
     left_curb_cloud.push_back(point);
     }
    }
   
    /**************publish*************************/
  /*pcl::PCLPointCloud2Ptr left_curb_cloudptr(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(left_curb_cloud, *left_curb_cloudptr);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_msgs;
  pcl::fromPCLPointCloud2(*left_curb_cloudptr, cloud_msgs);
  sensor_msgs::PointCloud2 rosCloud_left;
  pcl::toROSMsg(cloud_msgs, rosCloud_left);
  rosCloud_left.header.frame_id = "velodyne";

  //.... do something with the input and generate the output...
  pub.publish(rosCloud_left);*/
  float left_curb_dist=0.2*(i+1);
  std::cout<<"left cusrb distance is "<<left_curb_dist<<std::endl;
  //////////publish curb data

  
  break;
  }
  
}
pcl::PointCloud<pcl::PointXYZRGB> right_curb_cloud;
for (int i=0;i<70;i++)
{

  if (negative_y_avgz[i+1]>negative_y_avgz[i]+0.1)
  {
  
  for (int t = 0; t < cloud->points.size(); t++)
  {
  if ( cloud->points[t].y<i*(-0.2) && cloud->points[t].y>(i+1)*(-0.2))
     {
     uint8_t r = 255, g = 255, b = 255;
     std::cout<<"x "<<cloud->points[t].x<<std::endl;
     pcl::PointXYZRGB point = pcl::PointXYZRGB(r, g, b);
     point.x = cloud->points[t].x;
     point.y = cloud->points[t].y;
     point.z = cloud->points[t].z;
     right_curb_cloud.push_back(point);
     }
    }
  float right_curb_dist=0.2*(i+1);
  std::cout<<"right cusrb distance is "<<right_curb_dist<<std::endl;
  break;
  }
  

}

pcl::PointCloud<pcl::PointXYZRGB> curb_cloud;

curb_cloud=left_curb_cloud+right_curb_cloud;

    /**************publish*************************/
  pcl::PCLPointCloud2Ptr left_curb_cloudptr(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(curb_cloud, *left_curb_cloudptr);
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
  
  pub = nh.advertise<sensor_msgs::PointCloud2>("/curb_topic", 1);

  ros::spin();

  return 0;
}



