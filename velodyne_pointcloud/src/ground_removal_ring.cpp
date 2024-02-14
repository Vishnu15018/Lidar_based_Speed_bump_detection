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
 
  
  
  
  /************xyzir***************************************/
  
//PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR (new PointCloud<velodyne_pointcloud::PointXYZIR>);
  /*pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  fromPCLPointCloud2(pcl_pc2, *cloud_XYZIR);*/


//-----------------------------------Ground Removed---------------------------------------

  //pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::ptr cloud1;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
  *cloud=cloud1;
  
  //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  
 //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


  ros::Time::init();
  ros::Time begin= ros::Time::now();
   std::cout<<begin<<std::endl;

pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
int count_groud=0;


////////////////////////////////////////////////////////
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

std::cout<<"range of x valoues"<<xmin<<"to"<<xmax<<std::endl;
std::cout<<"range of y valoues"<<ymin<<"to"<<ymax<<std::endl;
float xminround=ceilf(xmin-1);
float xmaxround=ceilf(xmax+1);
float yminround=ceilf(ymin-1);
float ymaxround=ceilf(ymax+1);
 int xbin,ybin;
float d=0.5;
xbin=(xmaxround-xminround)/d;
ybin=(ymaxround-yminround)/d;
 std::cout<<xbin<<"   "<<ybin<<std::endl;
static std::vector<int> vect[1000][1200];

for (size_t i = 0; i < cloud->points.size (); ++i)
{
  int dx,dy;
 pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
 dx=(pt.x-xmin)/d;
 dy=(pt.y-ymin)/d;
 //std::cout<<dx<<"   "<<dy<<std::endl;
 vect[dx][dy].push_back(i);
}
float zmin[xbin+1][ybin+1];
float zmax[xbin+1][ybin+1];
float zavg[xbin+1][ybin+1];
for (int i=1;i<=xbin;++i)
for (int j=1;j<=ybin;++j)
{
zavg[i][j]=0;
int count=0;
for (int k=0;k<vect[i][j].size();++k)
{
zavg[i][j]=zavg[i][j]+cloud->points[vect[i][j][k]].z;
if (count ==0){
zmin[i][j]=cloud->points[vect[i][j][k]].z;ros::Time begin= ros::Time::now();
//std::cout<<begin<<std::endl;
zmax[i][j]=cloud->points[vect[i][j][k]].z;count=count+1;}
else {if (zmin[i][j] >= cloud->points[vect[i][j][k]].z)
      zmin[i][j]=cloud->points[vect[i][j][k]].z;
      if (zmax[i][j] <= cloud->points[vect[i][j][k]].z)
      zmax[i][j]=cloud->points[vect[i][j][k]].z;}

}
zavg[i][j]=zavg[i][j]/vect[i][j].size();
//std::cout<<"avg z for"<<i<<"and"<<j<<"is equal to"<<zavg[i][j]<<"     ";
}
float th1=0.2;	
float th2=0.3;
float th3=0.1;
float ground_threshold=0.1; //oringinal 0.1
float f=5;	
int count_ground=0; 


for (int i=1;i<=xbin;++i)
	for (int j=1;j<=ybin;++j)
	{
	if ((zmin[i][j]<th1) && (zmax[i][j]-zmin[i][j]>th2))
	{
		for (int k=0;k<vect[i][j].size();++k)
		{

			if ( (cloud->points[vect[i][j][k]].z <zmin[i][j]+ground_threshold))
     			{
      			//inliers->indices.push_back(vect[i][j][k]);
      			count_groud++;
      			
     			}
     			else{
     			   cloud_filtered.push_back(cloud->points[vect[i][j][k]]);
     			}
		}
	}	
	
	
	else if((zmin[i][j]<th1) && (th3<zmax[i][j]-zmin[i][j]) &&(zmax[i][j]-zmin[i][j]>th2))
	{
		for (int k=0;k<vect[i][j].size();++k)
		{

		if  (cloud->points[vect[i][j][k]].z < (zmin[i][j]+(zmax[i][j]-zmin[i][j])/f))
     		{
      			//inliers->indices.push_back(vect[i][j][k]);
      			count_groud++;
     		}
     		else{
     		
     		cloud_filtered.push_back(cloud->points[vect[i][j][k]]);
     		}
     			
		}
	}
	else if((zmin[i][j]<th1) && (th3>zmax[i][j]-zmin[i][j]))
	{
		for (int k=0;k<vect[i][j].size();++k)
		{

      			//inliers->indices.push_back(vect[i][j][k]);
      			count_groud++;  
		}
	}

//Putting Non-ground points
else{
      for (int k=0;k<vect[i][j].size();++k){      
      		cloud_filtered.push_back(cloud->points[vect[i][j][k]]);
      	
      	}
    }

}


  ros::Time time_at_end= ros::Time::now();
  std::cout<<time_at_end<<std::endl;
  std::cout<<"time for ground removal "<<time_at_end-begin<<std::endl;
  gremoval_time+=(time_at_end-begin).toSec();
  no_frame++;
  std::cout<<"average ground removal time: "<< (gremoval_time/no_frame)<<std::endl;
  std::cout << "Total no of frames: "<<no_frame<<std::endl;

//------------------------------------------------------------------------------------------
 
   
   
  
  
  //pcl::fromPCLPointCloud2(cloud_filtered, cloud_msgs);
  
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(cloud_filtered,pcl_pc2);
  
  
  
  sensor_msgs::PointCloud2 rosCloud;
  pcl_conversions::fromPCL(pcl_pc2,rosCloud);
 //  rosCloud.header.frame_id="velodyne";
   
  rosCloud.header.stamp = msg->header.stamp;
    rosCloud.header.frame_id="velodyne";
    rosCloud.header.stamp=ros::Time::now();
  pub_.publish(rosCloud);
  
  
  for (int i=1;i<=xbin;++i)
  for (int j=1;j<=ybin;++j)
   {
    vect[i][j].clear();
   }
  std::cout<<"vector cleared"<<std::endl;
  
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
  
  
  pub_ = nh_ground_removal_new.advertise<sensor_msgs::PointCloud2> ("/gremoved_topic", 1);
  //pub_ = nh_ground_removal_new.advertise<PointCloud>("/published_topic", 1);

  ros::spin();

  return 0;
}



