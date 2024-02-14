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
float thr1 =0.09;
float thr2=0.50;
float strip_size=1;
std::array<std::vector<float>, size_array> z_values;
// std::array<std::vector<float>, size_array> z_values;
std::vector<float> avg_z(size_array); 
double no_frame=0,gremoval_time=0;
  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
    // pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
    //   //Sensor msgs to pointcloud2 pointer
    //  // pcl::PointCloud<pcl::PointXYZI> cloud1;
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud1;
    pcl::fromROSMsg (*msg, cloud1);
  
  
  
  




  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
  
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_out (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
//   //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  *cloud=cloud1;
  // PointCloud::ConstPtr *cloud_out =boost::make_shared<const PointCloud>(*cloud_out) ;
  //  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // std::cout<<"Fstatiscial filter"<<endl;
    // pcl::StatisticalOutlierRemoval<velodyne_pointcloud::PointXYZIR> sor;
    // sor.setInputCloud(*cloud_out);
    // sor.setMeanK(50); // Adjust as needed
    // sor.setStddevMulThresh(1.0); 
    // sor.filter(*cloud);
    
  ros::Time::init();
  ros::Time begin= ros::Time::now();
   std::cout<<begin<<std::endl;

   pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
// int count_groud=0;

//  std::cout << "Received point cloud with " << cloud->size() << " points." << std::endl;
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

// std::cout<<"range of x valoues "<<xmin<<" to" <<xmax<<std::endl;
// std::cout<<"range of y valoues"<<ymin<<" to "<<ymax<<std::endl;

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

 for (int i=0 ; i < cloud->points.size ();++i){
    if (cloud->points[i].x>0 && (cloud->points[i].y >=-1 && cloud->points[i].y <=1) && cloud->points[i].z <= -1.0){
            int index= static_cast<int>((cloud->points[i].x)/strip_size) ;
            std::cout<<"x: "<<cloud->points[i].x<<endl;
            std::cout<<index<<endl;
           if (index>=0 and index<size_array*strip_size){
                z_values[index].push_back(cloud->points[i].z);
        // cloud_filtered.push_back(cloud->points[i]);     
    }
    }
 }


ros::Time timestamp = msg->header.stamp;
logfile << " frame: " << no_frame << std::endl;


for (int i = 0; i < size_array; ++i) {
        float sum = 0.0;
        for (const auto& z : z_values[i]) {
            sum +=z;
        }
        float average = z_values[i].empty() ? 0.0 : sum / z_values[i].size();
        avg_z[i]=average;
        // ROS_INFO_STREAM("Average z value for points with x in range " << (i) << " to " << (i +1) << ": " << average);
        logfile << timestamp << "Average z value for points with x in range " << i << " to " << i+1 << ": " << average << std::endl;
        
    }

// for(int i=0;i<size_array;++i){
//      std::cout<<avg_z[i]<<" ";
// }
std::cout<<endl;
// std::vector<float,11> diff;
//  if (point_old.x>0 && (point_old.y >=-1 && point_old.y <=1) )
//creating a pointcloud with rgb info
for (auto& point_old : cloud->points)

    {  
      pcl::PointXYZRGB point;
         point.x=point_old.x;
         point.y=point_old.y;
         point.z=point_old.z;
         point.r=255;
         point.g=255;
         point.b=255;
         cloud_filtered.push_back(point);
    
        
    }
float n=0,n1=0,n2=0;
for(int i=0;i<size_array -1;++i){
   
   std::cout<<"average diff: "<<avg_z[i] - avg_z[i+1]<< "for "<<i<<" to "<<i+1<<endl;
   logfile << "average diff b/w: "<<i<< "to "<<i+1<<"is"<<avg_z[i] - avg_z[i+1]<<std::endl;
   if( ((avg_z[i] - avg_z[i+1]) >= thr1 ) && ((avg_z[i] - avg_z[i+1]) < thr2)){
      std::cout<<"Index : "<< i <<endl;
      n=strip_size*i;
      n1=strip_size*(i+1);
      n2=strip_size*(i+2);

      float x1=(n+n1)/2;
      float x2= (n2+n1)/2;
      int r= 255;
      int g=0;
      int b=0;
      std::cout<<"r: "<<r<<"g: "<<g<<" b: "<<b<<endl;

      for (size_t j = 0; j < cloud_filtered.size(); ++j) {
          // Get the point's x coordinate
          float x = cloud_filtered.points[j].x;
          
          // Check if the x coordinate lies between x1 and x2
          if (x >= x1 && x <= x2) {
              // Create a new point with XYZ coordinates
              // Set color of the point to red (255, 0, 0)
              // point.rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
              cloud_filtered.points[j].r = r;
              cloud_filtered.points[j].g = g;
              cloud_filtered.points[j].b= b;

              // Push the colored point to the colored point cloud    
          }
         }


}
}

no_frame++;
//   std::cout<<"average ground removal time: "<< (gremoval_time/no_frame)<<std::endl;
// std::cout << "Total no of frames: "<<no_frame<<std::endl;

sensor_msgs::PointCloud2 output_msg;
pcl::toROSMsg(cloud_filtered, output_msg);
output_msg.header = msg->header;
    // Publish the filtered point cloud
pub_.publish(output_msg);

  }






// float xminround=ceilf(xmin-1);
// float xmaxround=ceilf(xmax+1);
// float yminround=ceilf(ymin-1);
// float ymaxround=ceilf(ymax+1);
//  int xbin,ybin;
// float d=0.5;
// xbin=(xmaxround-xminround)/d;
// ybin=(ymaxround-yminround)/d;
//  std::cout<<xbin<<"   "<<ybin<<std::endl;
// static std::vector<int> vect[1000][1200];

// for (size_t i = 0; i < cloud->points.size (); ++i)
// {
//   int dx,dy;
//  pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
//  dx=(pt.x-xmin)/d;
//  dy=(pt.y-ymin)/d;
//  //std::cout<<dx<<"   "<<dy<<std::endl;
//  vect[dx][dy].push_back(i);
// }
// float zmin[xbin+1][ybin+1];
// float zmax[xbin+1][ybin+1];
// float zavg[xbin+1][ybin+1];
// for (int i=1;i<=xbin;++i)
// for (int j=1;j<=ybin;++j)
// {
// zavg[i][j]=0;
// int count=0;
// for (int k=0;k<vect[i][j].size();++k)
// {
// zavg[i][j]=zavg[i][j]+cloud->points[vect[i][j][k]].z;
// if (count ==0){
// zmin[i][j]=cloud->points[vect[i][j][k]].z;ros::Time begin= ros::Time::now();
// //std::cout<<begin<<std::endl;
// zmax[i][j]=cloud->points[vect[i][j][k]].z;count=count+1;}
// else {if (zmin[i][j] >= cloud->points[vect[i][j][k]].z)
//       zmin[i][j]=cloud->points[vect[i][j][k]].z;
//       if (zmax[i][j] <= cloud->points[vect[i][j][k]].z)
//       zmax[i][j]=cloud->points[vect[i][j][k]].z;}

// }
// zavg[i][j]=zavg[i][j]/vect[i][j].size();
// //std::cout<<"avg z for"<<i<<"and"<<j<<"is equal to"<<zavg[i][j]<<"     ";
// }
// float th1=0.2;	
// float th2=0.3;
// float th3=0.1;
// float ground_threshold=0.1; //oringinal 0.1
// float f=5;	
// int count_ground=0; 


// for (int i=1;i<=xbin;++i)
// 	for (int j=1;j<=ybin;++j)
// 	{
// 	if ((zmin[i][j]<th1) && (zmax[i][j]-zmin[i][j]>th2))
// 	{
// 		for (int k=0;k<vect[i][j].size();++k)
// 		{

// 			if ( (cloud->points[vect[i][j][k]].z <zmin[i][j]+ground_threshold))
//      			{
//       			//inliers->indices.push_back(vect[i][j][k]);
//       			count_groud++;
      			
//      			}
//      			else{
//      			   cloud_filtered.push_back(cloud->points[vect[i][j][k]]);
//      			}
// 		}
// 	}	
	
	
// 	else if((zmin[i][j]<th1) && (th3<zmax[i][j]-zmin[i][j]) &&(zmax[i][j]-zmin[i][j]>th2))
// 	{
// 		for (int k=0;k<vect[i][j].size();++k)
// 		{

// 		if  (cloud->points[vect[i][j][k]].z < (zmin[i][j]+(zmax[i][j]-zmin[i][j])/f))
//      		{
//       			//inliers->indices.push_back(vect[i][j][k]);
//       			count_groud++;
//      		}
//      		else{
     		
//      		cloud_filtered.push_back(cloud->points[vect[i][j][k]]);
//      		}
     			
// 		}
// 	}
// 	else if((zmin[i][j]<th1) && (th3>zmax[i][j]-zmin[i][j]))
// 	{
// 		for (int k=0;k<vect[i][j].size();++k)
// 		{

//       			//inliers->indices.push_back(vect[i][j][k]);
//       			count_groud++;  
// 		}
// 	}

// //Putting Non-ground points
// else{
//       for (int k=0;k<vect[i][j].size();++k){      
//       		cloud_filtered.push_back(cloud->points[vect[i][j][k]]);
      	
//       	}
//     }

// }


//   ros::Time time_at_end= ros::Time::now();
//   std::cout<<time_at_end<<std::endl;
//   std::cout<<"time for ground removal "<<time_at_end-begin<<std::endl;
//   gremoval_time+=(time_at_end-begin).toSec();


// //------------------------------------------------------------------------------------------
 
   
   
  
  
//   //pcl::fromPCLPointCloud2(cloud_filtered, cloud_msgs);
  
//   pcl::PCLPointCloud2 pcl_pc2;
//   pcl::toPCLPointCloud2(cloud_filtered,pcl_pc2);
  
  
  
//   sensor_msgs::PointCloud2 rosCloud;
//   pcl_conversions::fromPCL(pcl_pc2,rosCloud);
//  //  rosCloud.header.frame_id="velodyne";
   
//   rosCloud.header.stamp = msg->header.stamp;
//     rosCloud.header.frame_id="velodyne";
//     rosCloud.header.stamp=ros::Time::now();
//   pub_.publish(rosCloud);
  
  
//   for (int i=1;i<=xbin;++i)
//   for (int j=1;j<=ybin;++j)
//    {
//     vect[i][j].clear();
//    }
//   std::cout<<"vector cleared"<<std::endl;
  
//   }
  
  
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "ground_removal_ring");
  ros::NodeHandle nh_ground_removal_new;
  double distance;
  nh_ground_removal_new.getParam("/ground_removal_new/distance_threshold",distance);
  // std::cout<<"Value of distance "<<distance<<"\n";
  //n.getParam("/main/x1",x1);
  ros::Subscriber sub_ = nh_ground_removal_new.subscribe("/velodyne_points", 1, callback);
  
  logfile.open("/home/lidar/Desktop/vishnu/log_file_test1.txt");
    if (!logfile.is_open()) {
        ROS_ERROR("Failed to open log file");
        return 1;
    }
  pub_ = nh_ground_removal_new.advertise<sensor_msgs::PointCloud2> ("/test_topic", 1);
  //pub_ = nh_ground_removal_new.advertise<PointCloud>("/published_topic", 1);

  ros::spin();
   logfile.close();
  return 0;
}



