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
#include <ctime>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>

#include <velodyne_pointcloud/organized_cloudXYZIR.h>
#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <velodyne_pointcloud/point_types.h>
#include <omp.h>
#include <queue>


#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
//#include <jsk_recognition_msgs/BoundingBox.h>
//#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <limits>
#include <utility>
using namespace pcl;
//using namespace std;

int j = 1;
ros::Publisher markerPub;
ros::Publisher pub;
//static double distance;
int count;
ros::Subscriber sub_;





////////////***********************CALLBACK FUNCTION*******************************************

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  ros::Time::init();
  ros::Time begin = ros::Time::now();
  std::cout << "inside callback" << std::endl;

  PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR(new PointCloud<velodyne_pointcloud::PointXYZIR>);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  fromPCLPointCloud2(pcl_pc2, *cloud_XYZIR);

  
 

  /**********creating pcd of clusters******************/
  
   std::stringstream ss;
   ss<<"/home/bhaskar/dataset/IIT_final2_pcd/";
   ss<<j<<".pcd";
   pcl::io::savePCDFileASCII(ss.str(), *cloud_XYZIR);
   j++;









  /*****************************************************************************/
  
  ros::Time time_at_end = ros::Time::now();
  std::cout << "time for segmentation " << time_at_end - begin << std::endl;


  std::cout << "-------END OF FRAME-------------------------------" << std::endl;
}

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "ground_removal_new");
  ros::NodeHandle nh;
  std::cout << "inside main" << std::endl;
  count = 0;

  sub_ = nh.subscribe("/gremoved_topic", 1, callback);

  //pub = nh.advertise<sensor_msgs::PointCloud2>("/depth_topic", 1);
  //markerPub= nh.advertise<visualization_msgs::MarkerArray> ("viz",1);
  ros::spin();

  return 0;
}

