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
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <limits>
#include <utility>
using namespace pcl;
//using namespace std;

int j = 1;
ros::Publisher markerPub;
ros::Publisher pub;
ros::Publisher box_pub;
//static double distance;
int count;
ros::Subscriber sub_;

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//using namespace std;

float find_angle_between_point(pcl::PointXYZI p1, pcl::PointXYZI p2)
{
  float mod_p1 = sqrt((p1.x * p1.x) + (p1.y * p1.y) + (p1.z * p1.z));
  float mod_p2 = sqrt((p2.x * p2.x) + (p2.y * p2.y) + (p2.z * p2.z));
  float p1_dot_p2 = p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
  float alpha = acos(p1_dot_p2 / (mod_p1 * mod_p2));
  return (alpha);
}
struct Point_ring_phi
{
  pcl::PointXYZI pcl_pointxyzi;
  float phi;
  float range;
  int label;
};

void swap(Point_ring_phi *a, Point_ring_phi *b)
{
  Point_ring_phi t = *a;
  *a = *b;
  *b = t;
}

// partition the array using last element as pivot
int partition(Point_ring_phi arr[], int low, int high)
{
  float pivot = arr[high].phi; // pivot
  int i = (low - 1);

  for (int j = low; j <= high - 1; j++)
  {
    //if current element is smaller than pivot, increment the low element
    //swap elements at i and j
    if (arr[j].phi <= pivot)
    {
      i++; // increment index of smaller element
      swap(&arr[i], &arr[j]);
    }
  }
  swap(&arr[i + 1], &arr[high]);
  return (i + 1);
}

void quickSort(Point_ring_phi arr[], int low, int high)
{
  if (low < high)
  {
    //partition the array
    int pivot = partition(arr, low, high);

    //sort the sub arrays independently
    quickSort(arr, low, pivot - 1);
    quickSort(arr, pivot + 1, high);
  }
}

////////////***********************CALLBACK FUNCTION*******************************************

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  ros::Time::init();
  ros::Time begin = ros::Time::now();
  std::cout << "inside callback" << std::endl;
  //Sensor msgs to pointcloud2 pointer
  //pcl::PointCloud<pcl::PointXYZ> cloud1;
  //pcl::fromROSMsg (*msg, cloud1);

  PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR(new PointCloud<velodyne_pointcloud::PointXYZIR>);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  fromPCLPointCloud2(pcl_pc2, *cloud_XYZIR);

  //int number_of_col=cloud_XYZIR->points.size()/32*2;
  int number_of_col = 65536 / 32;
  Point_ring_phi array_P_r_phi[32][number_of_col];

  //***initialize all the points label with zero
  for (int i = 0; i < 32; i++)
    for (int j = 0; j < number_of_col; j++)
    {
      array_P_r_phi[i][j].label = 0;
      array_P_r_phi[i][j].range = 0;
    }

  //**counter for number of points in a row
  int counter[32] = {0};
  std::cout << "cloud_XYZIR->points.size() " << cloud_XYZIR->points.size() << " number_of_col " << number_of_col << std::endl;
  for (size_t i = 0; i < cloud_XYZIR->points.size(); i++)
  {
    int position;
    int ring_number;
    pcl::PointXYZI temp_point_xyzi;
    temp_point_xyzi.x = cloud_XYZIR->points[i].x;
    temp_point_xyzi.y = cloud_XYZIR->points[i].y;
    temp_point_xyzi.z = cloud_XYZIR->points[i].z;
    temp_point_xyzi.intensity = cloud_XYZIR->points[i].intensity;
    float temp_phi = 180 / M_PI * atan(cloud_XYZIR->points[i].y / cloud_XYZIR->points[i].x);
    if (cloud_XYZIR->points[i].x < 0 && cloud_XYZIR->points[i].y > 0)
      temp_phi = temp_phi + 180;
    if (cloud_XYZIR->points[i].x < 0 && cloud_XYZIR->points[i].y < 0)
      temp_phi = temp_phi + 180;
    if (cloud_XYZIR->points[i].x > 0 && cloud_XYZIR->points[i].y < 0)
      temp_phi = temp_phi + 360;
    position = temp_phi / 0.2;

    float temp_range = sqrt(cloud_XYZIR->points[i].x * cloud_XYZIR->points[i].x + cloud_XYZIR->points[i].y * cloud_XYZIR->points[i].y + cloud_XYZIR->points[i].z * cloud_XYZIR->points[i].z);
    //std::cout<<cloud_XYZIR->points[i].ring<<"  "<<cloud_XYZIR->points[i].intensity<<std::endl;
    ring_number = cloud_XYZIR->points[i].ring;

    array_P_r_phi[ring_number][position].pcl_pointxyzi = temp_point_xyzi;
    array_P_r_phi[ring_number][position].phi = temp_phi;
    array_P_r_phi[ring_number][position].range = temp_range;
    counter[ring_number]++;

    //std::cout<<temp_phi<<std::endl;
  }
  for (int i = 0; i < 32; i++)
    std::cout << "ring " << i << " " << counter[i] << std::endl;

  ros::Time depth = ros::Time::now();
  std::cout << "time for craeting depth image is " << depth - begin << std::endl;

  //*********CLUSTERING******************************************************/

  int Label = 0;
  uint8_t r = 255, g = 255, b = 255;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> vector_cluster;
  pcl::PointCloud<pcl::PointXYZRGB> cluster;
  float threshold = 7;
  for (int i = 0; i < 32; i++)
    for (int j = 0; j < number_of_col; j++)
    //for (int j=0;j<100;j++)
    {
      // if (array_P_r_phi[i][j].range !=0 && array_P_r_phi[i][j].range <2 )
      //threshold=3+array_P_r_phi[i][j].range*(10-2)/100;
       // threshold=0.25;
        //if (array_P_r_phi[i][j].range !=0 && array_P_r_phi[i][j].range >2 ) threshold=4 ;
        
        
      if (array_P_r_phi[i][j].label == 0 && array_P_r_phi[i][j].range != 0)
      {
        //pcl::PointCloud<pcl::PointXYZRGB> cluster;
        Label = Label + 1;
        r = r + Label * 20;
        g = g + Label * 25;
        b = b + Label * 30;

        //pcl::PointXYZRGB point = pcl::PointXYZRGB((uint8_t) array_P_r_phi[i][j].label,(uint8_t) 2*array_P_r_phi[i][j].label,(uint8_t) 3*array_P_r_phi[i][j].label);
        pcl::PointXYZRGB point = pcl::PointXYZRGB(r, g, b);
        point.x = array_P_r_phi[i][j].pcl_pointxyzi.x;
        point.y = array_P_r_phi[i][j].pcl_pointxyzi.y;
        point.z = array_P_r_phi[i][j].pcl_pointxyzi.z;
        cluster.push_back(point);

        std::queue<std::pair<int, int>> myqueue;
        myqueue.push(std::make_pair(i, j));

        while (!myqueue.empty())
        {
          std::pair<int, int> temp_pair = myqueue.front();
          if (array_P_r_phi[temp_pair.first][temp_pair.second].label == 0)
            array_P_r_phi[temp_pair.first][temp_pair.second].label = Label;
          /****LEFT NEIGHBOR**************************/
          float d1, d2;

          if (temp_pair.second > 0)
          {
            // if(array_P_r_phi[temp_pair.first][temp_pair.second-1].range!=0 && array_P_r_phi[temp_pair.first][temp_pair.second-1].label==0 )
            int temp = temp_pair.second - 1;
            while (temp >= temp_pair.second - 5 && temp >= 0)
            //while (temp>=0)
            {
              if (array_P_r_phi[temp_pair.first][temp].range != 0 && array_P_r_phi[temp_pair.first][temp].label == 0)
              {

                float alpha = find_angle_between_point(array_P_r_phi[temp_pair.first][temp_pair.second].pcl_pointxyzi, array_P_r_phi[temp_pair.first][temp].pcl_pointxyzi);
                if (array_P_r_phi[temp_pair.first][temp_pair.second].range > array_P_r_phi[temp_pair.first][temp].range)
                {
                  d1 = array_P_r_phi[temp_pair.first][temp_pair.second].range;
                  d2 = array_P_r_phi[temp_pair.first][temp].range;
                }
                else
                {
                  d1 = array_P_r_phi[temp_pair.first][temp].range;
                  d2 = array_P_r_phi[temp_pair.first][temp_pair.second].range;
                }
                float beta = 180 / M_PI * atan(d2 * sin(alpha) / (d1 - d2 * cos(alpha)));
                //std::cout<<"beta is "<<beta<<std::endl;
                if (beta > threshold)
                {
                  myqueue.push(std::make_pair(temp_pair.first, temp));
                  array_P_r_phi[temp_pair.first][temp].label = Label;

                  pcl::PointXYZRGB point = pcl::PointXYZRGB(r, g, b);
                  point.x = array_P_r_phi[temp_pair.first][temp].pcl_pointxyzi.x;
                  point.y = array_P_r_phi[temp_pair.first][temp].pcl_pointxyzi.y;
                  point.z = array_P_r_phi[temp_pair.first][temp].pcl_pointxyzi.z;
                  cluster.push_back(point);
                }

                break;
              }

              if (array_P_r_phi[temp_pair.first][temp].range != 0 && array_P_r_phi[temp_pair.first][temp].label != 0)
                break;
              if (array_P_r_phi[temp_pair.first][temp].range == 0 && array_P_r_phi[temp_pair.first][temp].label == 0)
                temp = temp - 1;
            }
          }

          if (temp_pair.second == 0)
          {
            if (array_P_r_phi[temp_pair.first][number_of_col - 1].range != 0 && array_P_r_phi[temp_pair.first][number_of_col - 1].label == 0)
            {
              float alpha = find_angle_between_point(array_P_r_phi[temp_pair.first][temp_pair.second].pcl_pointxyzi, array_P_r_phi[temp_pair.first][number_of_col - 1].pcl_pointxyzi);
              if (array_P_r_phi[temp_pair.first][temp_pair.second].range > array_P_r_phi[temp_pair.first][number_of_col - 1].range)
              {
                d1 = array_P_r_phi[temp_pair.first][temp_pair.second].range;
                d2 = array_P_r_phi[temp_pair.first][number_of_col - 1].range;
              }
              else
              {
                d1 = array_P_r_phi[temp_pair.first][number_of_col - 1].range;
                d2 = array_P_r_phi[temp_pair.first][temp_pair.second].range;
              }
              float beta = 180 / M_PI * atan(d2 * sin(alpha) / (d1 - d2 * cos(alpha)));
              if (beta > threshold)
              {
                myqueue.push(std::make_pair(temp_pair.first, number_of_col - 1));
                array_P_r_phi[temp_pair.first][number_of_col - 1].label = Label;

                // pcl::PointXYZRGB point = pcl::PointXYZRGB((uint8_t) array_P_r_phi[temp_pair.first][number_of_col-1].label,(uint8_t) 2*array_P_r_phi[temp_pair.first][number_of_col-1].label,(uint8_t) 3*array_P_r_phi[temp_pair.first][number_of_col-1].label);
                pcl::PointXYZRGB point = pcl::PointXYZRGB(r, g, b);
                point.x = array_P_r_phi[temp_pair.first][number_of_col - 1].pcl_pointxyzi.x;
                point.y = array_P_r_phi[temp_pair.first][number_of_col - 1].pcl_pointxyzi.y;
                point.z = array_P_r_phi[temp_pair.first][number_of_col - 1].pcl_pointxyzi.z;
                cluster.push_back(point);
              }
            }
          }

          /*********RIGHT NEIGHBOR******************/
          if (temp_pair.second < number_of_col - 1)
          {
            int temp = temp_pair.second + 1;
            while (temp <= temp_pair.second + 2 && temp <= number_of_col - 1)
            //while(temp<=number_of_col-1)
            {
              if (array_P_r_phi[temp_pair.first][temp].range != 0 && array_P_r_phi[temp_pair.first][temp].label == 0)
              {
                float alpha = find_angle_between_point(array_P_r_phi[temp_pair.first][temp_pair.second].pcl_pointxyzi, array_P_r_phi[temp_pair.first][temp].pcl_pointxyzi);
                if (array_P_r_phi[temp_pair.first][temp_pair.second].range > array_P_r_phi[temp_pair.first][temp].range)
                {
                  d1 = array_P_r_phi[temp_pair.first][temp_pair.second].range;
                  d2 = array_P_r_phi[temp_pair.first][temp].range;
                }
                else
                {
                  d1 = array_P_r_phi[temp_pair.first][temp].range;
                  d2 = array_P_r_phi[temp_pair.first][temp_pair.second].range;
                }
                float beta = 180 / M_PI * atan(d2 * sin(alpha) / (d1 - d2 * cos(alpha)));
                //std::cout<<"beta is "<<beta<<std::endl;
                if (beta > threshold)
                {
                  myqueue.push(std::make_pair(temp_pair.first, temp));
                  array_P_r_phi[temp_pair.first][temp].label = Label;

                  pcl::PointXYZRGB point = pcl::PointXYZRGB(r, g, b);
                  point.x = array_P_r_phi[temp_pair.first][temp].pcl_pointxyzi.x;
                  point.y = array_P_r_phi[temp_pair.first][temp].pcl_pointxyzi.y;
                  point.z = array_P_r_phi[temp_pair.first][temp].pcl_pointxyzi.z;
                  cluster.push_back(point);
                }
                break;
              }
              if (array_P_r_phi[temp_pair.first][temp].range != 0 && array_P_r_phi[temp_pair.first][temp].label != 0)
                break;
              if (array_P_r_phi[temp_pair.first][temp].range == 0 && array_P_r_phi[temp_pair.first][temp].label == 0)
                temp = temp + 1;
            }
          }

          if (temp_pair.second == number_of_col - 1)
          {
            if (array_P_r_phi[temp_pair.first][0].range != 0 && array_P_r_phi[temp_pair.first][0].label == 0)
            {
              float alpha = find_angle_between_point(array_P_r_phi[temp_pair.first][temp_pair.second].pcl_pointxyzi, array_P_r_phi[temp_pair.first][0].pcl_pointxyzi);
              if (array_P_r_phi[temp_pair.first][temp_pair.second].range > array_P_r_phi[temp_pair.first][0].range)
              {
                d1 = array_P_r_phi[temp_pair.first][temp_pair.second].range;
                d2 = array_P_r_phi[temp_pair.first][0].range;
              }
              else
              {
                d1 = array_P_r_phi[temp_pair.first][0].range;
                d2 = array_P_r_phi[temp_pair.first][temp_pair.second].range;
              }
              float beta = 180 / M_PI * atan(d2 * sin(alpha) / (d1 - d2 * cos(alpha)));
              if (beta > threshold)
              {
                myqueue.push(std::make_pair(temp_pair.first, 0));
                array_P_r_phi[temp_pair.first][0].label = Label;

                // pcl::PointXYZRGB point = pcl::PointXYZRGB((uint8_t) array_P_r_phi[temp_pair.first][0].label,(uint8_t) 2*array_P_r_phi[temp_pair.first][0].label,(uint8_t) 3*array_P_r_phi[temp_pair.first][0].label);
                pcl::PointXYZRGB point = pcl::PointXYZRGB(r, g, b);
                point.x = array_P_r_phi[temp_pair.first][0].pcl_pointxyzi.x;
                point.y = array_P_r_phi[temp_pair.first][0].pcl_pointxyzi.y;
                point.z = array_P_r_phi[temp_pair.first][0].pcl_pointxyzi.z;
                cluster.push_back(point);
              }
            }
          }
          
         

          /***********TOP NEIGHBOR*****************************/
          if (temp_pair.first > 0)
          {
             int temp = temp_pair.first - 1;
            while (temp >= temp_pair.first - 5 && temp >= 0)
            {
            //if (array_P_r_phi[temp_pair.first - 1][temp_pair.second].range != 0 && array_P_r_phi[temp_pair.first - 1][temp_pair.second].label == 0)
            if (array_P_r_phi[temp][temp_pair.second].range != 0 && array_P_r_phi[temp][temp_pair.second].label == 0)
            {
              float alpha = find_angle_between_point(array_P_r_phi[temp_pair.first][temp_pair.second].pcl_pointxyzi, array_P_r_phi[temp][temp_pair.second].pcl_pointxyzi);
              if (array_P_r_phi[temp_pair.first][temp_pair.second].range > array_P_r_phi[temp][temp_pair.second].range)
              {
                d1 = array_P_r_phi[temp_pair.first][temp_pair.second].range;
                d2 = array_P_r_phi[temp][temp_pair.second].range;
              }
              else
              {
                d1 = array_P_r_phi[temp][temp_pair.second].range;
                d2 = array_P_r_phi[temp_pair.first][temp_pair.second].range;
              }
              float beta = 180 / M_PI * atan(d2 * sin(alpha) / (d1 - d2 * cos(alpha)));
              if (beta > threshold)
              {
                myqueue.push(std::make_pair(temp, temp_pair.second));
                array_P_r_phi[temp][temp_pair.second].label = Label;

                //pcl::PointXYZRGB point = pcl::PointXYZRGB((uint8_t) array_P_r_phi[temp_pair.first - 1][temp_pair.second].label,(uint8_t) 2*array_P_r_phi[temp_pair.first - 1][temp_pair.second].label,(uint8_t) 3*array_P_r_phi[temp_pair.first - 1][temp_pair.second].label);
                pcl::PointXYZRGB point = pcl::PointXYZRGB(r, g, b);
                point.x = array_P_r_phi[temp][temp_pair.second].pcl_pointxyzi.x;
                point.y = array_P_r_phi[temp][temp_pair.second].pcl_pointxyzi.y;
                point.z = array_P_r_phi[temp][temp_pair.second].pcl_pointxyzi.z;
                cluster.push_back(point);
              }
            }
             temp = temp - 1;
          }
          }
           
          /***********DOWN NEIGHBOR*****************************/
          if (temp_pair.first < 31)
          {
           int temp = temp_pair.first + 1;
            while (temp <= temp_pair.first + 5 && temp <= 31)
            {
           // if (array_P_r_phi[temp_pair.first + 1][temp_pair.second].range != 0 && array_P_r_phi[temp_pair.first + 1][temp_pair.second].label == 0)
           if (array_P_r_phi[temp][temp_pair.second].range != 0 && array_P_r_phi[temp][temp_pair.second].label == 0)
            {
              float alpha = find_angle_between_point(array_P_r_phi[temp_pair.first][temp_pair.second].pcl_pointxyzi, array_P_r_phi[temp][temp_pair.second].pcl_pointxyzi);
              if (array_P_r_phi[temp_pair.first][temp_pair.second].range > array_P_r_phi[temp][temp_pair.second].range)
              {
                d1 = array_P_r_phi[temp_pair.first][temp_pair.second].range;
                d2 = array_P_r_phi[temp][temp_pair.second].range;
              }
              else
              {
                d1 = array_P_r_phi[temp][temp_pair.second].range;
                d2 = array_P_r_phi[temp_pair.first][temp_pair.second].range;
              }
              float beta = 180 / M_PI * atan(d2 * sin(alpha) / (d1 - d2 * cos(alpha)));
              if (beta > threshold)
              {
                myqueue.push(std::make_pair(temp, temp_pair.second));
                array_P_r_phi[temp][temp_pair.second].label = Label;

                //pcl::PointXYZRGB point = pcl::PointXYZRGB((uint8_t) array_P_r_phi[temp_pair.first + 1][temp_pair.second].label,(uint8_t) 2*array_P_r_phi[temp_pair.first + 1][temp_pair.second].label,(uint8_t) 3*array_P_r_phi[temp_pair.first + 1][temp_pair.second].label);
                pcl::PointXYZRGB point = pcl::PointXYZRGB(r, g, b);
                point.x = array_P_r_phi[temp][temp_pair.second].pcl_pointxyzi.x;
                point.y = array_P_r_phi[temp][temp_pair.second].pcl_pointxyzi.y;
                point.z = array_P_r_phi[temp][temp_pair.second].pcl_pointxyzi.z;
                cluster.push_back(point);
              }
            }
            temp = temp + 1;
          }
           
          }
            
          myqueue.pop();
        }
      }
      //std::cout<<"cluster size"<<cluster.points.size()<<std::endl;
      if (cluster.points.size() > 1 )
        vector_cluster.push_back(cluster);
      cluster.clear();
    }
  
  /*************************************************/
  ros::Time after_depth = ros::Time::now();
  std::cout << "time for clustering " << after_depth - begin << std::endl;
  /**********creating pcd of clusters******************/
  std::cout << "vector cluster size " << vector_cluster.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB> merged_cluster;
  for (int i = 0; i < vector_cluster.size(); i++)
  {
    if (i == 0)
      merged_cluster = vector_cluster[i];
    else
      merged_cluster = merged_cluster + vector_cluster[i];
  }

  //pcl::io::savePCDFileASCII("/home/bhaskar/Downloads/result/ring_data/clusters/all_cluster.pcd", merged_cluster);

  /**************publish*************************/
  pcl::PCLPointCloud2Ptr cloud3(new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(merged_cluster, *cloud3);
  pcl::PointCloud<pcl::PointXYZRGB> cloud_msgs;
  pcl::fromPCLPointCloud2(*cloud3, cloud_msgs);
  sensor_msgs::PointCloud2 rosCloud;
  pcl::toROSMsg(cloud_msgs, rosCloud);
  rosCloud.header.frame_id = "velodyne";

  //.... do something with the input and generate the output...
  pub.publish(rosCloud);

  /**********************************MARKER for SHOWING CLUSTERS************************/
  visualization_msgs::MarkerArray clusterMarkers;
 /*for (int i=0;i<vector_cluster.size();i++)
     {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = i;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
 //--------------------------------
    pcl::PointXYZRGB centroid;
    pcl::computeCentroid(vector_cluster[i], centroid);
     // std::vector<double> cen_enu = {centroid.x, centroid.y, centroid.z};
  // geometry_msgs::Point pt;
   // pt.x=centroid.x;
    //pt.y=centroid.y;
    //pt.z=centroid.z;
   double distance=sqrt(pow(centroid.x,2)+pow(centroid.y,2));
   //std::cout<<"position  "<<position<<std::endl;
   std::stringstream marker_text ;
   marker_text<<distance;
  // marker_text<<"cluster "<<i;
   std::string marker_text_string=marker_text.str();
   //std::cout<<"s is "<<s<<std::endl;
   marker.pose.position.x = centroid.x;
   marker.pose.position.y = centroid.y;
   marker.pose.position.z = centroid.z+1;
   marker.pose.orientation.x = 0.0;
   marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
   marker.scale.x = 0.5;
   marker.scale.y = 0.5;
   marker.scale.z = 0.5;
   marker.color.a = 4.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
   marker.color.g = 1.0;
   marker.color.b = 1.0;

   //marker.text="object";
   marker.text=marker_text_string;
   clusterMarkers.markers.push_back(marker);

}
//------marker for origin--------------------------
  visualization_msgs::Marker marker1;
    marker1.header.frame_id = "velodyne";
    marker1.header.stamp = ros::Time();
    marker1.ns = "my_namespace";
    marker1.id = vector_cluster.size();
    marker1.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker1.action = visualization_msgs::Marker::ADD;
   marker1.pose.position.x = 0;
   marker1.pose.position.y = 0;
   marker1.pose.position.z = -1;
   marker1.pose.orientation.x = 0.0;
   marker1.pose.orientation.y = 0.0;
  marker1.pose.orientation.z = 0.0;
  marker1.pose.orientation.w = 1.0;
   marker1.scale.x = 1;
   marker1.scale.y = 1;
   marker1.scale.z = 1;8
   marker1.color.a = 4.0; // Don't forget to set the alpha!
  marker1.color.r = 0.0;
   marker1.color.g = 0.0;
   marker1.color.b = 1.0;
 
   //marker.text="object";
   std::stringstream origin;
   //std::string s_or="origin";
   //origin<<s_or;
   //origin<<"\n";
   origin<<"[0,0,0]";
   marker1.text=origin.str();
  clusterMarkers.markers.push_back(marker1);*/
//----------------------------------------------------------------//

   //markerPub.publish(clusterMarkers);
 
  /******Bounding Box*************************************/
  jsk_recognition_msgs::BoundingBoxArray box_array;
  //bounding box array creation
  for (int i = 0; i < vector_cluster.size(); i++)
  {
  jsk_recognition_msgs::BoundingBox element;
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(vector_cluster[i], minPt, maxPt);
    pcl::PointXYZRGB centroid;
    pcl::computeCentroid(vector_cluster[i], centroid);
    element.label = i + 1;
    element.pose.position.x = centroid.x;
    element.pose.position.y = centroid.y;
    element.pose.position.z = centroid.z;
    element.dimensions.x = maxPt.x - minPt.x;
    element.dimensions.y = maxPt.y - minPt.y;
    element.dimensions.z = maxPt.z - minPt.z;
    element.pose.orientation.w = 1.0;
    element.header.frame_id = "velodyne";
    element.header.stamp = msg->header.stamp;
    if (maxPt.x - minPt.x<5 && maxPt.y - minPt.y<5 &&  maxPt.z - minPt.z<2.5)
    box_array.boxes.push_back(element);
    
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "velodyne";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = i;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
 //--------------------------------
    //pcl::PointXYZRGB centroid;
    //pcl::computeCentroid(vector_cluster[i], centroid);
     // std::vector<double> cen_enu = {centroid.x, centroid.y, centroid.z};
  // geometry_msgs::Point pt;
   // pt.x=centroid.x;
    //pt.y=centroid.y;
    //pt.z=centroid.z;
    int cluster_size=vector_cluster[i].size();
    double volume=(maxPt.x - minPt.x)*(maxPt.y - minPt.y)*(maxPt.z - minPt.z);
   double density=cluster_size/volume;
   double distance=sqrt(pow(centroid.x,2)+pow(centroid.y,2));
   //std::cout<<"position  "<<position<<std::endl;
   std::stringstream marker_text ;
   marker_text<<distance<<'\n'<<cluster_size<<'\n'<<density;
  // marker_text<<distance;
  // marker_text<<"cluster "<<i;
   std::string marker_text_string=marker_text.str();
   //std::cout<<"s is "<<s<<std::endl;
   marker.pose.position.x = centroid.x;
   marker.pose.position.y = centroid.y;
   marker.pose.position.z = centroid.z+1;
   marker.pose.orientation.x = 0.0;
   marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
   marker.scale.x = 0.5;
   marker.scale.y = 0.5;
   marker.scale.z = 0.5;
   marker.color.a = 4.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
   marker.color.g = 1.0;
   marker.color.b = 1.0;

   //marker.text="object";
   marker.text=marker_text_string;
    if (maxPt.x - minPt.x<5 && maxPt.y - minPt.y<5 &&  maxPt.z - minPt.z<2.5)
   clusterMarkers.markers.push_back(marker);

 
  
  }
  
markerPub.publish(clusterMarkers);

box_array.header.stamp = msg->header.stamp;
box_array.header.frame_id = "velodyne";
std::cout << "Bounding Box is published\n"
          << std::endl;
box_pub.publish(box_array);
//box_pub.publish(element);

/*****************************************************************************/
std::cout << "the number of clusters is " << Label << std::endl;
ros::Time time_at_end = ros::Time::now();
std::cout << "time for complete frame processing " << time_at_end - begin << std::endl;

std::cout << "-------END OF FRAME-------------------------------" << std::endl;
}

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "ground_removal_new");
  ros::NodeHandle nh;
  std::cout << "inside main" << std::endl;
  count = 0;
  //n.getParam("/main/x1",x1);
  //sub_ = nh.subscribe("/velodyne_points", 1, callback);
  //sub_ = nh.subscribe("/kitti/velo/pointclouds", 1, callback);
  sub_ = nh.subscribe("/gremoved_topic", 1, callback);

  pub = nh.advertise<sensor_msgs::PointCloud2>("/depth_topic", 1);
  box_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/depth_bound_topic", 10);
  markerPub = nh.advertise<visualization_msgs::MarkerArray>("viz", 1);
  ros::spin();

  return 0;
}
