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
// const int size_array = 11;
float thr1 =0.036;
float thr2=0.100;


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

    // Define 2D array of vectors of float
    std::vector<std::vector<std::vector<float>>> array2D(rows, std::vector<std::vector<float>>(cols));

    std::cout<<"vector is created:"<<endl;
    // Loop through point cloud data
    for (const auto& point : cloud->points) {
        // Apply constraints
        if (point.x >= 0 && point.x <x_strip && point.y > -y_strip && point.y < y_strip && point.z<-1.1) {
            // Determine row index
            int row = static_cast<int>(point.x / strip_size);
                // std::cout<<"fillinf the values"<<endl;

            // Determine column index
            int col;
            col = static_cast<int>((point.y + y_strip) / strip_size) ;
            // std::cout<<"Point x: "<<point.x<<" pointy: "<<point.y<< " row: "<<row<<"col: "<<col<<endl;
            // Place point coordinates (x, y, z) in the array of vectors

            array2D[row][col].push_back(point.z);
        }
    }
// for finding the average in the grid
 std::vector<std::vector<float>> avg_arr(rows,std::vector<float>(cols));
  for(int i=0;i<rows;i++){
      for(int j=0 ; j<cols ;j++){
          float sum=0;
          for(int k=0; k < array2D[i][j].size();k++){
              sum+=array2D[i][j][k];
          }
          float average = array2D[i][j].empty() ? 0.0 : sum / array2D[i][j].size();
          avg_arr[i][j]=average;
          
        //   std::cout<<"array[i][j]"<<i<<"j"<<j<<" average: "<<average<<endl;
      }
  }
  // for publishing cloud 
  logfile << " frame: " << no_frame << std::endl;
  
  for (auto& point_old : cloud->points)

    {  
    //   pcl::PointXYZRGB point;
    //      point.x=point_old.x;
    //      point.y=point_old.y;
    //      point.z=point_old.z;
    //      point.r=255;
    //      point.g=255;
    //      point.b=255;
    //      cloud_filtered.push_back(point);
            pcl::PointXYZRGB pcl_point;
            pcl_point.x = point_old.x;
            pcl_point.y = point_old.y;
            pcl_point.z = point_old.z;
        if (point_old.x > 0 && point_old.y >= -y_strip && point_old.y <= y_strip && point_old.z<-1.1) {

            pcl_point.r = 0;
            pcl_point.g = 255;
            pcl_point.b = 0;
            
        }
        else{
            pcl_point.r = 255;
            pcl_point.g = 255;
            pcl_point.b = 255;
        }
        cloud_filtered.push_back(pcl_point);
    
        
    }
  //finding the differences 
//    for (size_t i = 0; i < rows; ++i) {
//         // Iterate over elements in the row
//         for (size_t j = 1; j < cols; ++j) {
//             // Calculate the difference between consecutive elements
//             float diff = std::abs(avg_arr[i][j]) - std::abs(avg_arr[i][j - 1]);
//             std::cout << "Difference between [" << i<<","<<j-1<< "] and [" << i<<","<<j << "] is: " << diff << endl;
//             logfile<<"Difference between [" << i<<","<<j-1<< "] and [" << i<<","<<j << "] is: " << diff << std::endl;
//             // Check if the difference is within the threshold
//             if (std::abs(diff)>= thr1 && std::abs(diff) <= thr2) {
//                 // Print the difference
//                 std::cout<<"THRESHOLD ACHIEVED"<<endl;
//                 // std::cout <<" Difference between [" << i <<","<<j-1<< "] and [" << i<<","<<j << "] is: " << diff << endl;
//                 std::cout<<"achieved"<<endl;
                
//                 float x1=(i)*strip_size;
//                 float x2=(i+1)*strip_size;
//                 float j1c1=(j-1)*strip_size - y_strip;
//                 float j1c2=(j)*strip_size - y_strip;
//                 float j2c2=(j+1)*strip_size - y_strip ;               
//                 float y1=(j1c1+j1c2)/2;
//                 float y2=(j1c2+j2c2)/2;
//                 std::cout<<"x1: "<< (i)*strip_size <<"x2: " << x2 <<" y1: "<<y1 <<"y2: "<<y2<<endl;
//                 logfile<<"x1: "<< (i)*strip_size <<"x2: " << x2 <<" y1: "<<y1 <<"y2: "<<y2<<std::endl;
//                 // for Highlithing the cloud 
//                 int r= 255;
//                 int g=0;
//                 int b=0;
//                 std::cout<<"r: "<<r<<"g: "<<g<<" b: "<<b<<endl;

//                 for (size_t k = 0; k < cloud_filtered.size(); ++k) {
//                     // Get the point's x coordinate
//                     float x = cloud_filtered.points[k].x;
//                     float y= cloud_filtered.points[k].y;
          
//                      // Check if the x coordinate lies between x1 and x2
//                     if (x >= x1 && x <= x2 && y>=y1 && y<=y2) {
//                             cloud_filtered.points[k].r = r;
//                             cloud_filtered.points[k].g = g;
//                             cloud_filtered.points[k].b= b;
                        
//                     // Push the colored point to the colored point cloud    
//                     }
//                 }
//             }

//         }
//     }



//  row wise access

// for (size_t i = 0; i < cols; ++i) {
//         // Iterate over elements in the row
//         for (size_t j = 1; j < rows; ++j) {
//             // Calculate the difference between consecutive elements
//             float diff = std::abs(avg_arr[j][i]) - std::abs(avg_arr[j][i- 1]);
//             std::cout << "Difference between [" << i<<","<<j-1<< "] and [" << i<<","<<j << "] is: " << diff << endl;
//             logfile<<"Difference between [" << i<<","<<j-1<< "] and [" << i<<","<<j << "] is: " << diff << std::endl;
//             // Check if the difference is within the threshold
//             if (std::abs(diff)>= thr1 && std::abs(diff) <= thr2) {
//                 // Print the difference
//                 std::cout<<"THRESHOLD ACHIEVED"<<endl;
//                 // std::cout <<" Difference between [" << i <<","<<j-1<< "] and [" << i<<","<<j << "] is: " << diff << endl;
//                 std::cout<<"achieved"<<endl;
                
//                 float y1=(i)*strip_size -y_strip;
//                 float y2=(i+1)*strip_size -y_strip;
//                 float j1c1=(j-1)*strip_size ;
//                 float j1c2=(j)*strip_size;
//                 float j2c2=(j+1)*strip_size;               
//                 float x1=(j1c1+j1c2)/2;
//                 float x2=(j1c2+j2c2)/2;
//                 std::cout<<"x1: "<< x1 <<"x2: " << x2 <<" y1: "<<y1 <<"y2: "<<y2<<endl;
//                 logfile<<"x1: "<< x1  <<"x2: " << x2 <<" y1: "<<y1 <<"y2: "<<y2<<std::endl;
//                 // for Highlithing the cloud 
//                 int r= 255;
//                 int g=0;
//                 int b=0;
//                 std::cout<<"r: "<<r<<"g: "<<g<<" b: "<<b<<endl;

//                 for (size_t k = 0; k < cloud_filtered.size(); ++k) {
//                     // Get the point's x coordinate
//                     float x = cloud_filtered.points[k].x;
//                     float y= cloud_filtered.points[k].y;
          
//                      // Check if the x coordinate lies between x1 and x2
//                     if (x >= x1 && x <= x2 && y>=y1 && y<=y2) {
//                             cloud_filtered.points[k].r = r;
//                             cloud_filtered.points[k].g = g;
//                             cloud_filtered.points[k].b= b;
                        
//                     // Push the colored point to the colored point cloud    
//                     }
//                 }
//             }

//         }
//     }
//    4 neighbors technique :
// std::vector int1;
for(int i=1;i<rows-1;i++){
    for(int j=1;j<cols-1;j++){
        //finding the neighbors average 
        float ne_avg = ( avg_arr[i][j-1] + avg_arr[i][j+1] + avg_arr[i-1][j] + avg_arr[i+1][j])/4.0;
        float diff =std::abs(ne_avg)- std::abs(avg_arr[i][j]);
        std::cout << "AVG of neighbors" << ne_avg<< "and the i,j position is "<<avg_arr[i][j]<<endl;
        logfile<<"Avg_diff neighbors: "<< ne_avg <<"and the i,j position is"<<avg_arr[i][j]<< std::endl;
        std::cout << "Difference between [" << i<<","<<j<< "] and neighbors is: " << diff << endl;
        logfile<<"Difference between [" << i<<","<<j<< "] and neighbors is: "<<diff<< std::endl;
            // Check if the difference is within the threshold
            if (std::abs(diff)>= thr1 && std::abs(diff) <= thr2) {
                // Print the difference
                std::cout<<"THRESHOLD ACHIEVED"<<endl;
                // std::cout <<" Difference between [" << i <<","<<j-1<< "] and [" << i<<","<<j << "] is: " << diff << endl;
                std::cout<<"achieved"<<endl;
            
                float x1=((i)*strip_size + (i-1)* strip_size)/2.0;

                float x2=((i+1)*strip_size + (i+2)* strip_size)/2.0;

                float j1c1=(j-1)*strip_size - y_strip;
                float j1c2=(j)*strip_size - y_strip;
                float j2c1=(j+1)*strip_size - y_strip ;   
                float j2c2=(j+2)*strip_size - y_strip ;             
                float y1=(j1c1+j1c2)/2.0;
                float y2=(j2c1+j2c2)/2.0;
                std::cout<<"x1: "<< (i)*strip_size <<"x2: " << x2 <<" y1: "<<y1 <<"y2: "<<y2<<endl;
                logfile<<"x1: "<< (i)*strip_size <<"x2: " << x2 <<" y1: "<<y1 <<"y2: "<<y2<<std::endl;
                // for Highlithing the cloud 
                int r= 255;
                int g=0;
                int b=0;
                std::cout<<"r: "<<r<<"g: "<<g<<" b: "<<b<<endl;

                for (size_t k = 0; k < cloud_filtered.size(); ++k) {
                    // Get the point's x coordinate
                    float x = cloud_filtered.points[k].x;
                    float y= cloud_filtered.points[k].y;
          
                     // Check if the x coordinate lies between x1 and x2
                    if (x >= x1 && x <= x2 && y>=y1 && y<=y2) {
                            cloud_filtered.points[k].r = r;
                            cloud_filtered.points[k].g = g;
                            cloud_filtered.points[k].b= b;
                        
                    // Push the colored point to the colored point cloud    
                    }
                }
            }
    }
}


// for publishing the filtered data
no_frame++;
sensor_msgs::PointCloud2 output_msg;
pcl::toROSMsg(cloud_filtered, output_msg);
output_msg.header = msg->header;
    // Publish the filtered point cloud

pub_.publish(output_msg);

    std::clock_t end = std::clock();

    // Compute the elapsed time
    double elapsed_time = static_cast<double>(end - start) / CLOCKS_PER_SEC;


    // clearing the array 
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j)
         array2D[i][j].clear();

    }


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
  std::string filename = "/home/lidar/Desktop/vishnu/output_txts/log_file_path_hole_" + std::to_string(thr1) + "_" + std::to_string(thr2)+ "_"+std::to_string(x) + ".txt";
  logfile.open(filename);
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



