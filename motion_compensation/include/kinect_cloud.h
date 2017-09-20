#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <math.h>
#include <iomanip>
#include <Eigen/Dense>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/common/intensity.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

using namespace Eigen;
class kinect_cloud{
private:
  ros::Subscriber pt_cloud_sub;
  ros::Publisher pt_cloud_pub;
  ros::Publisher joint_pub;
public:
  kinect_cloud(ros::NodeHandle n)
  {
    pt_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/segmented/depth_registered/points",1000);
    pt_cloud_sub = n.subscribe("/camera/depth_registered/points",10, &kinect_cloud::cloudCB,this);
    joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);
  }
void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);
MatrixXd IK_PSM(MatrixXd pos);

};
