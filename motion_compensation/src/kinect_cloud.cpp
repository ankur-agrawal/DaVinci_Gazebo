#include "kinect_cloud.h"
#include <typeinfo>

MatrixXd kinect_cloud::IK_PSM(MatrixXd pos)
{
  MatrixXd JointAngle(1,3);
  if (pos(0,0)==0)
  {
    JointAngle(0,0)=0;
  }

  else
  {
    JointAngle(0,0)=atan2(pos(0,0),-pos(1,0));
  }
  if (M_PI-abs(JointAngle(0,0))<0.5)
  {
    JointAngle(0,0)=(M_PI-abs(JointAngle(0,0)))*JointAngle(0,0)/abs(JointAngle(0,0));
  }
  JointAngle(0,1)=atan2(-(pos(2,0)+0.369),sqrt(pow(pos(0,0),2)+pow(pos(1,0),2)));
  if (JointAngle(0,1)<-M_PI/2)
  {
    JointAngle(0,1)=M_PI+JointAngle(0,1);
  }
  else if (JointAngle(0,1)>M_PI/2)
  {
    JointAngle(0,1)=-M_PI+JointAngle(0,1);
  }
  JointAngle(0,2)=sqrt(pow(pos(0,0),2)+pow(pos(1,0),2)+pow((pos(2,0)+0.369),2));

  return JointAngle;

}


void kinect_cloud::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::common::IntensityFieldAccessor<pcl::PointXYZRGB> c;
  sensor_msgs::PointCloud2 output;
  float intensity;

  pcl::fromROSMsg(*input, *cloud);

  for(size_t i = 0; i < cloud->points.size(); i++)
  {
    if (isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z))
    {
      continue;
    }
    else
    {
      cloud_filtered->insert(cloud_filtered->end(), cloud->points[i]);
      c.get(cloud->points[i], intensity);
      // std::cout << intensity << '\n';
    }
  }
  for (size_t i=0; i<cloud_filtered->points.size();i++)
  {
    // c.get(cloud_filtered->points[i], intensity);
    if (cloud_filtered->points[i].r>40 && cloud_filtered->points[i].g<100 && cloud_filtered->points[i].b<100)
    // if (intensity>50 && intensity<60)
    {
      cloud_f->push_back(cloud_filtered->points[i]);
    }
  }
  // std::cout << cloud_f->points.size() << '\n';
  // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  // seg.setOptimizeCoefficients (true);
  // seg.setModelType (pcl::SACMODEL_PLANE);
  // seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setMaxIterations (100);
  // seg.setDistanceThreshold (0.02);
  //
  // int i=0, nr_points = (int) cloud_filtered->points.size ();
  // while (cloud_filtered->points.size () > 0.3 * nr_points)
  // {
  //   // Segment the largest planar component from the remaining cloud
  //   seg.setInputCloud (cloud_filtered);
  //   seg.segment (*inliers, *coefficients);
  //   if (inliers->indices.size () == 0)
  //   {
  //     // std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  //     break;
  //   }
  //
  //   // Extract the planar inliers from the input cloud
  //   pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  //   extract.setInputCloud (cloud_filtered);
  //   extract.setIndices (inliers);
  //   extract.setNegative (false);
  //
  //   // Get the points associated with the planar surface
  //   extract.filter (*cloud_plane);
  //   // std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  //
  //   // Remove the planar inliers, extract the rest
  //   extract.setNegative (true);
  //   extract.filter (*cloud_f);
  //   *cloud_filtered = *cloud_f;
  // }
  //
  // // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud_f);
  //
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.01); // 2cm
  ec.setMinClusterSize (2000);
  ec.setMaxClusterSize (70000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_f);
  ec.extract (cluster_indices);
  //
  // std::cout << cluster_indices.size() << '\n';
  size_t max=0;
  int j = 0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB>);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      cloud_cluster->points.push_back (cloud_f->points[*pit]); //*
    }
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    j++;
    if (cloud_cluster->points.size()> max)
    {
        *cloud_output=*cloud_cluster;
        max=cloud_cluster->points.size();
    }
  }
  pcl::PointXYZRGB min=cloud_output->points[0], x_min=cloud_output->points[0], y_min=cloud_output->points[0], z_min=cloud_output->points[0];
  if (cloud_output->points.size() !=0)
  {

    for (size_t i=0; i < cloud_output->points.size(); i++)
    {
        if (cloud_output->points[i].x < min.x)
        {
          x_min=cloud_output->points[i];
          min.x=cloud_output->points[i].x;
        }
        if (cloud_output->points[i].y < min.y)
        {
          y_min=cloud_output->points[i];
          min.y=cloud_output->points[i].y;
        }
        if (cloud_output->points[i].z < min.z)
        {
          z_min=cloud_output->points[i];
          min.z=cloud_output->points[i].z;
        }
    }
  }
  // std::cout << x_min << '\t' << y_min << '\t' << z_min << '\n';
  // std::cout << y_min << '\n';

  MatrixXd JointAngles(1,3);
  MatrixXd pos(3,1);

  pos(0,0)=y_min.x;
  pos(1,0)=y_min.y;
  pos(2,0)=y_min.z;

  JointAngles=kinect_cloud::IK_PSM(pos);

  // std::cout << JointAngles << '\n';
  sensor_msgs::JointState joint_state;

  joint_state.header.stamp = ros::Time::now();
  joint_state.header.frame_id = "world";
  joint_state.name.resize(7);
  joint_state.position.resize(7);
  joint_state.name[0] ="one_outer_yaw_joint";
  joint_state.position[0] = 1;
  joint_state.name[1] ="one_outer_pitch_joint_1";
  joint_state.position[1] = 1;
  joint_state.name[2] ="one_outer_insertion_joint";
  joint_state.position[2] = 0;
  // joint_state.name[1] ="tilt";
  // joint_state.position[1] = tilt;
  // joint_state.name[2] ="periscope";
  // joint_state.position[2] = height;


  joint_pub.publish(joint_state);

  tf::Transform transform;
  static tf::TransformBroadcaster br;
  tf::Quaternion q;

  q.setRPY(0, M_PI, 0);

  transform.setOrigin( tf::Vector3(-0.4, 0.0, 1.25) );
  transform.setRotation( q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_depth_optical_frame"));

  pcl::toROSMsg(*cloud_output, output);
  output.header.frame_id = "camera_depth_optical_frame";
  // output.header.frame_id= "transform";
  pt_cloud_pub.publish(output);

}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "kinect_cloud");
  ros::NodeHandle n;
  kinect_cloud obj(n);
  ros::spin();
}
