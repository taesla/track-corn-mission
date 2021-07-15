#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

ros::Publisher cloud_pub;

void cloud_callback (const sensor_msgs::PointCloud2& cloud_msg)
{
  //make container for converted cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  *ptr_filtered = cloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);


  //other pointclouds for filtering
//  pcl::PCLPointCloud2::Ptr pass_z (new pcl::PCLPointCloud2 ());
//  pcl::PCLPointCloud2::Ptr pass_zx (new pcl::PCLPointCloud2 ());

  sensor_msgs::PointCloud2 cloud_out;

  //Passthrough filter the points in z
	pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud(cloud);
  // pass.setFilterFieldName("z");
  // pass.setFilterLimits(-1,1);
  // pass.filter(*cloud);

//   Passthough in y
  pass.setInputCloud(ptr_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.6,0.6);
  pass.setFilterLimitsNegative(true);
  pass.filter(*cloud_a);

  //passthrough filter the points in x
  pass.setInputCloud(ptr_filtered);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-1,1);
  pass.setFilterLimitsNegative(true);
  pass.filter(*cloud_b);

  *cloud_filtered = *cloud_a + *cloud_b;
  
  // pass.setInputCloud(cloud);
  // pass.setFilterFieldName("x");
  // pass.setFilterLimits(-5,100);
  // pass.setFilterLimitsNegative(false);
  // pass.filter(*cloud);
  //Voxel Grid Filter the points
//   pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
//   vox.setInputCloud(cloud);
//   vox.setLeafSize(0.01,0.01,0.01);
//   vox.filter(*cloud_filtered);

//  pcl_pub.publish(cloud_filtered);

  sensor_msgs::PointCloud2 cloudmsg;
  pcl::toROSMsg(*cloud_filtered, cloudmsg);
  //Publisher
  // pcl_conversions::fromPCL(*cloud, cloud_filtered);
  cloud_pub.publish(cloudmsg);
  //cloud_pub.publish(*cloud_filtered);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_ROI");
  ros::NodeHandle nh;

  ros::Rate loop_rate(30);

  ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, cloud_callback);

  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered", 1);
  loop_rate.sleep();

  ros::spin();
}