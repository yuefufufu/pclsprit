#include <iostream>
#include <cmath>
#include <algorithm>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/progressive_morphological_filter.h>


ros::Publisher points_pub;
ros::Subscriber points_sub;

pcl::PointCloud<pcl::PointXYZ> input_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud4(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud5(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud6(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud7(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud8(new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);

const double leafsize = 0.1;


void rs_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
//pclに変換
  pcl::fromROSMsg(*input, input_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(input_cloud));

//フィルタの宣言
  pcl::VoxelGrid<pcl::PointXYZ> voxe;//ダウンサンプリング
  pcl::PassThrough<pcl::PointXYZ> pass;//範囲指定
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;//地面検出
  pcl::ExtractIndices<pcl::PointXYZ> extr_pmf;//差分除去
  pcl::SACSegmentation<pcl::PointXYZ> seg;//平面検出
  pcl::ExtractIndices<pcl::PointXYZ> extr_seg;//差分除去
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);

//ダウンサンプリング
  voxe.setInputCloud(input_cloud_ptr);
  voxe.setLeafSize(leafsize, leafsize, leafsize);
  voxe.filter(*filtered_cloud1);

//範囲指定
  pass.setInputCloud(filtered_cloud1);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-5, 5);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-5, 5);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1, 2);
  pass.filter(*filtered_cloud2);

  //平面検出
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);//検出するモデルのタイプを指定
  seg.setMethodType (pcl::SAC_RANSAC);//検出に使用する方法を指定
  seg.setDistanceThreshold (0.01);//RANSACの最小二乗法の許容誤差範囲
  seg.setInputCloud (filtered_cloud2);
  seg.segment (*indices, *coefficients);
  seg.setMaxIterations(100);
  seg.setProbability(0.95);
  extr_seg.setInputCloud(filtered_cloud2);
  extr_seg.setIndices(indices);
  extr_seg.setNegative(false);//trueの場合出力は検出された平面以外のデータ falseの場合は平面のデータ
  extr_seg.filter(*filtered_cloud3);


//pcl2に変換、パブリッシュ
  pcl::toROSMsg(*filtered_cloud3, *output_cloud);
  points_pub.publish(*output_cloud);
}
  
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne_points_change");
  ros::NodeHandle nh;
  points_pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_1", 100);
  points_sub = nh.subscribe("/velodyne_points", 100, rs_callback);
  ros::spin();
}