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
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>




ros::Publisher cmd_pub;
ros::Publisher points_pub;

ros::Subscriber points_sub;
geometry_msgs ::Vector3 cmd;

pcl::PointCloud<pcl::PointXYZ> input_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud4(new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);

const double leafsize = 0.1;
Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);


//フィルタの宣言
  pcl::VoxelGrid<pcl::PointXYZ> voxe;//ダウンサンプリング
  pcl::PassThrough<pcl::PointXYZ> pass;//範囲指定
  //pcl::SACSegmentation<pcl::PointXYZ> seg;//平面検出
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extr_seg;//差分除去
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);


void rs_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
//pclに変換
  pcl::fromROSMsg(*input, input_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(input_cloud));


//ダウンサンプリング
  voxe.setInputCloud(input_cloud_ptr);
  voxe.setLeafSize(leafsize, leafsize, leafsize);
  voxe.filter(*filtered_cloud1);

//範囲指定
  pass.setInputCloud(filtered_cloud1);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-5, 5);
  pass.filter(*filtered_cloud2);
  pass.setInputCloud(filtered_cloud1);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(0, 5);
  pass.filter(*filtered_cloud1);
  pass.setInputCloud(filtered_cloud1);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1, 2);
  pass.filter(*filtered_cloud2);


  //法線検出
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(filtered_cloud2);

	// Estimate the normals.      法線を推定
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(filtered_cloud2);
	normalEstimation.setRadiusSearch(0.01);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

  //平面検出
  //std::cout << "a" << std::endl;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//検出するモデルのタイプを指定
  seg.setMethodType (pcl::SAC_RANSAC);//検出に使用する方法を指定
  seg.setDistanceThreshold (0.1);//RANSACの最小二乗法の許容誤差範囲
  seg.setMaxIterations(50);//最大計算回数
  seg.setAxis(axis);
  seg.setEpsAngle(20.0f*(M_PI/180.0f));
  seg.setInputNormals(normals);
  seg.setInputCloud (filtered_cloud2);
  seg.segment (*indices, *coefficients);
  extr_seg.setInputCloud(filtered_cloud2);
  extr_seg.setIndices(indices);
  extr_seg.setNegative(false);//trueの場合出力は検出された平面以外のデータ falseの場合は平面のデータ
  extr_seg.filter(*filtered_cloud3);

  pass.setInputCloud(filtered_cloud3);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-0.1, 0.1);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.1, 0.1);
  pass.filter(*filtered_cloud2);
  double dist_y = 0;
  for(int i=0;i < filtered_cloud2->points.size();i++){
    dist_y =dist_y +filtered_cloud2->points[i].y;
  }
  dist_y =dist_y/filtered_cloud2->points.size();
  std::cout << dist_y  << std::endl;



/*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> PointNormalEstimation;
  pcl::PointCloud<pcl::PointNormal>::Ptr pointNormal(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointXYZ searchPoint = filtered_cloud3->points[100];
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(filtered_cloud3);
  float radius = 1;
  std::vector<int> ind;
  std::vector<float> dist;
  tree->radiusSearch(searchPoint,radius,ind,dist);
  Eigen::Vector4f PlameParams;
  float nx;
  float ny;
  float nz;
  float cur;
  PointNormalEstimation.computePointNormal(*filtered_cloud3,ind,nx,ny,nz,cur);

  cmd.x=nx;
  cmd.y=ny;
  cmd.z=nz;
  
  std::cout << cmd.x << std::endl;
  std::cout << cmd.y << std::endl;
  std::cout << cmd.z << std::endl;*/
 
    
//pcl2に変換、パブリッシュ
  pcl::toROSMsg(*filtered_cloud3, *output_cloud);
  points_pub.publish(*output_cloud);
  //cmd_pub.publish(cmd);
}
  
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne_points_change");
  ros::NodeHandle nh;
  points_pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_change", 100);
  cmd_pub = nh.advertise<geometry_msgs::Vector3>("vector", 10);
  points_sub = nh.subscribe("/velodyne_points", 100, rs_callback);
  ros::spin();
}