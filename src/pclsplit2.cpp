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

ros::Publisher points_pub[10];
ros::Subscriber points_sub;

pcl::PointCloud<pcl::PointXYZ> input_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud4(new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

const double leafsize = 0.1;

//フィルタの宣言
  pcl::VoxelGrid<pcl::PointXYZ> voxe;//ダウンサンプリング
  pcl::PassThrough<pcl::PointXYZ> pass;//範囲指定
  pcl::SACSegmentation<pcl::PointXYZ> seg;//平面検出
  pcl::ExtractIndices<pcl::PointXYZ> extr_seg;//差分
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;//クラスタリング
  pcl::ExtractIndices<pcl::PointXYZ> extr_ece;//差分


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
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-5, 5);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1, 2);
  pass.filter(*filtered_cloud2);
  
//クラスタリング
	/*kd-treeクラスを宣言*/
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	/*探索する点群をinput*/
	tree->setInputCloud(filtered_cloud2);
	/*クラスタリング後のインデックスが格納されるベクトル*/
	std::vector<pcl::PointIndices> cluster_indices;
	/*距離の閾値を設定*/
	ece.setClusterTolerance(0.15);
	/*各クラスタのメンバの最小数を設定*/
	ece.setMinClusterSize(100);
	/*各クラスタのメンバの最大数を設定*/
	ece.setMaxClusterSize(5000);
	/*探索方法を設定*/
	ece.setSearchMethod(tree);
	/*クラスリング対象の点群をinput*/
	ece.setInputCloud(filtered_cloud2);
	/*クラスリング実行*/
	ece.extract(cluster_indices);



	extr_ece.setInputCloud(filtered_cloud2);
	extr_ece.setNegative(false);
  clusters.clear();
	for(int i=0;i<cluster_indices.size();i++){
		/*extract*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
		*tmp_clustered_indices = cluster_indices[i];
		extr_ece.setIndices(tmp_clustered_indices);
		extr_ece.filter(*filtered_cloud3);
    clusters.push_back(filtered_cloud3);
	}

  for(int i=0;i<3;i++){
  	pcl::toROSMsg(*clusters[i], *output_cloud);
    points_pub[i].publish(*output_cloud);
  }



}
  
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne_points_change");
  ros::NodeHandle nh;
  points_pub[0] = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points0", 100);
  points_pub[1] = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points1", 100);
  points_pub[2] = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points2", 100);
  points_sub = nh.subscribe("/velodyne_points", 100, rs_callback);
  ros::spin();
}