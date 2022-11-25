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


ros::Publisher vec_pub;
ros::Publisher twi_pub;
ros::Publisher points_pub;
ros::Subscriber points_sub;

geometry_msgs::Vector3 cmd;
geometry_msgs::Twist cmd_t;

int flag = 1;
float def_ang = 0;
float dist_k = 0;

pcl::PointCloud<pcl::PointXYZ> input_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud4(new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);

const double leafsize = 0.1;
Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,0.0);


//フィルタの宣言
  pcl::VoxelGrid<pcl::PointXYZ> voxe;//ダウンサンプリング
  pcl::PassThrough<pcl::PointXYZ> pass;//範囲指定
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;//平面検出
  pcl::ExtractIndices<pcl::PointXYZ> extr_seg;//差分除去
  pcl::PointIndices::Ptr indices(new pcl::PointIndices);


void rs_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	if(flag == 1){
		//std::cout << "1" << std::endl;
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
  		pass.setFilterLimits(0.2, 5);
  		pass.filter(*filtered_cloud2);
  		pass.setInputCloud(filtered_cloud1);
  		pass.setFilterFieldName("y");
  		pass.setFilterLimits(-5, 5);
  		pass.filter(*filtered_cloud1);
  		pass.setInputCloud(filtered_cloud1);
  		pass.setFilterFieldName("z");
  		pass.setFilterLimits(-1, 2);
  		pass.filter(*filtered_cloud2);
		//全点群の法線を推定
  		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		kdtree->setInputCloud(filtered_cloud2);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(filtered_cloud2);
		normalEstimation.setRadiusSearch(0.01);
		normalEstimation.setSearchMethod(kdtree);
		normalEstimation.compute(*normals);
		//平面検出
  		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  		seg.setOptimizeCoefficients (true);
  		seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//検出するモデルのタイプを指定
  		seg.setMethodType (pcl::SAC_RANSAC);//検出に使用する方法を指定
  		seg.setDistanceThreshold (0.1);//RANSACの最小二乗法の許容誤差範囲
  		seg.setMaxIterations(50);//最大計算回数
		Eigen::Vector3f axis = Eigen::Vector3f(1.0,0.0,0.0);
  		seg.setAxis(axis);
  		seg.setEpsAngle(60.0f*(M_PI/180.0f));
  		seg.setInputNormals(normals);
  		seg.setInputCloud (filtered_cloud2);
  		seg.segment (*indices, *coefficients);
		if(indices->indices.size() < 3){
			return ;
		}
  		extr_seg.setInputCloud(filtered_cloud2);
  		extr_seg.setIndices(indices);
  		extr_seg.setNegative(false);//trueの場合出力は検出された平面以外のデータ falseの場合は平面のデータ
  		extr_seg.filter(*filtered_cloud3);
		//検出平面の法線ベクトルを推定
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
		float cur;
		PointNormalEstimation.computePointNormal(*filtered_cloud3,ind,PlameParams,cur);
		if(PlameParams(0) < 0 && PlameParams(1) < 0){
			cmd.x= -1 * PlameParams(0);//xの法線ベクトル
  			cmd.y= -1 * PlameParams(1);//yの法線ベクトル
  			cmd.z=PlameParams(2);//zの法線ベクトル
		}
		else{
			cmd.x=PlameParams(0);//xの法線ベクトル
  			cmd.y=PlameParams(1);//yの法線ベクトル
  			cmd.z=PlameParams(2);//zの法線ベクトル
		}
		//検出平面とロボットのなす角を計算
		if(cmd.y>=0){
			def_ang = 90 - (180*(acos(cmd.x/sqrt((cmd.x*cmd.x)+(cmd.y*cmd.y))))/M_PI);
		}
		else{
			def_ang = 90 + (180*(acos(cmd.x/sqrt((cmd.x*cmd.x)+(cmd.y*cmd.y))))/M_PI);
		}
		//検出平面との最短距離を推定
  		dist_k = -1 * PlameParams(3)/sqrt((cmd.x*cmd.x)+(cmd.y*cmd.y)+(cmd.z*cmd.z));
		//std::cout << cmd.x << " " << cmd.y  << " "<< cmd.z << std::endl;
		std::cout << def_ang << "[°]   " << dist_k  << "[m]"<< std::endl;
		if(def_ang < 45){
			flag = 2;
			std::cout << "flag change [ 1 >> 2 ]" << std::endl;
		}
		else if(def_ang > 135){
			flag = 4;
			std::cout << "flag change [ 1 >> 4 ]" << std::endl;
		}
		else{
			std::cout << "flag [ 1 ]" << std::endl;
		}
		cmd_t.linear.x = 0;
		cmd_t.linear.y = 0;
		cmd_t.angular.z = 0;
	}
	else if(flag == 2){
		//std::cout << "1" << std::endl;
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
  		pass.setFilterLimits(0.2, 5);
  		pass.filter(*filtered_cloud1);
  		pass.setInputCloud(filtered_cloud1);
  		pass.setFilterFieldName("z");
  		pass.setFilterLimits(-1, 2);
  		pass.filter(*filtered_cloud2);
		//全点群の法線を推定
  		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		kdtree->setInputCloud(filtered_cloud2);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(filtered_cloud2);
		normalEstimation.setRadiusSearch(0.01);
		normalEstimation.setSearchMethod(kdtree);
		normalEstimation.compute(*normals);
		//平面検出
  		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  		seg.setOptimizeCoefficients (true);
  		seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);//検出するモデルのタイプを指定
  		seg.setMethodType (pcl::SAC_RANSAC);//検出に使用する方法を指定
  		seg.setDistanceThreshold (0.1);//RANSACの最小二乗法の許容誤差範囲
  		seg.setMaxIterations(50);//最大計算回数
		Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);
  		seg.setAxis(axis);
  		seg.setEpsAngle(60.0f*(M_PI/180.0f));
  		seg.setInputNormals(normals);
  		seg.setInputCloud (filtered_cloud2);
  		seg.segment (*indices, *coefficients);
		if(indices->indices.size() < 3){
			return ;
		}
  		extr_seg.setInputCloud(filtered_cloud2);
  		extr_seg.setIndices(indices);
  		extr_seg.setNegative(false);//trueの場合出力は検出された平面以外のデータ falseの場合は平面のデータ
  		extr_seg.filter(*filtered_cloud3);
		//検出平面の法線ベクトルを推定
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
		float cur;
		PointNormalEstimation.computePointNormal(*filtered_cloud3,ind,PlameParams,cur);
		if(PlameParams(0) > 0 && PlameParams(1) > 0){
			cmd.x=PlameParams(0);//xの法線ベクトル
  			cmd.y=PlameParams(1);//yの法線ベクトル
  			cmd.z=PlameParams(2);//zの法線ベクトル
		}
		else{
			cmd.x= -1 * PlameParams(0);//xの法線ベクトル
  			cmd.y= -1 * PlameParams(1);//yの法線ベクトル
  			cmd.z=PlameParams(2);//zの法線ベクトル
		}
		//検出平面とロボットのなす角を計算
		if(cmd.y>=0){
			def_ang = 90 - (180*(acos(cmd.x/sqrt((cmd.x*cmd.x)+(cmd.y*cmd.y))))/M_PI);
		}
		else{
			def_ang = 90 + (180*(acos(cmd.x/sqrt((cmd.x*cmd.x)+(cmd.y*cmd.y))))/M_PI);
		}
		//検出平面との最短距離を推定
  		dist_k = PlameParams(3)/sqrt((cmd.x*cmd.x)+(cmd.y*cmd.y)+(cmd.z*cmd.z));
		std::cout << cmd.x << " " << cmd.y  << " "<< cmd.z << std::endl;
		std::cout << def_ang << "[°]   " << dist_k  << "[m]"<< std::endl;

		pass.setInputCloud(filtered_cloud3);
  		pass.setFilterFieldName("x");
  		pass.setFilterLimits(0.1, 0.8);
  		pass.filter(*filtered_cloud4);
  		pass.setInputCloud(filtered_cloud4);
  		pass.setFilterFieldName("y");
  		pass.setFilterLimits(0, 1);
  		pass.filter(*filtered_cloud4);
  		pass.setInputCloud(filtered_cloud4);
  		pass.setFilterFieldName("z");
  		pass.setFilterLimits(0, 0.2);
  		pass.filter(*filtered_cloud4);

		if(filtered_cloud4->points.size() == 0){
			flag = 5;
			std::cout << "flag change [ 2 >> 5 ]" << std::endl;
		}
		else if(def_ang > 45){
			flag = 1;
			std::cout << "flag change [ 2 >> 1 ]" << std::endl;
		}
		else if(def_ang < -45){
			flag = 3;
			std::cout << "flag change [ 2 >> 3 ]" << std::endl;
		}
		else{
			std::cout << "flag [ 2 ]" << std::endl;
		}

		cmd_t.linear.x = 0;
		cmd_t.linear.y = 0;
		cmd_t.angular.z = 0;
	}
	else if(flag = 5){
		std::cout << "flag [ 5 ]" << std::endl;
		cmd_t.linear.x = 0;
		cmd_t.linear.y = 0;
		cmd_t.angular.z = 0;
	}
	else{
		flag = 1;
	}
	//pcl2に変換、パブリッシュ
  	pcl::toROSMsg(*filtered_cloud3, *output_cloud);
  	points_pub.publish(*output_cloud);
  	vec_pub.publish(cmd);
	twi_pub.publish(cmd_t);	
}
  
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "velodyne_points_change");
  ros::NodeHandle nh;
  points_pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_change", 100);
  vec_pub = nh.advertise<geometry_msgs::Vector3>("vector", 10);
  twi_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  points_sub = nh.subscribe("/velodyne_points", 100, rs_callback);
  ros::spin();
}