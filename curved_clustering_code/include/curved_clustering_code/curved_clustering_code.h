#ifndef CURVED_CLUSTERING_CODE_H
#define CURVED_CLUSTERING_CODE_H

//ROS
#include "pcl_ros/point_cloud.h"



// ROS msgs
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <curved_clustering_code/centroid.h>
#include <curved_clustering_code/centroid_list.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

//C++
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/Core>


using namespace Eigen;


template<typename T> 
std::string toString(const T& t) {
	std::ostringstream oss;
	oss << t;
	return oss.str();
}

struct Point { 
    float x;
    float y; 
    float z;
};

struct PointAPR{
   float azimuth;
   float polar_angle;
   float range;
};

struct Voxel{
   bool haspoint = false;
   int cluster = -1;
   std::vector<int> index;
};

// curved-voxel cluster merge
void centroidCallback_1(const curved_clustering_code::centroid_list& centroid_list);
void centroidCallback_2(const curved_clustering_code::centroid_list& centroid_list);
float calculate_distance(pcl::PointXYZ centroid_1, int i, pcl::PointXYZ centroid_2, int j);
void PublishMarkers(std::vector<pcl::PointXYZ> &centroids);
void MergeClusters(std::vector<pcl::PointXYZ> centroids_1, std::vector<pcl::PointXYZ> centroids_2, std::vector<pcl::PointXYZ> &centroids);
std::vector<pcl::PointXYZ> centroid_vector_1;
std::vector<pcl::PointXYZ> centroid_vector_2;
ros::Publisher markers_pub;


class CVC{
	public:
		CVC();
		CVC(std::vector<float>& param){
			if(param.size() != 3){
				printf("Param number is not correct!");
				std::abort();		
			}
			for(int i=0; i<param.size(); ++i){
				deltaA_ = param[0];
				deltaR_ = param[1];
				deltaP_ = param[2];
			}
		}

		~CVC(){}
		bool initialize();
		void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
		void spinNode();
		void calculateAPR(const pcl::PointCloud<pcl::PointXYZ>& cloud_IN, std::vector<PointAPR>& vapr);
		float euc_dist(Vector3d P1, Vector3d P2);
		void build_hash_table(const std::vector<PointAPR>& vapr, std::unordered_map<int, Voxel> &map_out);
		void find_neighbors(int polar, int range, int azimuth, std::vector<int>& neighborindex);
		bool most_frequent_value(std::vector<int> values, std::vector<int> &cluster_index);
		bool cluster_result(std::vector<int> cluster_indices, std::vector<std::vector<pcl::PointXYZ>> &cluster_results, const pcl::PointCloud<pcl::PointXYZ>& cloud_IN, std::vector<pcl::PointXYZ> &centroids) ;
		void mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2);
		std::vector<int> cluster(std::unordered_map<int, Voxel> &map_in,const std::vector<PointAPR>& vapr);
		void PublishMarker(std::vector<pcl::PointXYZ> &centroids);
		void process();
		std::vector<pcl::PointXYZ> getCentroid(std::vector<std::vector<pcl::PointXYZ>> cluster_results);
		void PublishCentroid(std::vector<pcl::PointXYZ> &centroids, std::vector<std::vector<pcl::PointXYZ>> &cluster_results);
		ros::Subscriber input_points;
		ros::Publisher marker_pub;
		ros::Publisher centroid_pub;

	private:
		ros::NodeHandle nh_;
		double time_init;
		float frequency = 10;
		float deltaA_ = 2;
		float deltaR_ = 0.35;
		float deltaP_ = 1.2;
		float min_range_ = std::numeric_limits<float>::max();
		float max_range_ = std::numeric_limits<float>::min();
		float min_azimuth_ = -24.8 * M_PI/180;
		float max_azimuth_ = 2 * M_PI/180;
		int length_ = 0;
		int width_  = 0;
		int height_ = 0;

		float RVIZ_COLOR_GREEN;
		float RVIZ_COLOR_RED;
		std::string VELODYNE_BASE;
};



#endif


