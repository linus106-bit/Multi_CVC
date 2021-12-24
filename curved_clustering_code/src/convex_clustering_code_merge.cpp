#include <ros/ros.h>
#include "curved_clustering_code/curved_clustering_code.h"

using namespace std;

float calculate_distance(pcl::PointXYZ centroid_1, pcl::PointXYZ centroid_2)
{
    return std::sqrt((centroid_1.x + centroid_2.x-2) * (centroid_1.x + centroid_2.x-2)
                    + (centroid_1.y + centroid_2.y-2) * (centroid_1.y + centroid_2.y-2)
                    + (centroid_1.z - centroid_2.z) * (centroid_1.z - centroid_2.z));
}

void centroidCallback_1(const curved_clustering_code::centroid_list& centroid_list){
    centroid_vector_1.clear();
    for (int i = 0 ; i < centroid_list.group.size(); i++)
    {
        pcl::PointXYZ centroid_vector;
        centroid_vector.x = (centroid_list.group[i].x);
        centroid_vector.y = (centroid_list.group[i].y);
        centroid_vector.z = (centroid_list.group[i].z);
        centroid_vector_1.push_back(centroid_vector);
    }
}

void centroidCallback_2(const curved_clustering_code::centroid_list& centroid_list){
    centroid_vector_2.clear();
    for (int i = 0 ; i < centroid_list.group.size(); i++)
    {
        pcl::PointXYZ centroid_vector;
        centroid_vector.x = (centroid_list.group[i].x);
        centroid_vector.y = (centroid_list.group[i].y);
        centroid_vector.z = (centroid_list.group[i].z);
        centroid_vector_2.push_back(centroid_vector);
    }
    cout << "size 1 : " << centroid_vector_1.size() << endl;
    cout << "size 2 : " << centroid_vector_2.size() << endl;
    std::vector<pcl::PointXYZ>* centroids(new std::vector<pcl::PointXYZ>);
    MergeClusters(centroid_vector_1,centroid_vector_2,*centroids);
    PublishMarkers(*centroids);
}



void MergeClusters(std::vector<pcl::PointXYZ> centroids_1, std::vector<pcl::PointXYZ> centroids_2, std::vector<pcl::PointXYZ> &centroids){
    centroids = centroids_1;
    for (int i = 0 ; i < centroids_2.size() ; i++)
    {
        for (int j = 0 ; j < centroids_1.size() ; j++){
            float dist = calculate_distance(centroids_1[j],centroids_2[i]);
            cout << "--------- " << i << "," << j << " ---------" << endl;
            cout << "dist: " << dist << endl;
            if (dist < 0.5) {
                break;
            }
            else if (j == centroids_1.size()-1){
                pcl::PointXYZ centroid_2_moved;
                centroid_2_moved.x = -centroids_2[i].x+2;
                centroid_2_moved.y = -centroids_2[i].y+2;
                centroid_2_moved.z = centroids_2[i].z;
                centroids.push_back(centroid_2_moved);
                cout << "Plus" << endl;
            }
            cout << "total : " << centroids.size() << endl;
        }
    }
}




void PublishMarkers(std::vector<pcl::PointXYZ> &centroids){ // Rviz에 Cluster ID 별로 visualization
    Point p; 
    std::vector<Point> vec_point;
    for (int i = 0 ; i < centroids.size() ; i++){
        p.x = centroids[i].x;
        p.y = centroids[i].y;
        p.z = centroids[i].z;
        vec_point.push_back(p);
    }
    std::cout << "num of Centroids : " << vec_point.size() << std::endl;
    visualization_msgs::MarkerArray node_arr;
    int cnt = 0;
    for (size_t i = 0; i < vec_point.size(); i++){
        Point o_node = vec_point[i];

        visualization_msgs::Marker node; 
        node.header.frame_id = "/velodyne_front_base_link"; 
        // map frame 기준 
        node.header.stamp = ros::Time::now(); 
        node.type = visualization_msgs::Marker::SPHERE; 
        node.id = i+1; 
        node.action = visualization_msgs::Marker::ADD; 
        node.pose.orientation.w = 1.0; 
        node.pose.position.x = o_node.x; //노드의 x 좌표 
        node.pose.position.y = o_node.y; //노드의 y 좌표 
        // Points are green 
        node.color.g = 0.5;
        node.color.r = 0; 
        node.color.a = 1.0; 
        node.scale.x = 1.0; 
        node.scale.y = 1.0; 
        node_arr.markers.push_back(node); 

        visualization_msgs::Marker node_name; 
        node_name.header.frame_id = "/velodyne_front_base_link"; // frame 기준 
        node_name.header.stamp = ros::Time::now(); 
        node_name.text = std::to_string(i+1); 
        node_name.color.a = 1.0; 
        node_name.scale.z = 1.0; 
        node_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        node_name.id = i + 100; 
        node_name.action = visualization_msgs::Marker::ADD; 
        node_name.pose.orientation.w = 1.0; 
        node_name.pose.position.x = o_node.x; //노드의 x 좌표 
        node_name.pose.position.y = o_node.y; //노드의 y 좌표 
        node_arr.markers.push_back(node_name);
    }
    markers_pub.publish(node_arr);
}


int main(int argc, char **argv)
{
    try
    {
        // ROS init
        ros::init(argc, argv, "curved_clustering_merge");
        ros::NodeHandle nh_;
        ros::Subscriber centroid_sub_1;
        ros::Subscriber centroid_sub_2;
        markers_pub = nh_.advertise<visualization_msgs::MarkerArray>("output_marker", 10);
        cout << "START" << endl;
        centroid_sub_1 = nh_.subscribe("centroids_1", 10, centroidCallback_1);
        centroid_sub_2 = nh_.subscribe("centroids_2", 10, centroidCallback_2);
        ros::spin();
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("Curved Clustering: Error occured: %s", e.what());
        exit(1);
    }
    return 0;
}