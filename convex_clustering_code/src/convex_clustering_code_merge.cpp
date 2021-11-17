#include <ros/ros.h>
#include "convex_clustering_code/convex_clustering_code.h"

using namespace std;


int main(int argc, char **argv)
{
    try
    {
        // ROS init
        ros::init(argc, argv, "convex_clustering_merge");
        ros::NodeHandle nh_;
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("Curved Clustering: Error occured: %s", e.what());
        exit(1);
    }
    return 0;
}