#include <ros/ros.h>
#include "convex_clustering_code/convex_clustering_code.h"


using namespace std;

int main(int argc, char **argv)
{
    try
    {
        // ROS init
        ros::init(argc, argv, "convex_clustering");
        ros::NodeHandle nh;
        vector<float> param(3,0);
        param[0] = 2;   // deltaA_ 
        param[1] = 0.4; // deltaR_ 
        param[2] = 1.5; // deltaP_ 
        CVC Cluster(param);
        if(!Cluster.initialize())
        {
          exit(1);
        }
        else{
            while (nh.ok()) {
                cout << "Start" << endl;
                Cluster.spinNode();
            }
        }
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("Curved Clustering: Error occured: %s", e.what());
        exit(1);
    }
    return 0;
}