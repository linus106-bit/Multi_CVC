#include <ros/ros.h>
#include "convex_clustering_code/convex_clustering_code.h"

using namespace std;




void centroidCallback(const convex_clustering_code::centroid_list& centroid_list){
    cout << centroid_list.group.size() << endl;
    for (int i = 0 ; i < centroid_list.group.size(); i++)
    {
        for (int j = 0 ; j < centroid_list.group.size() ; j++)
        {
            cout << "a : " << centroid_list.group.size() << endl;
        }
    }
}


int main(int argc, char **argv)
{
    try
    {
        // ROS init
        ros::init(argc, argv, "convex_clustering_merge");
        ros::NodeHandle nh_;
        ros::Subscriber centroid_sub;
        cout << "START" << endl;
        centroid_sub = nh_.subscribe("input_centroid", 1, centroidCallback);
        ros::spin();
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("Curved Clustering: Error occured: %s", e.what());
        exit(1);
    }
    return 0;
}