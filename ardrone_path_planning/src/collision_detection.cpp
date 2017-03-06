#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_detection");
    ros::start();
    
    ros::NodeHandle node_handle;
    ros::Subscriber point_cloud_sub = node_handle.subscribe("environment/point_cloud",2);
    
    while(ros::ok())
    {
        ros::spinOnce();
        
    }
    return 0;
}
