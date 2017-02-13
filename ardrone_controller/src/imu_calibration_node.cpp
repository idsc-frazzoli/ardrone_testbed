#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <assert.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>



class hover_offset
{
public:
    geometry_msgs::Vector3 acceleration;//This is in the body frame
    unsigned int count;
    hover_offset():count(0)
    {
        acceleration.x=acceleration.y=acceleration.z=0.0;
    }

    void imu_msg_callback(sensor_msgs::Imu imu_data)//DO NOT PASS BY REFERENCE
    {
        acceleration.x=acceleration.x*count+imu_data.linear_acceleration.x;
        acceleration.y=acceleration.y*count+imu_data.linear_acceleration.y;
        acceleration.z=acceleration.z*count+imu_data.linear_acceleration.z;
        count++;
        acceleration.x=acceleration.x/count;
        acceleration.y=acceleration.y/count;
        acceleration.z=acceleration.z/count;
    }
};

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "imu_calibration");
    
    ros::NodeHandle n;
    hover_offset calibration;
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/ardrone/imu",100,&hover_offset::imu_msg_callback, &calibration);
    ros::Rate loop_rate(5);//the received msg is published at 200Hz.
    while (ros::ok())
    {
        printf("a_x: %f, a_y: %f, a_z: %f\n",calibration.acceleration.x,
                                             calibration.acceleration.y,
                                             calibration.acceleration.z);
        ros::spinOnce();
        
        loop_rate.sleep();
    }
    
    return 0;
}