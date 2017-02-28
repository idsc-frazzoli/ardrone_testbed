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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>



class ardrone_odometry
{
public:
    geometry_msgs::Vector3 position,velocity,acceleration,acc_avg;//All in world frame alligned with magnetometer
    double t_now, t_last ,t_start;
    double dt;
    bool first_msg;
    unsigned int count;
    
    tf::TransformListener slam_listener;
    tf::Vector3 slam_pos;
    tf::Vector3 odom_pos;
    tf::Quaternion slam_quat;
    tf::Quaternion imu_quat;
    tf::Quaternion quat_cam_drone;
    tf::TransformBroadcaster br;
    tf::Transform level;
    
    
    
    
    ardrone_odometry():count(0)
    {
        first_msg = true;
        position.x=position.y=position.z=0.0;
        velocity=acceleration=position;
        //Rough calibration for imu alignment
//         acc_avg.x=1.55;
//         acc_avg.y=1.31;
//         acc_avg.z=9.23;
        t_last = t_now = t_start = NULL; 
    }

    //TODO account for attitude in estimate
    void imu_msg_callback(sensor_msgs::Imu imu_data)//DO NOT PASS BY REFERENCE
    {
        if(first_msg){
            first_msg = false;
            t_last=t_now=t_start=imu_data.header.stamp.toSec();
        }
        else{
            t_last=t_now;
            t_now=imu_data.header.stamp.toSec();
        }
    
        dt=t_now-t_last;
        tf::quaternionMsgToTF(imu_data.orientation,imu_quat);
        acceleration = bodyToWorldFrame(imu_data.linear_acceleration,imu_data.orientation);
    }
    
    geometry_msgs::Vector3 bodyToWorldFrame(const geometry_msgs::Vector3& x_body, 
                                            const geometry_msgs::Quaternion& _q)
    {
    
        tf::quaternionMsgToTF(_q,imu_quat);
        tf::Quaternion z(x_body.x,x_body.y,x_body.z,0.0);// 
        z=imu_quat*z*(imu_quat.inverse());
        
        geometry_msgs::Vector3 x_world;
        x_world.x=z.x();x_world.y=z.y();x_world.z=z.z();
        return x_world;
    }
    
    tf::Vector3 rotateVector(const tf::Vector3& vec, const tf::Quaternion& q)
    {
        
        tf::Quaternion z(vec.getX(),vec.getY(),vec.getZ(),0.0);// 
        z=q*z*(q.inverse());
        
        tf::Vector3 out(z.getX(),z.getY(),z.getZ());
        return out;
    }
    
    void get_slam_tf()
    {
        //try to get stuff from tf node
        try{
            tf::StampedTransform slam_tf;
            slam_listener.lookupTransform("/level", "/camera",ros::Time(0), slam_tf);
            slam_quat = slam_tf.getRotation();
            slam_pos = slam_tf.getOrigin();
        }
        catch(tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }
    }
    void broadcast_rel_tf()
    {
        tf::Quaternion r1 = tf::createQuaternionFromRPY(-M_PI/2.0,M_PI/2.0,0.0);
        tf::Quaternion r2 = tf::createQuaternionFromRPY(0,-M_PI/2,0);
        tf::Quaternion r3 = tf::createQuaternionFromRPY(0,0,-M_PI/2);
        
        quat_cam_drone = r3*r2*r1;
        
        tf::Vector3 origin(0.0,0.0,0.0);
        level.setOrigin(origin);
        level.setRotation(quat_cam_drone);
        odom_pos = tf::quatRotate(quat_cam_drone,slam_pos);
        br.sendTransform(tf::StampedTransform(level, ros::Time::now(), "/ardrone_base_link", "/level"));
        
    }
    
};

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "controller");
    ardrone_odometry odometer;
    ros::NodeHandle n;
    
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>( "/ardrone/imu", 10,  &ardrone_odometry::imu_msg_callback, &odometer);
    
    ros::Rate loop_rate(10);//the received msg is published at 200Hz.
    while (ros::ok())
    {    
        odometer.get_slam_tf();
        odometer.broadcast_rel_tf();
        ros::spinOnce();
        
        loop_rate.sleep();
    }
    
    return 0;
}