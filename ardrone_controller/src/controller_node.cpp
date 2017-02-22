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
    tf::Quaternion slam_quat;
    tf::Quaternion imu_quat;
    tf::Quaternion avg_quat;
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
            slam_listener.lookupTransform("level", "odom",ros::Time(0), slam_tf);
            slam_quat = slam_tf.getRotation();
            slam_pos = slam_tf.getOrigin();
        }
        catch(tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }
    }
    void update_quat_avg()
    {
        count++;
        tf::Quaternion zccw(1.0/sqrt(2.0),0.0,0.0,-1.0/sqrt(2.0));
        tf::Quaternion xcw(1.0/sqrt(2.0),1.0/sqrt(2.0),0.0,0.0);
        avg_quat = xcw*zccw;
//         avg_quat=imu_quat*(slam_quat*right).inverse();
        tf::Vector3 origin(0.0,0.0,0.0);
        level.setOrigin(origin);
        level.setRotation(avg_quat);
        br.sendTransform(tf::StampedTransform(level, ros::Time::now(), "world", "level"));
        
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

//         printf("a_x: %f, a_y: %f, a_z: %f\n",odometer.acceleration.x,
//                                              odometer.acceleration.y,
//                                              odometer.acceleration.z);
        printf("x: %f, y: %f, z: %f\n",odometer.slam_pos.getX(),
                                       odometer.slam_pos.getY(),
                                       odometer.slam_pos.getZ());
        printf("slam_w: %f, slam_x: %f ,slam_y: %f, slam_z: %f \n",odometer.slam_quat.getW(),
                                            odometer.slam_quat.getX(),
                                            odometer.slam_quat.getY(),
                                            odometer.slam_quat.getZ());
        
        printf(" imu_w: %f,  imu_x: %f,  imu_y: %f,  imu_z: %f \n",odometer.imu_quat.getW(),
               odometer.imu_quat.getX(),
               odometer.imu_quat.getY(),
               odometer.imu_quat.getZ());
        
        printf("diff_x: %f, diff_x: %f, diff_y: %f, diff_z: %f \n",odometer.avg_quat.getW(),
               odometer.avg_quat.getX(),
               odometer.avg_quat.getY(),
               odometer.avg_quat.getZ());
        
        odometer.get_slam_tf();
        odometer.update_quat_avg();
        ros::spinOnce();
        
        loop_rate.sleep();
    }
    
    return 0;
}