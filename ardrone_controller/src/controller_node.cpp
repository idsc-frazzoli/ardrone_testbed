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



class imu_odometry
{
public:
    geometry_msgs::Vector3 position,velocity,acceleration,acc_avg;//All in world frame alligned with magnetometer
    double t_now, t_last ,t_start;
    double dt;
    bool first_msg;
    unsigned int count;
    imu_odometry():count(500)
    {
        first_msg = true;
        position.x=position.y=position.z=0.0;
        velocity=acceleration=position;
        //Rough calibration for imu alignment
        acc_avg.x=1.55;
        acc_avg.y=1.31;
        acc_avg.z=9.23;
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
//         std::cout << "imu msg received at " << t_now-t_start << std::endl;
//         std::cout << "dt = " << dt << std::endl;
        
        acceleration = bodyToWorldFrame(imu_data.linear_acceleration,imu_data.orientation);
        
        
        
        //TODO calibrate accelerometer
        velocity.x+=dt*acceleration.x;
        velocity.y+=dt*acceleration.y;
        velocity.z+=dt*acceleration.z;
        position.x+=dt*velocity.x;
        position.y+=dt*velocity.y;
        position.z+=dt*velocity.z;
        
        //On average the acceleration is zero. Use to update imu calibration.
        acc_avg.x=acc_avg.x*count+imu_data.linear_acceleration.x;
        acc_avg.y=acc_avg.y*count+imu_data.linear_acceleration.y;
        acc_avg.z=acc_avg.z*count+imu_data.linear_acceleration.z;
        count++;
        acc_avg.x=acc_avg.x/count;
        acc_avg.y=acc_avg.y/count;
        acc_avg.z=acc_avg.z/count;
        
        
    }
    geometry_msgs::Vector3 bodyToWorldFrame(const geometry_msgs::Vector3& x_body, 
                                            const geometry_msgs::Quaternion& _q)
    {
    
        tf::Quaternion q;
        tf::quaternionMsgToTF(_q,q);
        tf::Quaternion z(x_body.x-acc_avg.x,x_body.y-acc_avg.y,x_body.z-acc_avg.z,0.0);// 
        z=q*z*(q.inverse());
//         prinft("Transforming : (%f,%f,%f)\n",x_body.x,x_body.y,x_body.z); 
        
        geometry_msgs::Vector3 x_world;
        x_world.x=z.x();x_world.y=z.y();x_world.z=z.z();
        return x_world;
    }
    
};

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "controller");
    
    ros::NodeHandle n;
    imu_odometry odometer;
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("/ardrone/imu",10,&imu_odometry::imu_msg_callback, &odometer);
    ros::Rate loop_rate(100);//the received msg is published at 200Hz.
    while (ros::ok())
    {
        printf("a_x: %f, a_y: %f, a_z: %f\n",odometer.acceleration.x,
                                             odometer.acceleration.y,
                                             odometer.acceleration.z);
        printf("x: %f, y: %f, z: %f\n",odometer.position.x,
                                       odometer.position.y,
                                       odometer.position.z);

        ros::spinOnce();
        
        loop_rate.sleep();
    }
    
    return 0;
}