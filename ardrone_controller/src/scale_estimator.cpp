#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <std_msgs/Float32.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath//Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

class ScaleEstimator
{
public:
  float scale = 1; // has units m^-1 
  float sxx=0, syy=0, sxy=0, orb_noise=1, imu_noise=1; // noise params must be tuned
    
  tf2::Vector3 orb_displacement, orb_meas1, orb_meas2, orb_meas3; // past 1 --> 2 --> 3 present
  ros::Time T1, T2, T3;
  
  tf2::Vector3 curr_acc, orb_acc;
  tf2::Quaternion orb_orientation;
  
  int msg_n = 0;
  
  string imu_frame_id, orb_frame_id;
 
  void imu_callback(sensor_msgs::Imu msg)
  {

    tf2::Vector3 curr_acc_meas = tf2::Vector3(tf2Scalar(msg.linear_acceleration.x), 
				             tf2Scalar(msg.linear_acceleration.y), 
					     tf2Scalar(msg.linear_acceleration.z));
    imu_frame_id = msg.header.frame_id;
    
    tf2::Transform tf = tf2::Transform(tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w));
    
    // derotate acceleration
    curr_acc = tf(curr_acc_meas);
  }
  
  void orb_callback(geometry_msgs::PoseWithCovarianceStamped msg)
  { 
    // compute new T1, T2 and T3
    T1 = T2;
    T2 = T3;
    T3 = msg.header.stamp;
    
    // compute new pose1, pose2 and pose3
    orb_meas1 = orb_meas2;
    orb_meas2 = orb_meas3;
    orb_meas3 = tf2::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    
    msg_n ++;
    
    if (msg_n < 3) return;
    
    // compute acceleration
    ros::Duration dT32 = T3 - T2;
    ros::Duration dT21 = T2 - T1;
    
    tf2::Vector3 orb_vel3 = (orb_meas3 - orb_meas2)/(dT32.toSec());
    tf2::Vector3 orb_vel2 = (orb_meas2 - orb_meas1)/(dT21.toSec());
    
    orb_acc = (orb_vel3 - orb_vel2)/(dT32.toSec());
    cout << "Time: " << T1 << " " << T2 << " " << T3 << " " << dT21.toSec() << " " << dT32.toSec() << endl;
    cout << "orb vel3: " << orb_vel3.getX() << " " << orb_vel3.getY() << " " << orb_vel3.getZ() << endl;
    cout << "orb vel2: " << orb_vel2.getX() << " " << orb_vel2.getY() << " " << orb_vel2.getZ() << endl << endl;
    cout << "orb pose received: " << orb_meas3.getX() << " " << orb_meas3.getY() << " " << orb_meas3.getZ() << endl;
    cout << "orb acceleration: " << orb_acc.getX() << " " << orb_acc.getY() << " " << orb_acc.getZ() << endl << endl;
    
    orb_orientation.setW(msg.pose.pose.orientation.w);
    orb_orientation.setX(msg.pose.pose.orientation.x);
    orb_orientation.setY(msg.pose.pose.orientation.y);
    orb_orientation.setZ(msg.pose.pose.orientation.z);
    
    orb_frame_id = msg.header.frame_id;
  }
  
  geometry_msgs::PoseWithCovarianceStamped get_scaled_pose()
  { 
    geometry_msgs::PoseWithCovarianceStamped pose_out;
    
    pose_out.pose.pose.position.x = orb_meas3.x() / scale;
    pose_out.pose.pose.position.y = orb_meas3.y() / scale;
    pose_out.pose.pose.position.z = orb_meas3.z() / scale;
    
    pose_out.pose.pose.orientation = tf2::toMsg(orb_orientation);
    
    pose_out.header.frame_id = orb_frame_id;
    
    pose_out.header.stamp = ros::Time::now();
    
    return pose_out;
  }
  
  std_msgs::Float32 get_scale()
  {
    std_msgs::Float32 s;
    s.data = scale;
    return s;
  }
  
  void update_scale()
  {
    // update sxx, sxy, syy
    sxx += pow(imu_noise,2) * float(curr_acc.length2());
    syy += pow(orb_noise,2) * float(orb_acc.length2());
    sxy += imu_noise * orb_noise * float(orb_acc.dot(curr_acc));
    
    cout << "scale params updated: " << sxx << " " << syy << " " << sxy << endl;
    
    // update scale and filtered position
    scale = (sxx - syy + copysign(1.0, sxy) * sqrt(pow(sxx - syy,2) + 4*pow(sxy,2)))/(2*imu_noise / orb_noise * sxy);
    cout << "scale: " << scale << endl;
  }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scale_estimator");
    ros::NodeHandle nh;
    
    // estimates scale
    ScaleEstimator scale_est; 
    
    // subscribe to orb pose and accelerometer data from imu
    ros::Subscriber imu_sub = nh.subscribe( "/ardrone/imu", 12,  &ScaleEstimator::imu_callback, &scale_est);
    ros::Subscriber orb_sub = nh.subscribe("/orb/pose_unscaled", 10, &ScaleEstimator::orb_callback, &scale_est);
    
    // publish scale and filtered orb pose
    ros::Publisher filt_orb_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/orb/pose_scaled",2);
    ros::Publisher orb_scale_pub = nh.advertise<std_msgs::Float32>("/orb/scale", 2);
    
    ros::Rate loop_rate(5);
    
    while (ros::ok())
    {    
        ros::spinOnce();
      
	scale_est.update_scale();
	geometry_msgs::PoseWithCovarianceStamped scaled_orb_pose = scale_est.get_scaled_pose();
	
	// TODO: debug
	std_msgs::Float32 scale = scale_est.get_scale();
	cout << "Scale: " << scale << endl;
	cout << "Filtered Pose: " << scaled_orb_pose.pose.pose.position.x << " "
	                          << scaled_orb_pose.pose.pose.position.y << " "
				  << scaled_orb_pose.pose.pose.position.z << endl; 
	
	filt_orb_pub.publish(scaled_orb_pose);
	orb_scale_pub.publish(scale);
        
	loop_rate.sleep();
    }
    
    return 0;
}