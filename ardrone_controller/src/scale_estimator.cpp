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
  float sxx=0, syy=0, sxy=0, orb_noise=0, imu_noise=0; // noise params must be tuned
  float imu_time=0, deltaT=0;
  bool first_msg=true;
  
  tf2::Vector3 orb_displacement, last_orb_meas;
  tf2::Vector3 imu_displacement, curr_imu_vel, last_imu_meas;
  tf2::Quaternion orb_orientation;
  
  string imu_frame_id, orb_frame_id;
 
  void imu_callback(sensor_msgs::Imu msg)
  {
    // find past time
    if (first_msg) {
      first_msg = false;
      imu_time = msg.header.stamp.toSec();
      deltaT = 0;
    }
    else {
      deltaT = msg.header.stamp.toSec() - imu_time;
      imu_time = msg.header.stamp.toSec();
    }
    
    tf2::Vector3 curr_acc_meas = tf2::Vector3(tf2Scalar(msg.linear_acceleration.x), 
				             tf2Scalar(msg.linear_acceleration.y), 
					     tf2Scalar(msg.linear_acceleration.z));
    
    imu_frame_id = msg.header.frame_id;
    
    tf2::Transform tf = tf2::Transform(tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w));
    
    // derotate acceleration
    tf2::Vector3 derot_acc = tf(curr_acc_meas);
    
    // find imu_displacement
    imu_displacement = tf2Scalar(deltaT) * curr_imu_vel + derot_acc * tf2Scalar(0.5 * pow(deltaT, 2));  
    curr_imu_vel += derot_acc * tf2Scalar(deltaT);
  }
  
  void orb_callback(geometry_msgs::PoseWithCovarianceStamped msg)
  {
    tf2::Vector3 curr_orb_meas = tf2::Vector3(tf2Scalar(msg.pose.pose.position.x), 
				              tf2Scalar(msg.pose.pose.position.y), 
				              tf2Scalar(msg.pose.pose.position.z));
    
    orb_displacement = curr_orb_meas - last_orb_meas;
    last_orb_meas = curr_orb_meas;
    
    orb_orientation.setW(msg.pose.pose.orientation.w);
    orb_orientation.setX(msg.pose.pose.orientation.x);
    orb_orientation.setY(msg.pose.pose.orientation.y);
    orb_orientation.setZ(msg.pose.pose.orientation.z);
    
    orb_frame_id = msg.header.frame_id;
  }
  
  geometry_msgs::PoseWithCovarianceStamped get_scaled_pose()
  { 
    geometry_msgs::PoseWithCovarianceStamped pose_out;
    
    pose_out.pose.pose.position.x = last_orb_meas.x() / scale;
    pose_out.pose.pose.position.y = last_orb_meas.y() / scale;
    pose_out.pose.pose.position.z = last_orb_meas.z() / scale;
    
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
    sxx += pow(imu_noise,2) * float(orb_displacement.length2());
    syy += pow(orb_noise,2) * float(imu_displacement.length2());
    sxy += imu_noise * orb_noise * float(orb_displacement.dot(imu_displacement));
    
    // update scale and filtered position
    scale = (sxx - syy + copysign(1.0, sxy) * sqrt(pow(sxx - syy,2) + 4*pow(sxy,2)))/(2*imu_noise / orb_noise * sxy);
  }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scale_estimator");
    ros::NodeHandle nh;
    
    // estimates scale
    ScaleEstimator scale_est; 
    
    // subscribe to orb pose and accelerometer data from imu
    ros::Subscriber imu_sub = nh.subscribe( "/ardrone/imu", 10,  &ScaleEstimator::imu_callback, &scale_est);
    ros::Subscriber orb_sub = nh.subscribe("/orb/pose", 10, &ScaleEstimator::orb_callback, &scale_est);
    
    // publish scale and filtered orb pose
    ros::Publisher filt_orb_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/orb/pose_scaled",2);
    ros::Publisher orb_scale_pub = nh.advertise<std_msgs::Float32>("/orb/scale", 2);
    
    ros::Rate loop_rate(30);
    
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