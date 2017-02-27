/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <math.h> 

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/LinearMath/Quaternion.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include <../../opt/ros/kinetic/include/geometry_msgs/QuaternionStamped.h>
#include <tf/transform_broadcaster.h>


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM), pc(){
        pc.header.frame_id= "/level";
    }
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    
    ORB_SLAM2::System* mpSLAM;
    
    sensor_msgs::PointCloud pc;
    geometry_msgs::PoseWithCovarianceStamped pose_out_;
    tf::Quaternion tfqt_i;
    tf::Matrix3x3 tf3d;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    
    ImageGrabber igb(&SLAM);
    
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Publisher pub = nodeHandler.advertise<sensor_msgs::PointCloud>("/environment/point_cloud", 2);
    ros::Publisher pose_pub = nodeHandler.advertise<geometry_msgs::PoseWithCovarianceStamped>("orb_pose_measurement",2);
    
    ros::Rate loop_rate(30);
    
    while (ros::ok()) {
//       geometry_msgs::Quaternion q = igb.pose_out_.pose.pose.orientation;
// 	double ysqr = q.y * q.y;
// 
// 	// roll (x-axis rotation)
// 	double t0 = +2.0 * (q.w * q.x + q.y * q.z);
// 	double t1 = +1.0 - 2.0 * (q.x * q.x + ysqr);
// 	double roll1 = std::atan2(t0, t1);
// 
// 	// pitch (y-axis rotation)
// 	double t2 = +2.0 * (q.w * q.y - q.z * q.x);
// 	t2 = t2 > 1.0 ? 1.0 : t2;
// 	t2 = t2 < -1.0 ? -1.0 : t2;
// 	double pitch1 = std::asin(t2);
// 
// 	// yaw (z-axis rotation)
// 	double t3 = +2.0 * (q.w * q.z + q.x * q.y);
// 	double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);  
// 	double yaw1 = std::atan2(t3, t4);
// 	
// 	
// 	
// 	ysqr = igb.tfqt_i.getY() * igb.tfqt_i.getY();
// 
// 	// roll (x-axis rotation)
// 	t0 = +2.0 * (igb.tfqt_i.getW() * igb.tfqt_i.getX() + igb.tfqt_i.getY() * igb.tfqt_i.getZ());
// 	t1 = +1.0 - 2.0 * (igb.tfqt_i.getX() * igb.tfqt_i.getX() + ysqr);
// 	double roll = std::atan2(t0, t1);
// 
// 	// pitch (y-axis rotation)
// 	t2 = +2.0 * (igb.tfqt_i.getW() * igb.tfqt_i.getY() - igb.tfqt_i.getZ() * igb.tfqt_i.getX());
// 	t2 = t2 > 1.0 ? 1.0 : t2;
// 	t2 = t2 < -1.0 ? -1.0 : t2;
// 	double pitch = std::asin(t2);
// 
// 	// yaw (z-axis rotation)
// 	t3 = +2.0 * (igb.tfqt_i.getW() * igb.tfqt_i.getZ() + igb.tfqt_i.getX() * igb.tfqt_i.getY());
// 	t4 = +1.0 - 2.0 * (ysqr + igb.tfqt_i.getZ() * igb.tfqt_i.getZ());  
// 	double yaw = std::atan2(t3, t4);
       tf::Matrix3x3 tf3d_extra(0, 0, 1,1,0,0,0,1,0);
       tf::Matrix3x3 new_matrix;
       
       float roll, pitch, yaw, roll1, pitch1, yaw1; 
      
       igb.tf3d = igb.tf3d.inverse();
       
       igb.tf3d.getRPY(roll, pitch, yaw);
       tf::Matrix3x3 new_matrix = tf3d_extra * igb.tf3d;
       new_matrix.getRPY(roll1, pitch1, yaw1);
      
      
      
      	printf("roll: %f, pitch: %f, yaw: %f\nroll1: %f, pitch1: %f, yaw1: %f\n", roll, pitch, yaw, roll1, pitch1, yaw1);
	/*
		10*odometer.odom_pos.getZ())
			printf("slam_w: %f, slam_x: %f ,slam_y: %f, slam_z: %f\n",odometer.slam_quat.getW(),
		odometer.slam_quat.getX(),
		odometer.slam_quat.getY(),a
		
		odometer.slam_quat.getZ());

	printf(" imu_w: %f,  imu_x: %f,  imu_y: %f,  imu_z: %f\n",odometer.imu_quat.getW(),
	      odometer.imu_quat.getX(),
	      odometer.imu_quat.getY(),
	      odometer.imu_quat.getZ());

	printf("diff_x: %f, diff_x: %f, diff_y: %f, diff_z: %f\n",odometer.avg_quat.getW(),
	      odometer.avg_quat.getX(),
	      odometer.avg_quat.getY(),
	      odometer.avg_quat.getZ());*/
        
        ros::spinOnce();
        pub.publish(igb.pc);
	pose_pub.publish(igb.pose_out_);
        loop_rate.sleep();
    }
    
    
    // Stop all threads
    SLAM.Shutdown();
    
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    
    ros::shutdown();
    
    return 0;
}

//callback
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //     Main slam routine. Extracts new pose
    cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
    // if points can be tracked then broadcast the pose 
    if (not pose.empty()) {
        
        tf::Vector3 origin;
        tf::Quaternion tfqt;
        tf::Matrix3x3 tf3d;

	
        origin.setValue(pose.at<float>(0,3), 
                        pose.at<float>(1,3), 
                        pose.at<float>(2,3));
       
	tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), 
                      pose.at<float>(0,2), pose.at<float>(1,0), 
                      pose.at<float>(1,1), pose.at<float>(1,2), 
                      pose.at<float>(2,0), pose.at<float>(2,1), 
                      pose.at<float>(2,2));
        tf3d.getRotation(tfqt); 
        
        transform.setOrigin(tf3d.transpose() * origin * -1);
        transform.setRotation(tfqt.inverse());
	
	// invert pose for orb pose publishing
	tfqt_i = tfqt.inverse();
	tf::Matrix3x3 tf3d_extra_i = tf3d_extra.inverse();
	tf::Vector3 origin_i = tf3d.transpose() * origin * -1;
	
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/level", "/odom"));
	pose_out_.header.stamp = ros::Time::now();
	
        tf::Quaternion r1 = tf::createQuaternionFromRPY(-M_PI/2.0,M_PI/2.0,0.0);
        tf::Quaternion r2 = tf::createQuaternionFromRPY(0,-M_PI/2,0);
        tf::Quaternion r3 = tf::createQuaternionFromRPY(0,0,-M_PI/2);
        tf::Quaternion r4 = tf::createQuaternionFromRPY(0,0,M_PI/2);
	
        tf::Quaternion quat_cam_drone = r3*r2*r1;
	
	tf::Quaternion tfqt_i_rot = r4*quat_cam_drone*tfqt_i;
	origin_i = tf::quatRotate(quat_cam_drone,origin_i);
	
	
	pose_out_.pose.pose.orientation.x = tfqt_i_rot.getX();
	pose_out_.pose.pose.orientation.y = tfqt_i_rot.getY();
	pose_out_.pose.pose.orientation.z = tfqt_i_rot.getZ();
	pose_out_.pose.pose.orientation.w = tfqt_i_rot.getW();
	pose_out_.pose.pose.position.x = origin_i.getX();
	pose_out_.pose.pose.position.y = origin_i.getY();
	pose_out_.pose.pose.position.z = origin_i.getZ();
	
	//TODO: Set covariance
	for(auto& x:pose_out_.pose.covariance)
	  x = 0.0;
	
	
	pose_out_.header.frame_id = "world";
	
// 	pose_out.pose.covariance. = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
// 				    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
// 				    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
// 				    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
// 				    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
// 				    0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
// 				//    pose_out.pose.cova
    }
    
    // gets points from most recent frame
    // gets all points
    const std::vector<ORB_SLAM2::MapPoint*> &point_cloud = mpSLAM->mpMap->GetAllMapPoints();
    // TODO: make efficient (use mpSLAM->GetTrackedMapPoints() to get most recent points)
    pc.points.clear();
    for(size_t i=0; i<point_cloud.size();i++)
    {
        if(point_cloud[i]->isBad()/* or spRefMPs.count(vpMPs[i])*/)
            continue;
        cv::Mat pos = point_cloud[i]->GetWorldPos();
        geometry_msgs::Point32 pp;
        pp.x=pos.at<float>(0);
        pp.y=pos.at<float>(1);
        pp.z=pos.at<float>(2);
        
        pc.points.push_back(pp);
    }
    
}


