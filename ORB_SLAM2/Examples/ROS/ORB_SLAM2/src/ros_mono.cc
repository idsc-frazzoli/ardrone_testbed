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


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <time.h>
#include <math.h> 

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/LinearMath/Quaternion.h>

#include<opencv2/core/core.hpp>

#include "../../../include/System.h"
#include <../../opt/ros/kinetic/include/geometry_msgs/QuaternionStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM), rosPointCloud(){
        rosPointCloud.header.frame_id= "/first_keyframe";
    }
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void prepPointCloud();
    
    ORB_SLAM2::System* mpSLAM;
    
    sensor_msgs::PointCloud rosPointCloud;
    bool initialized = false;
    geometry_msgs::PoseWithCovarianceStamped pose_out_;
    tf::Quaternion quat_cam_drone;
    tf::Quaternion tfqt_adj;
    
    //tf::StampedTransform world_to_level_transform;
    tf::StampedTransform world_to_first_keyframe_transform;
    tf::StampedTransform first_keyframe_to_ardrone_base_frontcam_transform;
    tf::TransformListener tf_listener;
};

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
        if (not initialized) {
            
            // listen to transform odom baselink for trafo from world to first_keyframe
            //tf_listener.lookupTransform("/odom", "/ardrone_base_link", ros::Time(0), world_to_level_transform); // orientation of drone wrt north & down
            tf_listener.lookupTransform("/ardrone_base_link", "/ardrone_base_frontcam", ros::Time(0), world_to_first_keyframe_transform); // orientation of camera wrt drone
            world_to_first_keyframe_transform.setOrigin(tf::Vector3(0, 0, 0));
            
            initialized = true;
        }//ardrone_base_frontcam
        
        // set transformation between first_keyframe and ardrone_base_frontcam
        tf::Vector3 origin;
        tf::Quaternion transform_quat;
        tf::Matrix3x3 transform_matrix;
        
        origin.setValue(pose.at<float>(0,3), pose.at<float>(1,3), pose.at<float>(2,3));
        
        transform_matrix.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2), 
                                  pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2), 
                                  pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));
        
        transform_matrix.getRotation(transform_quat); 
        
        first_keyframe_to_ardrone_base_frontcam_transform.setOrigin(transform_matrix.transpose() * origin * -1);
        first_keyframe_to_ardrone_base_frontcam_transform.setRotation(transform_quat.inverse());
	 
	// broadcast all transforms
	ros::Time t = ros::Time::now();
	//br.sendTransform(tf::StampedTransform(world_to_level_transform.inverse(), t, "/level", "/world"));// world = odom at time 0, level = base_link at time 0 (when orb finds keyframe)
	br.sendTransform(tf::StampedTransform(world_to_first_keyframe_transform.inverse(), t, "/first_keyframe", "/world"));
	br.sendTransform(tf::StampedTransform(first_keyframe_to_ardrone_base_frontcam_transform.inverse(), t, "/ardrone_base_frontcam", "/first_keyframe"));
	
	// plot pose odom to world
	tf::Transform pose_out_tf_non_stamped;
	tf::StampedTransform ardrone_base_frontcam_to_ardrone_base_link;
	
	tf_listener.lookupTransform("/ardrone_base_frontcam", "/ardrone_base_link", ros::Time(0), ardrone_base_frontcam_to_ardrone_base_link); // orientation of odom wrt world
	
	pose_out_tf_non_stamped = first_keyframe_to_ardrone_base_frontcam_transform;// * ardrone_base_frontcam_to_ardrone_base_link;
	tf::StampedTransform pose_out_tf = tf::StampedTransform(pose_out_tf_non_stamped, t, "/world", "/ardrone_base_link");
	
	// 	tf::TransformListener listener1;

// 	listener1.lookupTransform("/world", "/ardrone_base_link", ros::Time(0), pose_out_tf); // orientation of odom wrt world
	pose_out_.header.frame_id = "/first_keyframe";
	pose_out_.header.stamp = t;
	
	tf::Quaternion pose_orientation = pose_out_tf.getRotation();
	tf::Vector3 pose_origin = pose_out_tf.getOrigin();
	pose_out_.pose.pose.orientation.x = pose_orientation.getX(); 
	pose_out_.pose.pose.orientation.y = pose_orientation.getY();
	pose_out_.pose.pose.orientation.z = pose_orientation.getZ();
	pose_out_.pose.pose.orientation.w = pose_orientation.getW();
	pose_out_.pose.pose.position.x = pose_origin.getX();
	pose_out_.pose.pose.position.y = pose_origin.getY();
	pose_out_.pose.pose.position.z = pose_origin.getZ();
	
	
	//tf::StampedTransform transform_out = tf::StampedTransform(transform, t, "/level", "/camera");
	
	//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/level", "/camera"));
	
	// rotate camera pose into world frame
	
	
	// define helper quaternions
// 	tf::Quaternion origin_transform = tf::Quaternion(-0.5, 0.5, -0.5, 0.5);
// 	tf::Quaternion other_transform = tf::Quaternion(0.5, -0.5, 0.5, 0.5);
// 	tf::Quaternion rotation_transform = tf::Quaternion(0, -sqrt(2)/2, 0, sqrt(2)/2);
// 	
// 	rotation_transform = other_transform.inverse() * transform_quat.inverse() * rotation_transform;
// 	
// 	tf::Vector3 origin_i_rotated = tf::quatRotate(origin_transform, transform_matrix.transpose() * origin * -1);
// 	
// 	pose_out_.pose.pose.orientation.x = rotation_transform.getX();
// 	pose_out_.pose.pose.orientation.y = rotation_transform.getY();
// 	pose_out_.pose.pose.orientation.z = rotation_transform.getZ();
// 	pose_out_.pose.pose.orientation.w = rotation_transform.getW();
// 	pose_out_.pose.pose.position.x = origin_i_rotated.getX();
// 	pose_out_.pose.pose.position.y = origin_i_rotated.getY();
// 	pose_out_.pose.pose.position.z = origin_i_rotated.getZ();
// 	
	//TODO: Set covariance
	for(auto& x:pose_out_.pose.covariance)
	  x = 0.0;
	
	pose_out_.pose.covariance[0] = 1;
	pose_out_.pose.covariance[7] = 1;
	pose_out_.pose.covariance[14] = 1;
	pose_out_.pose.covariance[21] = 1;
	pose_out_.pose.covariance[28] = 1;
	pose_out_.pose.covariance[35] = 1;
	
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
    
}

void ImageGrabber::prepPointCloud()
{
    //Get reference to ORB point cloud
    std::vector<ORB_SLAM2::MapPoint*> localPointCloud = mpSLAM->mpMap->GetAllMapPoints(); 
    //We are deleting the current point cloud and copying in the update from ORB
    rosPointCloud.points.clear();
    //Hopefully this will result in a balanced kd-tree
    std::random_shuffle(localPointCloud.begin(),localPointCloud.end());
    
    for(size_t i=0; i<localPointCloud.size();i++)
    {
        if(localPointCloud[i]->isBad()/* or spRefMPs.count(vpMPs[i])*/)
            continue;
        cv::Mat pos = localPointCloud[i]->GetWorldPos();
        geometry_msgs::Point32 pp;
        pp.x=pos.at<float>(0);
        pp.y=pos.at<float>(1);
        pp.z=pos.at<float>(2);
        
        rosPointCloud.points.push_back(pp);
    }
    
}

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
    
    //Timer for publishing point cloud asynchronously
    long int now = time(NULL);
    long int before = time(NULL);
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    
    ImageGrabber imageGrabber(&SLAM);
    
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 10, &ImageGrabber::GrabImage,&imageGrabber);
    ros::Publisher pub = nodeHandler.advertise<sensor_msgs::PointCloud>("/environment/point_cloud", 1);
    ros::Publisher pose_pub = nodeHandler.advertise<geometry_msgs::PoseWithCovarianceStamped>("/orb_pose_measurement",10);
    
    ros::Rate loop_rate(30);
    
    while (ros::ok()) {    

        ros::spinOnce();

        if(imageGrabber.initialized)
        {
            pose_pub.publish(imageGrabber.pose_out_);
        }
        now = time(NULL);
        if((float)(now-before)>0.85)//HACK?
        {
            before = now;
            imageGrabber.prepPointCloud();
            pub.publish(imageGrabber.rosPointCloud);
        }

        loop_rate.sleep();
    }
    
    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();
    
    return 0;
}




