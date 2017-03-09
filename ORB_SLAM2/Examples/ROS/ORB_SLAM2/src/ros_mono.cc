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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber ( ORB_SLAM2::System* pSLAM ) :mpSLAM ( pSLAM ), pc() {
        pc.header.frame_id= "/orb/odom";
        pose_out_.header.frame_id = "/orb/odom";
    }
    void GrabImage ( const sensor_msgs::ImageConstPtr& msg );
    geometry_msgs::TransformStamped toTFStamped ( tf2::Transform in , ros::Time t, string frame_id, string child_frame_id );


    ORB_SLAM2::System* mpSLAM;

    bool initialized = false;

    sensor_msgs::PointCloud pc;
    geometry_msgs::PoseWithCovarianceStamped pose_out_;
    
    tf::StampedTransform first_keyframe_to_odom_transform;
    tf::TransformListener tf_listener;
};

int main ( int argc, char **argv )
{
    ros::init ( argc,argv,"Mono" );
    ros::start();

    if ( argc != 3 ) {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM ( argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true );

    ImageGrabber igb ( &SLAM );

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Publisher pub = nodeHandler.advertise<sensor_msgs::PointCloud>("/environment/point_cloud", 2);
    ros::Publisher pose_pub = nodeHandler.advertise<geometry_msgs::PoseWithCovarianceStamped>("/orb_pose_unscaled",2);
    
    ros::Rate loop_rate(30);
    
    while (ros::ok()) {    
        ros::spinOnce();
	
        br.sendTransform ( igb.T_wo );
        br.sendTransform ( igb.T_of );
        br.sendTransform ( igb.T_fb );

        loop_rate.sleep();
    }


    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    
    ros::shutdown();

    return 0;
}

//callback
void ImageGrabber::GrabImage ( const sensor_msgs::ImageConstPtr& msg )
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare ( msg );
    } catch ( cv_bridge::Exception& e ) {
        ROS_ERROR ( "cv_bridge exception: %s", e.what() );
        return;
    }
    //     Main slam routine. Extracts new pose
    cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
    
    //////////////////////////////////TRANSFORMATIONS//////////////////////////////////////////////////////////////////
    //To fuse the orb SLAM pose estimate with the Kalman Filter of the robot_localization package, it is
    //necessary to publish any other sensor data and the orb SLAM data in a conforming parent frame which is typically
    //called 'odom'. See REP105 and REP103 on ros.org for further details on the concept. 
    //
    //The final transformation for the orb SLAM looks like this: 
    //
    //		odom --> first_keyframe (orb initialization) --> orb_pose --> correction
    //
    //odom is defined by the IMU at startup (this is not the startup time of the driver node but the time the plug
    //of the drone is connected) and published by the ardrone driver. Odom must not be changed at any time afterwards.
    //
    //The first keyframe transformation is set once the orb slam initializes (meaning it is able to estimate a position 
    //for the first time). It is set to be the transformation from odom to base_link (published by the driver). 
    //Since odom is set by the imu, this step is necessary to bring both sensors to the same coordinate base (namely odom). 
    //This transformation must also not change at any time after it's initialization.
    //
    //Camera frames for some reason always come in a way such that the z axis is pointing forwards while the y axis is
    //facing downwards. This has to be corrected through a manual rotation such that eventually the orb pose values 
    //which are beeing publsihed to a PoseWithCovarianceStamped topic represent the correct pose within the odom frame.
    //See the rqt_tf_tree for further calrification on the single transformations this code is performing.
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    // if points can be tracked then broadcast the pose 
    if (not pose.empty()) {

	if (not initialized) {
	//Initialization - set link between 'odom' and 'first_keyframe' frames
	
	tf_listener.lookupTransform("odom", "/ardrone_base_frontcam", ros::Time(0), first_keyframe_to_odom_transform);
	first_keyframe_to_odom_transform.setOrigin(tf::Vector3(0, 0, 0));	//just for clarification reasons
	initialized = true;
    }   
        //Extract transformation from the raw orb SLAM pose
        tf::Vector3 origin;
        tf::Quaternion transform_quat;
        tf::Matrix3x3 transform_matrix;
	tf::StampedTransform first_keyframe_to_orb_pose_transform;
	
        origin.setValue(pose.at<float>(0,3), pose.at<float>(1,3), pose.at<float>(2,3));
       
	transform_matrix.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2), 
		                  pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2), 
                                  pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));
	
        transform_matrix.getRotation(transform_quat); 
        
        first_keyframe_to_orb_pose_transform.setOrigin(transform_matrix.transpose() * origin * -1);
        first_keyframe_to_orb_pose_transform.setRotation(transform_quat.inverse());
	
	//initialize some stuff
	ros::Time t = ros::Time::now();
	tf::Transform pose_out_tf_non_stamped;
	tf::StampedTransform cam_to_base_link_transform;
	
	//Get correction transform
	tf_listener.lookupTransform("/ardrone_base_link", "/ardrone_base_frontcam", ros::Time(0), cam_to_base_link_transform); //camera frame correction
	//The driver publishes a translation (because the camera is aprox. 15cm in front of the body center), but we do not have a scale at this point 
	//so we cannot handle this transformation properly - the next line assumes that the camera is in the body frame (no translation) which is still
	//sufficiently accurate
	cam_to_base_link_transform.setOrigin(tf::Vector3(0, 0, 0)); 

	//now all rotation/translation axis are set correctly
	//however, we rotate the frame to align with odom (this is cosmetics only
	tf::Quaternion r1 = tf::createQuaternionFromRPY(0,-M_PI/2,0);
        tf::Quaternion r2 = tf::createQuaternionFromRPY(M_PI/2.0,0.0,0.0);
        
        tf::Quaternion correction = r1*r2;
        
	tf::Transform correction_transform;
	correction_transform.setRotation(correction);
	correction_transform.setOrigin(tf::Vector3(0, 0, 0));	//just for clarification reasons
	
	//Apply transformation and correction
	tf::Transform pose_out_corrected;
	pose_out_corrected = cam_to_base_link_transform*first_keyframe_to_orb_pose_transform*correction_transform;
	
	//Broadcast all transforms
	//br.sendTransform(tf::StampedTransform(pose_out_corrected, t, "odom", "/orb_pose_unscaled_corrected"));			//ONLY FOR DEBUGGING - UNNECESSARY TF LATER
	//br.sendTransform(tf::StampedTransform(first_keyframe_to_orb_pose_transform, t, "/first_keyframe", "/orb_pose_unscaled")); 	//ONLY FOR DEBUGGING - UNNECESSARY TF LATER
	br.sendTransform(tf::StampedTransform(first_keyframe_to_orb_pose_transform*correction_transform, t, "/first_keyframe", "/orb_pose_unscaled_corrected"));
    	br.sendTransform(tf::StampedTransform(first_keyframe_to_odom_transform, t, "odom", "/first_keyframe"));
	
	//generate pose for robot_localization EKF sensor fusion
	//the pose is simply generated from the above derived transformations
	pose_out_.header.frame_id = "odom";
	pose_out_.header.stamp = t;
	
	tf::Quaternion pose_orientation = pose_out_corrected.getRotation();
	tf::Vector3 pose_origin = pose_out_corrected.getOrigin();
	pose_out_.pose.pose.orientation.x = pose_orientation.getX(); 
	pose_out_.pose.pose.orientation.y = pose_orientation.getY();
	pose_out_.pose.pose.orientation.z = pose_orientation.getZ();
	pose_out_.pose.pose.orientation.w = pose_orientation.getW();
	pose_out_.pose.pose.position.x = pose_origin.getX();
	pose_out_.pose.pose.position.y = pose_origin.getY();
	pose_out_.pose.pose.position.z = pose_origin.getZ();
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////77
	//TODO: Set covariance
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for(auto& x:pose_out_.pose.covariance)
	  x = 0.0;
	
	pose_out_.pose.covariance[0] = 1;
	pose_out_.pose.covariance[7] = 1;
	pose_out_.pose.covariance[14] = 1;
	pose_out_.pose.covariance[21] = 1;
	pose_out_.pose.covariance[28] = 1;
	pose_out_.pose.covariance[35] = 1;
    }

    // gets points from most recent frame
    // gets all points
    const std::vector<ORB_SLAM2::MapPoint*> &point_cloud = mpSLAM->mpMap->GetAllMapPoints();
    // TODO: make efficient (use mpSLAM->GetTrackedMapPoints() to get most recent points)
    pc.points.clear();
    for ( size_t i=0; i<point_cloud.size(); i++ ) {
        if ( point_cloud[i]->isBad() /* or spRefMPs.count(vpMPs[i])*/ ) {
            continue;
        }
        cv::Mat pos = point_cloud[i]->GetWorldPos();
        geometry_msgs::Point32 pp;
        pp.x=pos.at<float> ( 0 );
        pp.y=pos.at<float> ( 1 );
        pp.z=pos.at<float> ( 2 );

        pc.points.push_back ( pp );
    }

}

geometry_msgs::TransformStamped ImageGrabber::toTFStamped ( tf2::Transform in , ros::Time t, string frame_id, string child_frame_id )
{
    geometry_msgs::TransformStamped out;
    out.child_frame_id = child_frame_id;
    out.header.frame_id = frame_id;
    out.header.stamp = t;

    out.transform.rotation.x = in.getRotation().getX();
    out.transform.rotation.y = in.getRotation().getY();
    out.transform.rotation.z = in.getRotation().getZ();
    out.transform.rotation.w = in.getRotation().getW();

    out.transform.translation.x = in.getOrigin().getX();
    out.transform.translation.y = in.getOrigin().getY();
    out.transform.translation.z = in.getOrigin().getZ();

    return out;
}


