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
#include <../../opt/ros/kinetic/include/geometry_msgs/QuaternionStamped.h>
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

    tf2::Transform orb_odom_to_frontcam, world_to_orb_odom, frontcam_to_base_link;

    geometry_msgs::TransformStamped T_wo;
    geometry_msgs::TransformStamped T_fb;
    geometry_msgs::TransformStamped T_of;
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
    ros::Subscriber sub = nodeHandler.subscribe ( "/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb );
    ros::Publisher point_cloud_pub = nodeHandler.advertise<sensor_msgs::PointCloud> ( "/orb/point_cloud", 2 );
    ros::Publisher pose_pub = nodeHandler.advertise<geometry_msgs::PoseWithCovarianceStamped> ( "/orb/pose",2 );

    tf::TransformListener tfListener;
    tf2_ros::TransformBroadcaster br;

    ros::Rate loop_rate ( 30 );

    while ( ros::ok() ) {
        point_cloud_pub.publish ( igb.pc );
        if ( igb.initialized ) {
            pose_pub.publish ( igb.pose_out_ );
        } else {
            try

            {
                tf::StampedTransform T_1, T_2;
		tfListener.waitForTransform("/odom", "/ardrone_base_frontcam", ros::Time::now(), ros::Duration(3.0));
                tfListener.lookupTransform ( "/odom", "/ardrone_base_frontcam", ros::Time ( 0 ), T_1 );
		tfListener.waitForTransform("/ardrone_base_frontcam", "/ardrone_base_link", ros::Time::now(), ros::Duration(3.0));
                tfListener.lookupTransform ( "/ardrone_base_frontcam", "/ardrone_base_link", ros::Time ( 0 ), T_2 );
		
                igb.world_to_orb_odom.setOrigin ( tf2::Vector3 ( T_1.getOrigin().getX(), T_1.getOrigin().getY(),T_1.getOrigin().getZ() ) );
                igb.frontcam_to_base_link.setOrigin ( tf2::Vector3 ( T_2.getOrigin().getX(), T_2.getOrigin().getY(),T_2.getOrigin().getZ() ) );
                igb.world_to_orb_odom.setRotation ( tf2::Quaternion ( T_1.getRotation().getX(), T_1.getRotation().getY(), T_1.getRotation().getZ(), T_1.getRotation().getZ() ) );
                igb.frontcam_to_base_link.setRotation ( tf2::Quaternion ( T_2.getRotation().getX(), T_2.getRotation().getY(), T_2.getRotation().getZ(), T_2.getRotation().getZ() ) );

            } catch ( tf2::TransformException &ex ) {
                ROS_WARN ( "%s",ex.what() );
                continue;
            }



        }
        ros::spinOnce();
	
        br.sendTransform ( igb.T_wo );
        br.sendTransform ( igb.T_of );
        br.sendTransform ( igb.T_fb );

        loop_rate.sleep();
    }


    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM ( "KeyFrameTrajectory.txt" );

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
    // Main slam routine. Extracts new pose
    cv::Mat pose = mpSLAM->TrackMonocular ( cv_ptr->image,cv_ptr->header.stamp.toSec() );

    // if points can be tracked then broadcast the pose
    if ( not pose.empty() ) {
        if ( not initialized ) {
            initialized = true;
        }

        static tf2_ros::TransformBroadcaster br;

        // set transformation between first_keyframe and ardrone_base_frontcam
        tf2::Vector3 translation = tf2::Vector3 ( pose.at<float> ( 0,3 ), pose.at<float> ( 1,3 ), pose.at<float> ( 2,3 ) );
        tf2::Matrix3x3 rotation = tf2::Matrix3x3 ( pose.at<float> ( 0,0 ), pose.at<float> ( 0,1 ), pose.at<float> ( 0,2 ),
                                  pose.at<float> ( 1,0 ), pose.at<float> ( 1,1 ), pose.at<float> ( 1,2 ),
                                  pose.at<float> ( 2,0 ), pose.at<float> ( 2,1 ), pose.at<float> ( 2,2 ) );

        orb_odom_to_frontcam = tf2::Transform ( rotation.transpose(), rotation.transpose() * translation * -1 );

        // broadcast all transforms
        T_wo = toTFStamped ( world_to_orb_odom, ros::Time::now(), "/world", "/orb/odom" );
        T_of = toTFStamped ( orb_odom_to_frontcam, ros::Time::now(), "/orb/odom", "/orb/frontcam" );
        T_fb = toTFStamped ( frontcam_to_base_link, ros::Time::now(), "/orb/frontcam", "/orb/base_link" );

        // get pose of base_link in world frame
	world_to_orb_odom.setOrigin(tf2::Vector3(0,0,0));
	frontcam_to_base_link.setOrigin(tf2::Vector3(0,0,0));
        tf2::Transform T_wb = world_to_orb_odom * orb_odom_to_frontcam * frontcam_to_base_link;

        // set pose
        pose_out_.header.stamp = ros::Time::now();

        pose_out_.pose.pose.orientation.x = T_wb.getRotation().getX();
        pose_out_.pose.pose.orientation.y = T_wb.getRotation().getY();
        pose_out_.pose.pose.orientation.z = T_wb.getRotation().getZ();
        pose_out_.pose.pose.orientation.w = T_wb.getRotation().getW();

        pose_out_.pose.pose.position.x = T_wb.getOrigin().getX();
        pose_out_.pose.pose.position.y= T_wb.getOrigin().getY();
        pose_out_.pose.pose.position.z = T_wb.getOrigin().getZ();

        //TODO: Set covariance
        for ( auto& x:pose_out_.pose.covariance ) {
            x = 0.0;
        }

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


