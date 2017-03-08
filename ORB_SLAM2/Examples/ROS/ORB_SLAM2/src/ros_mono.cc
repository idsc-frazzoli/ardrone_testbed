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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber ( ORB_SLAM2::System* pSLAM ) :mpSLAM ( pSLAM ), pc() {
        pc.header.frame_id= "/orb/odom";
        pose_out_.header.frame_id = "/orb/odom";
    }
    void GrabImage ( const sensor_msgs::ImageConstPtr& msg );

    ORB_SLAM2::System* mpSLAM;

    bool initialized = false;

    sensor_msgs::PointCloud pc;
    geometry_msgs::PoseWithCovarianceStamped pose_out_;

    tf2::Transform orb_odom_to_frontcam, world_to_orb_odom, frontcam_to_base_link;
    tf2_ros::Buffer tf_buffer;
};

int main ( int argc, char **argv )
{
    ros::init(argc,argv,"Mono");
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

    ros::Rate loop_rate ( 30 );

    while ( ros::ok() ) {
        ros::spinOnce();
        point_cloud_pub.publish ( igb.pc );
        if ( igb.initialized ) {
            pose_pub.publish ( igb.pose_out_ );
        }
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
            tf2::convert ( tf_buffer.lookupTransform ( "/odom", "/ardrone_base_frontcam", ros::Time ( 0 ) ), world_to_orb_odom );
            tf2::convert ( tf_buffer.lookupTransform ( "/adrone_base_frontcam", "/adrone_base_link", ros::Time ( 0 ) ), frontcam_to_base_link );

            initialized = true;
        }

        static tf2_ros::TransformBroadcaster br;

        // set transformation between first_keyframe and ardrone_base_frontcam
        tf2::Vector3 translation = tf2::Vector3 ( pose.at<float> ( 0,3 ), pose.at<float> ( 1,3 ), pose.at<float> ( 2,3 ) );
        tf2::Matrix3x3 rotation = tf2::Matrix3x3 ( pose.at<float> ( 0,0 ), pose.at<float> ( 0,1 ), pose.at<float> ( 0,2 ),
                                  pose.at<float> ( 1,0 ), pose.at<float> ( 1,1 ), pose.at<float> ( 1,2 ),
                                  pose.at<float> ( 2,0 ), pose.at<float> ( 2,1 ), pose.at<float> ( 2,2 ) );

        orb_odom_to_frontcam = tf2::Transform ( rotation.transpose(), rotation.transpose() * translation * -1);

        // broadcast all transforms
        geometry_msgs::TransformStamped T_wo, T_of, T_fb;

        T_wo.transform = tf2::toMsg ( world_to_orb_odom );
        T_of.transform = tf2::toMsg ( orb_odom_to_frontcam );
        T_fb.transform = tf2::toMsg ( frontcam_to_base_link );

        T_wo.header.stamp = T_of.header.stamp = T_fb.header.stamp = ros::Time::now();

        T_wo.child_frame_id = T_of.header.frame_id = "/orb/odom";
        T_of.child_frame_id = T_fb.header.frame_id = "/orb/frontcam";
        T_fb.child_frame_id = "/orb/base_link";
        T_wo.header.frame_id = "/world";

        br.sendTransform ( T_wo );
        br.sendTransform ( T_of );
        br.sendTransform ( T_fb );

        // get pose of base_link in world frame
        tf2::Transform T_wb = world_to_orb_odom * orb_odom_to_frontcam * frontcam_to_base_link;

        // set pose
        pose_out_.header.stamp = ros::Time::now();

        pose_out_.pose.pose.orientation = tf2::toMsg ( T_wb.getRotation() );
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


