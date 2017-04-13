/*
 * (c) 2017
 *
 * Authors:
 * Daniel Gehrig (ETH Zurich)
 * Maximilian Goettgens (ETH Zurich)
 * Brian Paden (MIT)
 *
 * This file was originally written as "ros_mono.cc" by Raul Mur-Artal and
 * has been modified by the authors to serve several tasks needed for the
 * ardrone to properly use the ORB SLAM 2 algorithm for state estimation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 * IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//////////////////////////////////TRANSFORMATIONS//////////////////////////////////////////////////////////////////
//To fuse the orb SLAM pose estimate with onboard sensor data, it is
//necessary to publish any other sensor data and the orb SLAM data in the same parent frame which is typically
//called 'odom'. See REP105 and REP103 on ros.org for further details on the concept.
//
//The final transformation for the orb SLAM looks like this:
//
//    odom --> second_keyframe_base_link --> second_keyframe_cam --> first_keyframe_cam
//    --> orb_pose_unscaled_cam --> orb_pose_unscaled_base_link
//
//Odom is defined by the drone on startup (this is not the startup time of the driver node but the time the plug is connected)
//Odom is defined with it's x-axis facing north and the negative z-axis aligned to the gravitation vector and must
//not be changed at any time afterwards.
//
//The first and second keyframe transformation is set once the orb slam initializes (meaning it is able to estimate a position
//for the first time). It is set to be the transformation from odom to ardrone_base_frontcam (published by the driver).
//It is important to mention, that the first transformation, that is actually received from the ORB SLAM algorithm is the
//transformation to the SECOND keyframe. There is a need to compensate for this to get a correct pose from the first keyframe on.
//
//Camera frames always come in a way such that the z axis is pointing forwards while the y axis is
//facing downwards. Therefore the transformations to switch frames are here achieved through a manual rotation such that eventually
//the orb pose values which are beeing publsihed to a PoseStamped topic represent the correct pose within the odom frame.
//See the rqt_tf_tree for further calrification on the single transformations this code is performing.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <nav_msgs/Path.h>

#include <tf2/LinearMath/Transform.h>
#include <std_msgs/Float32.h>
#include <ardrone_autonomy/Navdata.h>

#include<opencv2/core/core.hpp>

#include <System.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <ardrone_orb_adapter.h>

using namespace std;

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "Mono" );
    ros::start();

    if ( argc != 3 ) {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM ( argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true );

    ImageGrabber igb ( &SLAM );

    ros::NodeHandle nodeHandler;

    ros::Subscriber cam_sub = nodeHandler.subscribe ( "/camera/image_raw", 1, &ImageGrabber::grabImage, &igb );
    ros::Subscriber scale_sub = nodeHandler.subscribe ( "/scale_estimator/scale", 1, &ImageGrabber::setScale, &igb );
    ros::Subscriber nav_sub = nodeHandler.subscribe ( "/ardrone/navdata", 10, &ImageGrabber::navCallback, &igb );

    ros::Publisher pointcloud_pub = nodeHandler.advertise<sensor_msgs::PointCloud> ( "/orb/point_cloud", 2 );
    ros::Publisher pose_unscaled_pub = nodeHandler.advertise<geometry_msgs::PoseStamped> ( "/orb/pose_unscaled", 2 );
    ros::Publisher pose_scaled_pub = nodeHandler.advertise<geometry_msgs::PoseStamped> ( "/orb/pose_scaled", 2 );
    ros::Publisher path_pub = nodeHandler.advertise<nav_msgs::Path> ( "/ardrone/path_scaled",2 );

    ros::Rate loop_rate ( 30 );
    int counter = 0;

    while ( ros::ok() ) {
        ros::spinOnce();
        if ( igb.pc_init_  && counter % 30 == 0 ) {
            pointcloud_pub.publish ( igb.pc_ ); //publish at 1 Hz
        }

        if ( igb.pose_init_ && igb.tracking_state_ == 2 ) {
            pose_unscaled_pub.publish ( igb.pose_out_unscaled_ );
            pose_scaled_pub.publish ( igb.pose_out_scaled_ );
            if ( igb.isPathTracking_ ) {
                igb.updatePathArray();
                path_pub.publish ( igb.scaled_path_ );
            }
        }
        counter++;
        if (counter == 3000) counter = 0; //preventing overflow of int
        
        loop_rate.sleep();
    }

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
