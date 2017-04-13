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

using namespace std;

class ImageGrabber {
    int image_counter=0;
    public:
    ImageGrabber ( ORB_SLAM2::System *pSLAM );

    void grabImage ( const sensor_msgs::ImageConstPtr &msg );
    geometry_msgs::TransformStamped toTFStamped ( tf2::Transform in , ros::Time t, string frame_id, string child_frame_id );
    bool isPointCloudValid();
    void initializeTFTree ( cv::Mat Tcw );

    tf::Transform cvMatToTF ( cv::Mat Tcw );
    double getAverageDepth ();
    void navCallback ( ardrone_autonomy::Navdata msg ) ;

    void setScale ( std_msgs::Float32 msg );
    tf::Transform scaleTF ( tf::Transform T, double scale );
    void writePoseOutUnscaled ( tf::Transform &transform, ros::Time &stamp );
    void writePoseOutScaled ( geometry_msgs::PoseStamped &pose, double &scale );
    void updatePathArray ();

    ORB_SLAM2::System *mpSLAM_;

    bool pose_init_ = false;
    bool pc_init_ = false;
    bool is_started_ = false;
    int tracking_state_= 0;
    bool isPathTracking_ = false;

    double scale_ = 1;
    double normalizer_ = 1;

    sensor_msgs::PointCloud pc_;
    geometry_msgs::PoseStamped pose_out_unscaled_;
    geometry_msgs::PoseStamped pose_out_scaled_;

    tf::TransformListener tf_listener_;

    tf::TransformBroadcaster br_;

    tf::StampedTransform base_link_to_camera_transform_;
    tf::StampedTransform odom_to_second_keyframe_base_transform_;
    tf::Transform second_keyframe_cam_to_first_keyframe_cam_transform_;
    tf::StampedTransform base_link_to_camera_transform_no_translation_;
    tf::StampedTransform yaw_before_jump_tf_;
    tf::Transform yaw_correction_tf_ = tf::Transform ( tf::Quaternion ( 0,0,0,1 ) );

    vector<geometry_msgs::PoseStamped> poses_;
    nav_msgs::Path scaled_path_;

};