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

#include <ardrone_orb_adapter.h>

ImageGrabber::ImageGrabber ( ORB_SLAM2::System *pSLAM ) : mpSLAM_ ( pSLAM ), pc_() {
    pc_.header.frame_id = "/first_keyframe_cam";
    pose_out_scaled_.header.frame_id = "odom";
    pose_out_unscaled_.header.frame_id = "odom";
    scaled_path_.header.frame_id = "odom";
}

void ImageGrabber::grabImage ( const sensor_msgs::ImageConstPtr &msg ) {

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare ( msg );
    } catch ( cv_bridge::Exception &e ) {
        ROS_ERROR ( "cv_bridge exception: %s", e.what() );
        return;
    }
    // Main slam routine. Extracts new pose
    // (from ORB_SLAM2 API)
    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat Tcw = mpSLAM_->TrackMonocular ( cv_ptr->image, cv_ptr->header.stamp.toSec() );
    tracking_state_ = mpSLAM_->GetTrackingState();

    ros::Time t = msg->header.stamp;

    // if points can be tracked then broadcast the pose
    if ( not Tcw.empty() ) {
        if ( not pc_init_ && isPointCloudValid() ) {
            pc_init_ = true;
            normalizer_ = getAverageDepth();
        }
        
        //upon initialization, a correction 
        if ( not pose_init_ ) {
            initializeTFTree ( Tcw );
        }

        // rescale unscaled orb pose with normalizer
        tf::Transform orb_pose_unscaled_cam_to_first_keyframe_cam = cvMatToTF ( Tcw );
        orb_pose_unscaled_cam_to_first_keyframe_cam = scaleTF ( orb_pose_unscaled_cam_to_first_keyframe_cam , normalizer_ );

        // construct unscaled and scaled orb pose in odom frame
        tf::Transform odom_to_pose_out = odom_to_second_keyframe_base_transform_
                                         * base_link_to_camera_transform_no_translation_
                                         * second_keyframe_cam_to_first_keyframe_cam_transform_.inverse()
                                         * orb_pose_unscaled_cam_to_first_keyframe_cam
                                         * base_link_to_camera_transform_no_translation_.inverse();

        tf::Transform odom_to_pose_scaled_out = scaleTF ( odom_to_pose_out , scale_ );

        // construct key frame in odom frame
        tf::Transform odom_to_first_keyframe_unscaled = odom_to_second_keyframe_base_transform_
                * base_link_to_camera_transform_no_translation_
                * second_keyframe_cam_to_first_keyframe_cam_transform_.inverse();

        tf::Transform odom_to_first_keyframe_scaled = scaleTF ( odom_to_first_keyframe_unscaled, scale_ );

        // odom to second_keyframe_base_link
        br_.sendTransform ( tf::StampedTransform ( odom_to_second_keyframe_base_transform_, t, "odom", "/second_keyframe_base_link" ) );

        // second_keyframe_base_link to second_keyframe_cam
        br_.sendTransform ( tf::StampedTransform ( base_link_to_camera_transform_no_translation_, t, "/second_keyframe_base_link", "/second_keyframe_cam" ) );

        // second_keyframe_cam to first_keyframe_cam
        br_.sendTransform ( tf::StampedTransform ( second_keyframe_cam_to_first_keyframe_cam_transform_.inverse(), t, "/second_keyframe_cam", "/first_keyframe_cam" ) );

        // first_keyframe_cam to first_keyframe_base_link
        br_.sendTransform ( tf::StampedTransform ( base_link_to_camera_transform_no_translation_.inverse(), t, "/first_keyframe_cam", "/first_keyframe_base_link" ) );

        // first_keyframe_cam to orb_pose_unscaled_cam
        br_.sendTransform ( tf::StampedTransform ( orb_pose_unscaled_cam_to_first_keyframe_cam, t, "/first_keyframe_cam", "/orb_pose_unscaled_cam" ) );

        // orb_pose_unscaled_cam to orb_pose_unscaled
        br_.sendTransform ( tf::StampedTransform ( base_link_to_camera_transform_no_translation_.inverse(), t, "/orb_pose_unscaled_cam", "/orb_pose_unscaled" ) );

        // send orb_pose scaled
        br_.sendTransform ( tf::StampedTransform ( odom_to_pose_scaled_out , t, "odom", "/orb_pose_scaled" ) );

        writePoseOutUnscaled ( odom_to_pose_out, t );
        writePoseOutScaled ( pose_out_unscaled_, scale_ ); //scale_init already covered in the tf's

        // Copy map into ros point cloud
        image_counter++;
        if ( image_counter>30 ) {
            image_counter=0;
            const std::vector<ORB_SLAM2::MapPoint *> &point_cloud = mpSLAM_->mpMap->GetAllMapPoints();
            // TODO: make efficient (use mpSLAM->GetTrackedMapPoints() to get most recent points)
            pc_.points.clear();
            pc_.header.stamp = t;

            for ( size_t i = 0; i < point_cloud.size(); i++ ) {
                if ( point_cloud[i]->isBad() /* or spRefMPs.count(vpMPs[i])*/ ) {
                    continue;
                }
                cv::Mat pos = point_cloud[i]->GetWorldPos();
                geometry_msgs::Point32 pp;
                pp.x = pos.at<float> ( 0 ) / ( scale_ * normalizer_ );
                pp.y = pos.at<float> ( 1 ) / ( scale_ * normalizer_ );
                pp.z = pos.at<float> ( 2 ) / ( scale_ * normalizer_ );

                pc_.points.push_back ( pp );
            }
        }
    }
}

void ImageGrabber::writePoseOutUnscaled ( tf::Transform &transform, ros::Time &stamp ) {
    pose_out_unscaled_.header.stamp = stamp;

    tf::Quaternion pose_orientation = transform.getRotation();
    tf::Vector3 pose_origin = transform.getOrigin();

    pose_out_unscaled_.pose.orientation.x = pose_orientation.getX();
    pose_out_unscaled_.pose.orientation.y = pose_orientation.getY();
    pose_out_unscaled_.pose.orientation.z = pose_orientation.getZ();
    pose_out_unscaled_.pose.orientation.w = pose_orientation.getW();
    pose_out_unscaled_.pose.position.x = pose_origin.getX();
    pose_out_unscaled_.pose.position.y = pose_origin.getY();
    pose_out_unscaled_.pose.position.z = pose_origin.getZ();
}
void ImageGrabber::writePoseOutScaled ( geometry_msgs::PoseStamped &pose, double &scale ) {
    pose_out_scaled_.header.stamp = pose.header.stamp;

    pose_out_scaled_.pose.orientation = pose.pose.orientation;
    pose_out_scaled_.pose.position.x = pose.pose.position.x / scale;
    pose_out_scaled_.pose.position.y = pose.pose.position.y / scale;
    pose_out_scaled_.pose.position.z = pose.pose.position.z / scale;

    poses_.push_back ( pose_out_scaled_ );
}
void ImageGrabber::updatePathArray () {
    scaled_path_.header.stamp = ros::Time::now();
    scaled_path_.poses = poses_;
}

bool ImageGrabber::isPointCloudValid() {
    const std::vector<ORB_SLAM2::MapPoint *> &point_cloud = mpSLAM_->mpMap->GetAllMapPoints();
    return point_cloud.size() != 0;
}


double ImageGrabber::getAverageDepth () {

    double tot_z = 0;
    int counter = 0;
    const std::vector<ORB_SLAM2::MapPoint *> &point_cloud = mpSLAM_->mpMap->GetAllMapPoints();

    for ( size_t i = 0; i < point_cloud.size(); i++ ) {
        if ( point_cloud[i]->isBad() ) continue;

        cv::Mat p = point_cloud[i]->GetWorldPos();

        double l = p.dot ( p );
        tot_z += sqrt ( l );
        counter++;
    }

    float out = tot_z / counter;
}

void ImageGrabber::setScale ( std_msgs::Float32 msg ) {
    isPathTracking_ = true;
    scale_ = msg.data;
};

tf::Transform ImageGrabber::cvMatToTF ( cv::Mat Tcw ) {
    tf::Transform cam_to_first_keyframe_transform;
    // invert since Tcw (transform from world to camera)
    cv::Mat pose = Tcw.inv();

    //Extract transformation from the raw orb SLAM pose
    tf::Vector3 origin;
    //tf::Quaternion transform_quat;
    tf::Matrix3x3 transform_matrix;

    origin.setValue ( pose.at<float> ( 0, 3 ), pose.at<float> ( 1, 3 ), pose.at<float> ( 2, 3 ) );

    transform_matrix.setValue ( pose.at<float> ( 0, 0 ), pose.at<float> ( 0, 1 ), pose.at<float> ( 0, 2 ),
                                pose.at<float> ( 1, 0 ), pose.at<float> ( 1, 1 ), pose.at<float> ( 1, 2 ),
                                pose.at<float> ( 2, 0 ), pose.at<float> ( 2, 1 ), pose.at<float> ( 2, 2 ) );

    //transform_matrix.getRotation(transform_quat);
    cam_to_first_keyframe_transform.setOrigin ( origin );
    cam_to_first_keyframe_transform.setBasis ( transform_matrix );

    return cam_to_first_keyframe_transform;
}

void ImageGrabber::navCallback ( ardrone_autonomy::Navdata msg ) {
    if ( is_started_ ) return;   // we already have the yaw error correction so this function is very slim after initial yaw correction

    if ( msg.altd > 0 && not is_started_ ) { //the yaw jump occurs exactly when the first altd message arrives + make sure it gets called only once
        is_started_ = true;

        tf::StampedTransform yaw_after_jump_tf;

        try {
            tf_listener_.lookupTransform ( "/ardrone_base_link", "odom", ros::Time ( 0 ), yaw_after_jump_tf );
        } catch ( tf::LookupException e ) {
            ROS_WARN ( "Failed to lookup transformation after yaw jump! Do not trust the direction of flight!" );
        }

        tf::Transform yaw_correction_transform = yaw_before_jump_tf_ * yaw_after_jump_tf;
        yaw_correction_tf_ = yaw_correction_transform;

        return;
    }

    try {
        tf_listener_.lookupTransform ( "odom", "/ardrone_base_link", ros::Time ( 0 ), yaw_before_jump_tf_ );   //to make sure we have the latest coordinate frame before the jump
    } catch ( tf::LookupException e ) {
        ROS_WARN ( "failed to lookup ardrone_base_link - is the driver running?" );
    }
}

void ImageGrabber::initializeTFTree ( cv::Mat Tcw ) {
    try {
        tf_listener_.lookupTransform ( "odom", "/ardrone_base_link", ros::Time ( 0 ), odom_to_second_keyframe_base_transform_ );
        tf_listener_.lookupTransform ( "/ardrone_base_link", "/ardrone_base_frontcam", ros::Time ( 0 ), base_link_to_camera_transform_ );

        odom_to_second_keyframe_base_transform_.setData ( odom_to_second_keyframe_base_transform_ * yaw_correction_tf_.inverse() );
        base_link_to_camera_transform_.setData ( yaw_correction_tf_ * base_link_to_camera_transform_ );

        second_keyframe_cam_to_first_keyframe_cam_transform_ = cvMatToTF ( Tcw );

        odom_to_second_keyframe_base_transform_.setOrigin ( tf::Vector3 ( 0, 0, 0 ) );

        second_keyframe_cam_to_first_keyframe_cam_transform_ = scaleTF ( second_keyframe_cam_to_first_keyframe_cam_transform_, normalizer_ );

        base_link_to_camera_transform_no_translation_ = base_link_to_camera_transform_;
        base_link_to_camera_transform_no_translation_.setOrigin ( tf::Vector3 ( 0.0, 0.0, 0.0 ) );
        pose_init_ = true;

    } catch ( tf::LookupException e ) {
        cout << e.what() << endl;
        ROS_WARN("Initialization of transformation chain failed!");
    }
}

tf::Transform ImageGrabber::scaleTF ( tf::Transform T, double normalizer ) {
    T.setOrigin ( T.getOrigin() / normalizer );
    return T;
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
