/*
 * This is a ROS AR-Drone adapter for the ORB-SLAM package
 */


#include <iostream>
#include <algorithm>
#include <fstream>
#include <random>
#include <chrono>
#include <time.h>
#include <math.h>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <std_msgs/Float32.h>
#include <ardrone_autonomy/Navdata.h>

#include<opencv2/core/core.hpp>

#include <System.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

class ImageGrabber {
    int image_counter=0;
public:
    ImageGrabber ( ORB_SLAM2::System *pSLAM ) : mpSLAM_ ( pSLAM ), pc_() {
        pc_.header.frame_id = "/first_keyframe_cam";
        pose_out_.header.frame_id = "odom";
    }
    void grabImage ( const sensor_msgs::ImageConstPtr &msg );
    geometry_msgs::TransformStamped toTFStamped ( tf2::Transform in , ros::Time t, string frame_id, string child_frame_id );
    bool isPointCloudValid();

    void setScale ( std_msgs::Float32 msg ) {
        scale_ = msg.data;
    };
    tf::Transform cvMatToTF ( cv::Mat Tcw );
    double getAverageDepth ();
    void navCallback ( ardrone_autonomy::Navdata msg ) ;

    ORB_SLAM2::System *mpSLAM_;

    bool pose_init_ = false;
    bool pc_init_ = false;
    bool debug_mode_ = true;
    bool is_started_ = false;
    int tracking_state_=0;

    double scale_ = 1;
    double scale_init_ = 1;

    sensor_msgs::PointCloud pc_;
    geometry_msgs::PoseWithCovarianceStamped pose_out_;

    tf::TransformListener tf_listener_;

    tf::TransformBroadcaster br_;

    tf::StampedTransform base_link_to_camera_transform_;
    tf::StampedTransform odom_to_second_keyframe_base_transform_;
    tf::Transform second_keyframe_cam_to_first_keyframe_cam_transform_;
    tf::StampedTransform base_link_to_camera_transform_no_translation_;
    tf::StampedTransform yaw_before_jump_tf_;
    tf::Transform yaw_correction_tf_ = tf::Transform(tf::Quaternion(0,0,0,1));
    
};

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
    ros::Publisher pose_pub = nodeHandler.advertise<geometry_msgs::PoseWithCovarianceStamped> ( "/orb/pose_unscaled", 2 );

    ros::Rate loop_rate ( 30 );
    int counter = 0;

    while ( ros::ok() ) {
        ros::spinOnce();
        if ( igb.pc_init_  && counter % 30 == 0) {
            pointcloud_pub.publish ( igb.pc_ ); //publish at 1 Hz
        }

        if (igb.pose_init_ && igb.tracking_state_ == 2) {
            pose_pub.publish ( igb.pose_out_ );
        }

        counter++;
        loop_rate.sleep();
    }

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
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
    //     Main slam routine. Extracts new pose
    // (from ORB_SLAM2 API)
    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat Tcw = mpSLAM_->TrackMonocular ( cv_ptr->image, cv_ptr->header.stamp.toSec() );
    tracking_state_ = mpSLAM_->GetTrackingState();
    
    ros::Time t = msg->header.stamp;


    // TODO: adapt
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
    //for the first time). It is set to be the transformation from odom to ardrone_base_frontcam (published by the driver).
    //Odom is defined relative to the real world in a way that g(ravitation) is facing in negative z direction. This means
    //that 0 roll and 0 pitch ALWAYS means that the drone is in the horizontal plane of the earth. Since the IMU is not
    //able to make a reasonable yaw estimate itself, it uses the (arbitrary) direction of the drone at the time point where
    //the plug on the drone is connected as 0 yaw. This value will not change until the drone itself is shut off (regardless
    //of the driver node being turned on and off).
    //
    //Camera frames for some reason always come in a way such that the z axis is pointing forwards while the y axis is
    //facing downwards. This has to be corrected through a manual rotation such that eventually the orb pose values
    //which are beeing publsihed to a PoseWithCovarianceStamped topic represent the correct pose within the odom frame.
    //See the rqt_tf_tree for further calrification on the single transformations this code is performing.
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // if points can be tracked then broadcast the pose
    if ( not Tcw.empty() ) {

        if ( not pose_init_ ) { // TODO: find first keyframe from first pose by
            //Initialization - set link between 'odom' and 'first_keyframe' frames
            pose_init_ = true;

            try {
                tf_listener_.lookupTransform ( "odom", "/ardrone_base_link", ros::Time ( 0 ), odom_to_second_keyframe_base_transform_ );
                tf_listener_.lookupTransform ( "/ardrone_base_link", "/ardrone_base_frontcam", ros::Time ( 0 ), base_link_to_camera_transform_ );
                
                odom_to_second_keyframe_base_transform_.setData(odom_to_second_keyframe_base_transform_ * yaw_correction_tf_.inverse());
                base_link_to_camera_transform_.setData(yaw_correction_tf_ * base_link_to_camera_transform_);
                
                second_keyframe_cam_to_first_keyframe_cam_transform_ = cvMatToTF ( Tcw );

                odom_to_second_keyframe_base_transform_.setOrigin ( tf::Vector3 ( 0, 0, 0 ) );

            } catch ( tf::LookupException e ) {
                cout << e.what() << endl;
            }

            base_link_to_camera_transform_no_translation_ = base_link_to_camera_transform_;
            base_link_to_camera_transform_no_translation_.setOrigin ( tf::Vector3 ( 0.0, 0.0, 0.0 ) );
        }
        
        if (not pc_init_ && isPointCloudValid()) {
            scale_init_ = getAverageDepth();
            pc_init_ = true;
        }
            

        //tf::Transform cam_to_first_keyframe_transform = cvMatToTF ( Tcw );
        tf::Transform orb_pose_unscaled_cam_to_first_keyframe_cam = cvMatToTF ( Tcw );
        tf::Vector3 rescaled_origin = orb_pose_unscaled_cam_to_first_keyframe_cam.getOrigin() / scale_init_;
        orb_pose_unscaled_cam_to_first_keyframe_cam.setOrigin ( rescaled_origin );

        tf::Transform pose_out = odom_to_second_keyframe_base_transform_
                                 * base_link_to_camera_transform_no_translation_
                                 * second_keyframe_cam_to_first_keyframe_cam_transform_.inverse()
                                 * orb_pose_unscaled_cam_to_first_keyframe_cam
                                 * base_link_to_camera_transform_no_translation_.inverse();

        if ( debug_mode_ ) {
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
        } else
            br_.sendTransform ( tf::StampedTransform ( pose_out , t, "odom", "/orb_pose_unscaled" ) );

        //generate pose for robot_localization EKF sensor fusion
        //the pose is simply generated from the above derived transformations
        pose_out_.header.stamp = t;

        tf::Quaternion pose_orientation = pose_out.getRotation();
        tf::Vector3 pose_origin = pose_out.getOrigin();
        pose_out_.pose.pose.orientation.x = pose_orientation.getX();
        pose_out_.pose.pose.orientation.y = pose_orientation.getY();
        pose_out_.pose.pose.orientation.z = pose_orientation.getZ();
        pose_out_.pose.pose.orientation.w = pose_orientation.getW();
        pose_out_.pose.pose.position.x = pose_origin.getX();
        pose_out_.pose.pose.position.y = pose_origin.getY();
        pose_out_.pose.pose.position.z = pose_origin.getZ();

        //covariance
        for ( auto & x : pose_out_.pose.covariance ) {
            x = 0.0;
        }

        pose_out_.pose.covariance[0] = 0.1;
        pose_out_.pose.covariance[7] = 0.1;
        pose_out_.pose.covariance[14] = 0.1;
        pose_out_.pose.covariance[21] = 0.1;
        pose_out_.pose.covariance[28] = 0.1;
        pose_out_.pose.covariance[35] = 0.1;
    }

    // Copy map into ros point cloud
    image_counter++;
    if(image_counter>30){
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
            pp.x = pos.at<float> ( 0 ) / ( scale_ * scale_init_ );
            pp.y = pos.at<float> ( 1 ) / ( scale_ * scale_init_ );
            pp.z = pos.at<float> ( 2 ) / ( scale_ * scale_init_ );
            
            pc_.points.push_back ( pp );
        }
    }

}

bool ImageGrabber::isPointCloudValid()
{
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
        
        double l = p.dot(p);
        tot_z += sqrt(l);
        //tot_z += point_cloud[i]->GetWorldPos().at<float> ( 2 );
        counter++;
    }

    float out = tot_z / counter;
}

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

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
