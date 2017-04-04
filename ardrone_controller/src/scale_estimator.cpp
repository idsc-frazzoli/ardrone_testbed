#include <queue>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>

#include <ardrone_autonomy/Navdata.h>

#include <tf/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath//Vector3.h>
#include <tf/tfMessage.h>

using namespace std;

class ScaleEstimator {
    public:
    float scale = 1; // has units m^-1
    bool isStarted = false;

    tf::Vector3 curr_orb_position;
    tf::Vector3 filt_position;
    tf::Quaternion orb_orientation;
    tf::Quaternion yaw_correction_quat;
    tf::StampedTransform yaw_before_jump_tf;
    tf::TransformListener tf_listener;

    vector<double> scale_vector;
    vector<double> orbAverages;
    vector<double> altAverages;

    vector<ardrone_autonomy::Navdata> nav_data_queue;
    vector<geometry_msgs::PoseWithCovarianceStamped> orb_data_queue;
    vector<geometry_msgs::PoseStamped> poses;

    geometry_msgs::PoseWithCovarianceStamped newest_orb_msg;
    geometry_msgs::PoseWithCovarianceStamped filt_pose;
    nav_msgs::Path scaled_path;

    double get_average_z ( vector<ardrone_autonomy::Navdata> &queue ) {
        double data = 0;
        int counter = 0;
        for ( counter ; counter < queue.size() ; counter++ ) {
            data += queue[counter].altd;
        }
        return data / counter / 1000;
    }
    double get_average_z ( vector<geometry_msgs::PoseWithCovarianceStamped> &queue ) {
        double data = 0;
        int counter = 0;
        for ( counter ; counter < queue.size() ; counter++ ) {
            data += queue[counter].pose.pose.position.z;
        }
        return data / counter;
    }

    void reset_all() {
        nav_data_queue.clear();
        orb_data_queue.clear();
    }
    void process_queue() {
        orbAverages.push_back ( get_average_z ( orb_data_queue ) );
        altAverages.push_back ( get_average_z ( nav_data_queue ) );

        newest_orb_msg = orb_data_queue.back();
    }

    void estimate_scale() {

        if ( scale_vector.size() > 120 ) return; //enough scales for accuracy

        if ( orbAverages.size() != altAverages.size() ) return;

        cout << "reestimating scale..." << endl;

        double deltaX = ( ( orbAverages[0] + orbAverages[1] + orbAverages[2] ) -
                          ( orbAverages[3] + orbAverages[4] + orbAverages[5] ) ) / 3 ;
        double deltaY = ( ( altAverages[0] + altAverages[1] + altAverages[2] ) -
                          ( altAverages[3] + altAverages[4] + altAverages[5] ) ) / 3 ;
        double naive_scale = deltaX / deltaY;

        if ( naive_scale == naive_scale && naive_scale > 0 ) { //covers for nan/inf and bad readings (negative scale)
            cout << "pushing back scale vector" << endl;
            scale_vector.push_back ( naive_scale );   //naive scale
            std::sort ( scale_vector.begin() ,scale_vector.end() );
        }

        if ( scale_vector.size() > 50 ) {
            double scale_accumulated = 0.0;
            double samples = 16;
            for ( int index = ( ( scale_vector.size() / 2 ) - ( samples / 2 ) ) ; index < ( ( scale_vector.size() / 2 ) + ( samples / 2 ) ) ; index++ ) {
                scale_accumulated += scale_vector[index];
            }
            scale = scale_accumulated / samples;
        } else {
            if ( scale_vector.size() == 0 ) {
                return;
            }
            scale = scale_vector[scale_vector.size() / 2]; // simple median for first few readings
        }
        cout << "naive scale: " << scale << endl;
    }
    void estimate_pose() {
        if ( not isStarted ) return;

        //cout << "orb: " << orb_orientation.getX() << endl << orb_orientation.getY() << endl << orb_orientation.getZ() << endl << orb_orientation.getW() << endl;
        //cout << "yaw: " << yaw_correction_quat.getX() << endl << yaw_correction_quat.getY() << endl << yaw_correction_quat.getZ() << endl << yaw_correction_quat.getW() << endl;

        filt_pose.header.frame_id = "odom";
        filt_pose.header.stamp = ros::Time::now();

        orb_orientation.setX ( newest_orb_msg.pose.pose.orientation.x );
        orb_orientation.setY ( newest_orb_msg.pose.pose.orientation.y );
        orb_orientation.setZ ( newest_orb_msg.pose.pose.orientation.z );
        orb_orientation.setW ( newest_orb_msg.pose.pose.orientation.w );

        orb_orientation =  orb_orientation * yaw_correction_quat; // TODO: Something still seems to be wrong with the yaw correction - we have to figure out exactly how to to the tf's and double check the implementation

        // orientation
        filt_pose.pose.pose.orientation.x = orb_orientation.getX();
        filt_pose.pose.pose.orientation.y = orb_orientation.getY();
        filt_pose.pose.pose.orientation.z = orb_orientation.getZ();
        filt_pose.pose.pose.orientation.w = orb_orientation.getW();

        // position
        filt_pose.pose.pose.position.x = newest_orb_msg.pose.pose.position.x / scale;
        filt_pose.pose.pose.position.y = newest_orb_msg.pose.pose.position.y / scale;
        filt_pose.pose.pose.position.z = newest_orb_msg.pose.pose.position.z / scale; //TODO: Test if it is better to use the altd message for altitude

        for ( auto & x : filt_pose.pose.covariance ) {
            x = 0.0;
        }
        filt_pose.pose.covariance[0] = newest_orb_msg.pose.covariance[0] / scale;
        filt_pose.pose.covariance[7] = newest_orb_msg.pose.covariance[7] / scale;
        filt_pose.pose.covariance[14] = newest_orb_msg.pose.covariance[14] / scale;
        filt_pose.pose.covariance[21] = newest_orb_msg.pose.covariance[21] / scale;
        filt_pose.pose.covariance[28] = newest_orb_msg.pose.covariance[28] / scale;
        filt_pose.pose.covariance[35] = newest_orb_msg.pose.covariance[35] / scale;

        cout << "SCALED POSITION: " << endl
        << "X: " << filt_pose.pose.pose.position.x << endl
        << "Y: " << filt_pose.pose.pose.position.y << endl 		//debug
        << "Z: " << filt_pose.pose.pose.position.z << endl;


        geometry_msgs::PoseStamped path_pose;
        path_pose.header.stamp = ros::Time::now();
        path_pose.header.frame_id = "odom";

        // orientation
        path_pose.pose.orientation.x = filt_pose.pose.pose.orientation.x;
        path_pose.pose.orientation.y = filt_pose.pose.pose.orientation.y;
        path_pose.pose.orientation.z = filt_pose.pose.pose.orientation.z;
        path_pose.pose.orientation.w = filt_pose.pose.pose.orientation.w;

        // position
        path_pose.pose.position.x = newest_orb_msg.pose.pose.position.x / scale;
        path_pose.pose.position.y = newest_orb_msg.pose.pose.position.y / scale;
        path_pose.pose.position.z = newest_orb_msg.pose.pose.position.z / scale; //TODO: Test if it is better to use the altd message for altitude
        //TODO: Implement something that takes the nav_data.altd message if it is available (drone at maximum height of 4m) or elsewise orb.z / scale but calculate the offset in the z coordinate (after takeoff) anyway!!

        poses.push_back ( path_pose );
    }

    void update_path_array () {
        geometry_msgs::PoseStamped pose_array[poses.size()];
        for ( int i = 0; i < poses.size() ; i++ ) {
            pose_array[i] = poses.at ( i );
        }
        scaled_path.header.frame_id = "odom";
        scaled_path.header.stamp = ros::Time::now();
        scaled_path.poses = poses;
    }

    void orb_callback ( geometry_msgs::PoseWithCovarianceStamped msg ) {
        orb_data_queue.push_back ( msg );
    }
    void nav_callback ( ardrone_autonomy::Navdata msg ) {
        if ( isStarted ) {
            nav_data_queue.push_back ( msg );
            return;   // we already have the yaw error correction so this function is very slim after initial yaw correction
        }
        if ( msg.altd > 0 && not isStarted ) { //the yaw jump occurs exactly when the first altd message arrives + make sure it gets called only once
            isStarted = true;

            tf::StampedTransform yaw_after_jump_tf;
            try {
                tf_listener.lookupTransform ( "/ardrone_base_link", "odom", ros::Time ( 0 ), yaw_after_jump_tf );
            } catch ( tf::LookupException e ) {
                ROS_WARN ( "Failed to lookup transformation after yaw jump! Do not trust the direction of flight!" );

            }

            tf::Transform yaw_correction_transform = yaw_before_jump_tf * yaw_after_jump_tf;
            yaw_correction_quat = yaw_correction_transform.getRotation();

            return;
        }
        try {
            tf_listener.lookupTransform ( "odom", "/ardrone_base_link", ros::Time ( 0 ), yaw_before_jump_tf ); //we keep updating yaw_correction_before_jump_tf as long as there is no jump
        }                                                                                                             //to make sure we have the latest coordinate frame before the jump
        catch ( tf::LookupException e ) {
            cout << "failed to lookup ardrone_base_link - is the driver running?" << endl;
        }
    }

};

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "scale_estimator" );
    ros::NodeHandle nh;

    // estimates scale
    ScaleEstimator scale_est;


    // subscribe to orb pose and accelerometer data from imu
    ros::Subscriber orb_sub = nh.subscribe ( "/orb/pose_unscaled", 10, &ScaleEstimator::orb_callback, &scale_est );
    ros::Subscriber nav_sub = nh.subscribe ( "/ardrone/navdata", 50,  &ScaleEstimator::nav_callback, &scale_est );


    // publish scale and filtered orb pose
    ros::Publisher filt_orb_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ( "/ardrone/pose_scaled",2 );
    ros::Publisher orb_scale_pub = nh.advertise<std_msgs::Float32> ( "/orb/scale", 2 );
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path> ( "/ardrone/path",2 );

    int rate = 60;
    ros::Rate loop_rate ( rate );
    cout << "Started scale estimation with rate " << rate << " Hz" << endl;

    while ( ros::ok() ) {

        if ( scale_est.orb_data_queue.size() > 0 ) {

            ros::spinOnce();
            loop_rate.sleep();

            ros::spinOnce();
            loop_rate.sleep();

            scale_est.process_queue();

            if ( scale_est.orbAverages.size() > 5 ) {
                scale_est.estimate_scale();
                scale_est.estimate_pose();

                std_msgs::Float32 scale;
                scale.data= scale_est.scale;

                orb_scale_pub.publish ( scale_est.scale );
                filt_orb_pub.publish ( scale_est.filt_pose );
                scale_est.update_path_array();
                path_pub.publish ( scale_est.scaled_path );

                scale_est.orbAverages.clear();
                scale_est.altAverages.clear();

                scale_est.reset_all();
            }
        }
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; ;
