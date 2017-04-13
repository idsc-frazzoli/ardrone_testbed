/*
 * (c) 2017 
 * 
 * Authors: 
 * Daniel Gehrig (ETH Zurich)
 * Maximilian Goettgens (ETH Zurich)
 * Brian Paden (MIT)
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
        for ( int counter = 0 ; counter < queue.size() ; counter++ ) {
            data += queue[counter].pose.pose.position.z;
        }
        return data / queue.size();
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

        if ( scale_vector.size() > 40 ) return; //enough scales for accuracy

        if ( orbAverages.size() != altAverages.size() ) return; // TODO necessary?

        cout << "reestimating scale..." << endl;

        double deltaX = ( ( orbAverages[0] + orbAverages[1] + orbAverages[2] ) -
                          ( orbAverages[3] + orbAverages[4] + orbAverages[5] ) ) / 3 ;
        double deltaY = ( ( altAverages[0] + altAverages[1] + altAverages[2] ) -
                          ( altAverages[3] + altAverages[4] + altAverages[5] ) ) / 3 ;
        double naive_scale = deltaX / deltaY;

        if ( naive_scale == naive_scale && naive_scale > 0 ) { //covers for nan/inf and bad signal (negative scale)
            cout << "pushing back scale vector" << endl;
            scale_vector.push_back ( naive_scale );
            std::sort ( scale_vector.begin() ,scale_vector.end() ); //for median
        }

        if ( scale_vector.size() > 10 ) {
            double scale_accumulated = 0.0;
            double samples = 8;
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

        //filt_pose.pose.covariance = newest_orb_msg.pose.covariance / scale;

        geometry_msgs::PoseStamped path_pose;
        path_pose.header.stamp = ros::Time::now();
        path_pose.header.frame_id = "odom";

        // orientation
        path_pose.pose.orientation.= filt_pose.pose.pose.orientation;

        // position
        path_pose.pose.position.x = newest_orb_msg.pose.pose.position.x / scale; //TODO: Test if it is better to use the altd message for altitude
        path_pose.pose.position.y = newest_orb_msg.pose.pose.position.y / scale;
        path_pose.pose.position.z = newest_orb_msg.pose.pose.position.z / scale;//TODO: Implement something that takes the nav_data.altd message if it is available (drone at maximum height of 4m) or elsewise orb.z / scale but calculate the offset in the z coordinate (after takeoff) anyway!!

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
            ROS_WARN ( "failed to lookup ardrone_base_link - is the driver running?" );
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

    int rate = 20;
    ros::Rate loop_rate ( rate );
    cout << "Started scale estimation with rate " << rate << " Hz" << endl;

    while ( ros::ok() ) {

        ros::spinOnce();
        loop_rate.sleep();

        if ( scale_est.orb_data_queue.size() > 0 ) {
            scale_est.process_queue();

            if ( scale_est.orbAverages.size() > 5 ) {
                scale_est.estimate_scale();

                orb_scale_pub.publish ( scale_est.scale );
                filt_orb_pub.publish ( scale_est.filt_pose );
                scale_est.update_path_array();
                path_pub.publish ( scale_est.scaled_path );

                scale_est.orbAverages.clear();
                scale_est.altAverages.clear();

                scale_est.reset_all();
            }
        }

        scale_est.estimate_pose();
        orb_scale_pub.publish ( scale_est.filt_pose );

    }

    return 0;
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; ;
