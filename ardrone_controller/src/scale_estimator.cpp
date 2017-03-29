#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ardrone_autonomy/Navdata.h>

#include <std_msgs/Float32.h>
//#include "reset_scale_estimator.h"

#include <tf/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath//Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>

using namespace std;

// directly copied from tum
class ScaleStruct {
public:
     tf::Vector3 ptam;
     tf::Vector3 imu;
     tf::Vector3 realDisplacement;
     double ptamNorm;
     double imuNorm;
     double alphaSingleEstimate;
     double pp, ii, pi;

     double stdPTAM = 0.2;
     double stdIMU = 0.01;


     static inline double computeEstimator ( double spp, double sii, double spi, double stdDevPTAM = 0.01, double stdDevIMU = 0.2 ) {
          double sII = stdDevPTAM * stdDevPTAM * sii;
          double sPP = stdDevIMU * stdDevIMU * spp;
          double sPI = stdDevIMU * stdDevPTAM * spi;

          int sgn = copysign ( 1, spi );

          double tmp = ( sII-sPP ) * ( sII-sPP ) + 4*sPI*sPI;
          if ( tmp <= 0 ) {
               tmp = 1e-5;	// numeric issues
          }
          return 0.5* ( - ( sII-sPP ) +sgn*sqrt ( tmp ) ) / ( stdDevIMU * stdDevIMU * spi );

     }

     inline bool isInlier() {

          return true;
     }

     static inline tf::Vector3 computeRealDisplacement ( tf::Vector3 ptam,tf::Vector3 imu, double alpha, double stdDevPTAM = 0.01, double stdDevIMU = 0.2 ) {
          double d = alpha * alpha * stdDevIMU * stdDevIMU + stdDevPTAM * stdDevPTAM;
          double f1 = alpha * stdDevIMU * stdDevIMU / d;
          double f2 = stdDevPTAM * stdDevPTAM / d;

          return ptam * f1 + imu * f2;
     }

     inline ScaleStruct ( tf::Vector3 ptamDist, tf::Vector3 imuDist, double stdDevPTAM, double stdDevIMU ) {
          ptam = ptamDist;
          imu = imuDist;
          pp = ptam[0]*ptam[0] + ptam[1]*ptam[1] + ptam[2]*ptam[2];
          ii = imu[0]*imu[0] + imu[1]*imu[1] + imu[2]*imu[2];
          pi = imu[0]*ptam[0] + imu[1]*ptam[1] + imu[2]*ptam[2];

          ptamNorm = sqrt ( pp );
          imuNorm = sqrt ( ii );

          alphaSingleEstimate = computeEstimator ( pp,ii,pi, stdDevPTAM, stdDevIMU );

          realDisplacement = computeRealDisplacement ( ptam, imu, alphaSingleEstimate, stdDevPTAM, stdDevIMU );
     }

     inline bool operator < ( const ScaleStruct& comp ) const {
          return alphaSingleEstimate < comp.alphaSingleEstimate;
     }
};
//

class ScaleEstimator {
public:
     float scale = 1; // has units m^-1
      
     // used for correcting yaw switch
     tf::Transform T_correction;
     tf::StampedTransform T_prev, T_curr, T_init;
     ros::Time drone_state_time;
     bool quat_init = false;
     bool correction_made = false;

     // tuning parameters
     float orb_noise=0.01, nav_noise=0.2; // noise params must be tuned (init vals from tum)
     float dot_prod_tol = 0.05;
     float ratio = 0.2;
     int scale_samples = 10;
     float log_likelihood = 1e7;
     int max_scale_vector_size = 20;

     tf::Vector3 orb_displacement = tf::Vector3 ( 0,0,0 );
     tf::Vector3 nav_data_displacement = tf::Vector3 ( 0,0,0 );

     tf::Vector3 init_displacement = tf::Vector3 ( 0,0,0 );

     bool fixed_scale = false;
     bool orb_init = true;

     geometry_msgs::PoseWithCovarianceStamped filt_pose;

     tf::TransformListener tf_listener;
     tf::TransformBroadcaster tf_broadcaster;

     queue<geometry_msgs::PoseWithCovarianceStamped> orb_data_queue;
     vector<ScaleStruct> scale_vector;
     
     
     ScaleEstimator()
     {
       T_correction.setIdentity();
       T_prev.setIdentity();
       T_curr.setIdentity();
       T_init.setIdentity();
     }

     void reset() {
          // reset dispacements
          nav_data_displacement = tf::Vector3 ( 0.0, 0.0, 0.0 );
          orb_displacement = tf::Vector3 ( 0.0, 0.0, 0.0 );

          // clear queue
          queue<geometry_msgs::PoseWithCovarianceStamped> empty;
          swap ( orb_data_queue, empty );
          scale_vector.clear();
     }

     bool hard_reset() {
          reset();

          // also reset scale etc.
          scale = 1;
          fixed_scale = false;
          scale_vector.clear();
          geometry_msgs::PoseWithCovarianceStamped empty_pose;
          filt_pose = empty_pose;
          return true;
     }

     void print_all() {
          cout << endl << endl;
          cout << "scale: " << "\t" << scale << endl;
          cout << "scaled pose x:" << "\t" << filt_pose.pose.pose.position.x  <<endl;
     }

     void process_queue() {

          ros::Time t_old, t_new;
          tf::StampedTransform old_odom_to_base_link, new_odom_to_base_link;
          geometry_msgs::PoseWithCovarianceStamped old_orb_msg, new_orb_msg;

          // process orb
          if ( orb_data_queue.size() < max_scale_vector_size ) {

               orb_init = false;
               return;
          } else {
               cout << "number of samples: " << orb_data_queue.size() << endl;
               orb_init = true;

               // get oldest orb msg
               old_orb_msg = orb_data_queue.front();
               orb_data_queue.pop();

               // work through all others
               while ( !orb_data_queue.empty() ) {
                    // get next orb msg
                    new_orb_msg = orb_data_queue.front();
                    orb_data_queue.pop();

                    // get times, orb displacements and transforms at these times
                    t_old = old_orb_msg.header.stamp;
                    t_new = new_orb_msg.header.stamp;

                    double dx, dy, dz;
                    dx = new_orb_msg.pose.pose.position.x - old_orb_msg.pose.pose.position.x;
                    dy = new_orb_msg.pose.pose.position.y - old_orb_msg.pose.pose.position.y;
                    dz = new_orb_msg.pose.pose.position.z - old_orb_msg.pose.pose.position.z;

                    orb_displacement = tf::Vector3 ( dx,dy,dz );

                    try {
                         tf_listener.lookupTransform ( "odom", "/ardrone_base_link_corrected", t_old, old_odom_to_base_link );
                         tf_listener.lookupTransform ( "odom", "/ardrone_base_link_corrected", t_new, new_odom_to_base_link );

                    } catch ( tf2::ExtrapolationException e ) {
                         cout << e.what() << endl;
                    }
		    
                    nav_data_displacement = new_odom_to_base_link.getOrigin() - old_odom_to_base_link.getOrigin();

                    // add new point estimate
                    ScaleStruct s = ScaleStruct ( orb_displacement, nav_data_displacement, orb_noise, nav_noise );

                    if ( s.isInlier() ) { // TODO maybe add condition
                         cout << "lambda: " << s.alphaSingleEstimate << endl;
                         cout << "x: " << s.ptam[0] << "\t" << s.ptam[1] << "\t" << s.ptam[2] << endl;
                         cout << "y: " << s.imu[0] << "\t" << s.imu[1] << "\t" << s.imu[2] << endl;
                         cout << "m: " << s.realDisplacement[0] << "\t" << s.realDisplacement[1] << "\t" << s.realDisplacement[2] << endl;
                         cout << "|mu|* lambda: " << s.realDisplacement.length() * s.alphaSingleEstimate << "\t" << "|mu|: " << s.realDisplacement.length() << endl;
                         cout << "|x|: " << s.ptam.length() << "\t" << "|y|: " << s.imu.length() << endl;
                         cout << endl;
                         scale_vector.push_back ( s );
                    }

                    old_orb_msg = new_orb_msg;

               }

               // calculate new scale, displacements and likelihood for the data.
               float scale_est = estimate_scale ( scale_vector );
               vector<tf::Vector3> displacement_est = estimate_displacements ( scale_vector, scale_est );
               float log_likelihood_est = estimate_likelihood ( scale_vector, scale_est, displacement_est );

               // replace scale if the log likelihood is better
               if ( log_likelihood_est < log_likelihood ) {
                    log_likelihood = log_likelihood_est;
                    scale = scale_est;
               }


               cout << "new batch estimate: " << endl;
               cout << "\t cur scale: " << scale << endl;
               cout << "\t new scale: " << scale_est << endl;
               cout << "\t cur likelihood: " << log_likelihood_est << endl;
               cout << "\t old likelihood: " << log_likelihood << endl;
               cout << "scaled pos: " << new_orb_msg.pose.pose.position.x / scale << "\t"
                    << new_orb_msg.pose.pose.position.y / scale << "\t" << new_orb_msg.pose.pose.position.z / scale << endl;



               // clear the queue
               reset();

          }
     }

     vector<tf::Vector3> estimate_displacements ( vector<ScaleStruct> scale_vector, float scale_est ) {
          vector<tf::Vector3> displacement_vector;
          for ( int i=0; i < scale_vector.size(); i++ ) {
               tf::Vector3 x = scale_vector[i].ptam;
               tf::Vector3 y = scale_vector[i].imu;

               tf::Vector3 mu = ScaleStruct::computeRealDisplacement ( x, y, scale_est, orb_noise, nav_noise );
               displacement_vector.push_back ( mu );
          }
          return displacement_vector;
     }

     float estimate_likelihood ( vector<ScaleStruct> scale_vector, float scale_est, vector<tf::Vector3> displacement_est ) {
          float likelihood_func = 0;
          for ( int i=0; i < displacement_est.size(); i++ ) {
               tf::Vector3 diff_x = scale_vector[i].ptam - scale_est * displacement_est[i];
               tf::Vector3 diff_y = scale_vector[i].imu - displacement_est[i];
               likelihood_func += diff_x.length2() / ( 2*orb_noise * orb_noise ) + diff_y.length2() / ( 2*nav_noise * nav_noise );
          }

          return likelihood_func;
     }

     float estimate_scale ( vector<ScaleStruct> scale_vector ) {
          float s_xx, s_yy, s_xy;
          s_xx = s_yy = s_xy = 0;

          for ( int i=0; i<scale_vector.size(); i++ ) {
               s_xx += scale_vector[i].pp;
               s_yy += scale_vector[i].ii;
               s_xy += scale_vector[i].pi;

          }

          return ScaleStruct::computeEstimator ( s_xx, s_yy, s_xy, orb_noise, nav_noise );

     }



     void set_orb_pose() {
          if ( orb_data_queue.empty() ) return;

          geometry_msgs::PoseWithCovarianceStamped newest_orb_msg = orb_data_queue.back();

          filt_pose.header.stamp = newest_orb_msg.header.stamp;
          filt_pose.header.frame_id = "odom";

          // orientation
          filt_pose.pose.pose.orientation = newest_orb_msg.pose.pose.orientation;

          // position
          filt_pose.pose.pose.position.x = newest_orb_msg.pose.pose.position.x /scale;
          filt_pose.pose.pose.position.y = newest_orb_msg.pose.pose.position.x /scale;
          filt_pose.pose.pose.position.z = newest_orb_msg.pose.pose.position.z /scale;

     }


     void publish_scaled_pose ( ros::Publisher &publisher ) {
          publisher.publish ( filt_pose );
     }

     void orb_callback ( geometry_msgs::PoseWithCovarianceStamped msg ) {
          orb_data_queue.push ( msg );
     }

     void nav_callback ( ardrone_autonomy::Navdata navdata ) {
	  float deg_to_rad = 3.14159 / 180;
	  
	  // get current frame
	  try 
	  {
	    tf_listener.lookupTransform("odom", "/ardrone_base_link", navdata.header.stamp, T_curr);
	    T_curr.setOrigin(tf::Vector3(0,0,0));
	  }
	  catch (tf2::ExtrapolationException e) {cout << e.what() << endl;}
	  
	  
	  // initialize previous frame
	  if ( not quat_init) { T_init = T_prev = T_curr; quat_init = true;} 
	  
	  if (not correction_made) 
	  {
	    // calculate correction transformation
	    tf::Transform delta_T = T_curr.inverse() * T_prev;
	    
	    // if angle jump is too large then assign the correction transform
	    float angle_deg = delta_T.getRotation().getAngle() / deg_to_rad;
	    if (abs(angle_deg) > 5) { T_correction = T_curr.inverse() * T_init; correction_made = true;}
	    
	    T_prev = T_curr;
	  }
	 
	 // publish corrected frame
	 tf_broadcaster.sendTransform(tf::StampedTransform(T_correction, navdata.header.stamp, "/ardrone_base_link", "/ardrone_base_link_corrected"));
    }

void set_initial_nav_position ( tf::Vector3 pos ) {
          init_displacement = pos;
     }

};


int main ( int argc, char **argv )
{
     ros::init ( argc, argv, "scale_estimator" );
     ros::NodeHandle nh;

     // estimates scale
     ScaleEstimator scale_est;

     // reset command
     //ros::ServiceServer s = nh.advertiseService("reset_scale_estimator", &ScaleEstimator::hard_reset, &scale_est);

     // subscribe to orb pose and accelerometer data from imu
     ros::Subscriber orb_sub = nh.subscribe ( "/orb/pose_unscaled", 10, &ScaleEstimator::orb_callback, &scale_est );

     // get rotation due to magnetic field
     ros::Subscriber nav_sub = nh.subscribe ( "/ardrone/navdata", 10, &ScaleEstimator::nav_callback, &scale_est );

     // publish scale and filtered orb pose
     ros::Publisher filt_orb_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ( "/orb/pose_scaled",2 );

     int rate = 20;
     ros::Rate loop_rate ( rate );

     scale_est.reset();

     while ( ros::ok() ) {
          // work through all messages received
          ros::spinOnce();

          // computes displacements for nav_data msgs and orb poses
          scale_est.process_queue();

          // set latest orb pose
          scale_est.set_orb_pose();
          //scale_est.print_all();

          scale_est.publish_scaled_pose ( filt_orb_pub );

          loop_rate.sleep();
     }

     return 0;
}

     
     