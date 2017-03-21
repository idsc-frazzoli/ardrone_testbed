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
#include <tf/tfMessage.h>

using namespace std;

// directly copied from tum
class ScaleStruct
{
public:
	tf::Vector3 ptam;
	tf::Vector3 imu;
	double ptamNorm;
	double imuNorm;
	double alphaSingleEstimate;
	double pp, ii, pi;

	static inline double computeEstimator(double spp, double sii, double spi, double stdDevPTAM = 0.2, double stdDevIMU = 0.1)
	{
		double sII = stdDevPTAM * stdDevPTAM * sii;
		double sPP = stdDevIMU * stdDevIMU * spp;
		double sPI = stdDevIMU * stdDevPTAM * spi;
		
		int sgn = copysign(1, spi);

		double tmp = (sII-sPP)*(sII-sPP) + 4*sPI*sPI;
		if(tmp <= 0) 
		{
		  tmp = 1e-5;	// numeric issues
		}
		return 0.5*(-(sII-sPP)+sgn*sqrt(tmp)) / (stdDevIMU * stdDevIMU * spi);

	}

	inline ScaleStruct(tf::Vector3 ptamDist, tf::Vector3 imuDist)
	{
		ptam = ptamDist;
		imu = imuDist;
		pp = ptam[0]*ptam[0] + ptam[1]*ptam[1] + ptam[2]*ptam[2];
		ii = imu[0]*imu[0] + imu[1]*imu[1] + imu[2]*imu[2];
		pi = imu[0]*ptam[0] + imu[1]*ptam[1] + imu[2]*ptam[2];

		ptamNorm = sqrt(pp);
		imuNorm = sqrt(ii);

		alphaSingleEstimate = computeEstimator(pp,ii,pi);
	}

	inline bool operator < (const ScaleStruct& comp) const
	{
		return alphaSingleEstimate < comp.alphaSingleEstimate;
	}
};
//

class ScaleEstimator {
public:
     float scale = 1; // has units m^-1
     
     // tuning parameters
     float orb_noise=0.2, nav_noise=0.01; // noise params must be tuned (init vals from tum)
     float dot_prod_tol = 0.05;
     float ratio = 0.2;
     int scale_samples = 10;
     
     tf::Vector3 orb_displacement = tf::Vector3 ( 0,0,0 );
     tf::Vector3 nav_data_displacement = tf::Vector3 ( 0,0,0 );

     tf::Vector3 init_displacement = tf::Vector3 ( 0,0,0 );

     bool fixed_scale = false;

     geometry_msgs::PoseWithCovarianceStamped filt_pose;

     tf::TransformListener tf_listener;

     queue<geometry_msgs::PoseWithCovarianceStamped> orb_data_queue;
     vector<ScaleStruct> scale_vector;
     
     void reset() {
          // reset dispacements
          nav_data_displacement = tf::Vector3 ( 0.0, 0.0, 0.0 );
          orb_displacement = tf::Vector3 ( 0.0, 0.0, 0.0 );

          // clear queue
          queue<geometry_msgs::PoseWithCovarianceStamped> empty;
          swap ( orb_data_queue, empty );
     }
     
     void hard_reset()
     {
       reset();
       
       // also reset scale etc.
       scale = 1;
       fixed_scale = false;
       scale_vector.clear();
       geometry_msgs::PoseWithCovarianceStamped empty_pose;
       filt_pose = empty_pose;
     }   

     void estimate_scale() {
	  // taken from tum   
	  // find median.
	  sort(scale_vector.begin(), scale_vector.end());
	  
	  double median; 
	  if (scale_vector.size() < 5)
	  {
	    median = 1;
	  }
	  else {
 	    median = scale_vector[(scale_vector.size()+1)/2].alphaSingleEstimate;
	  }
	  // find sums and median.
	  // do separately for xy and z and xyz-all and xyz-filtered
	  double sumII = 0;
	  double sumPP = 0;
	  double sumPI = 0;
	  double totSumII = 0;
	  double totSumPP = 0;
	  double totSumPI = 0;
	  
	  double sumIIxy = 0;
	  double sumPPxy = 0;
	  double sumPIxy = 0;
	  double sumIIz = 0;
	  double sumPPz = 0;
	  double sumPIz = 0;

	  int numIn = 0;
	  int numOut = 0;
	  int count = 0;
	  
	  for(unsigned int i=0;i<scale_vector.size();i++)
	  {
		  if(scale_vector.size() < 5 || (scale_vector[i].alphaSingleEstimate > median * ratio && scale_vector[i].alphaSingleEstimate < median / ratio))
		  {
			  sumII += scale_vector[i].ii;
			  sumPP += scale_vector[i].pp;
			  sumPI += scale_vector[i].pi;

			  sumIIxy += scale_vector[i].imu[0]*scale_vector[i].imu[0] + scale_vector[i].imu[1]*scale_vector[i].imu[1];
			  sumPPxy += scale_vector[i].ptam[0]*scale_vector[i].ptam[0] + scale_vector[i].ptam[1]*scale_vector[i].ptam[1];
			  sumPIxy += scale_vector[i].ptam[0]*scale_vector[i].imu[0] + scale_vector[i].ptam[1]*scale_vector[i].imu[1];
		  
			  sumIIz += scale_vector[i].imu[2]*scale_vector[i].imu[2];
			  sumPPz += scale_vector[i].ptam[2]*scale_vector[i].ptam[2];
			  sumPIz += scale_vector[i].ptam[2]*scale_vector[i].imu[2];
			  count++;

		  }
		  else
		  {
			  totSumII += scale_vector[i].ii;
			  totSumPP += scale_vector[i].pp;
			  totSumPI += scale_vector[i].pi;
		  }
	  }

	  fixed_scale = fixed_scale or count > scale_samples;
	  
	  if (not fixed_scale)
	  {
	      scale = ScaleStruct::computeEstimator(sumPP,sumII,sumPI, orb_noise,nav_noise);
	      
	  } else {
	    cout << "scale fixed" << endl;
	  }
	  // taken from tum
     }
     
     void print_all()
     {       
       cout << endl << endl;
       cout << "scale: " << "\t" << scale << endl;
       cout << "scaled pose x:" << "\t" << filt_pose.pose.pose.position.x  <<endl;
     }

     void process_queue() {

          ros::Time t_oldest, t_newest;
          tf::StampedTransform old_odom_to_base_link, new_odom_to_base_link;
          geometry_msgs::PoseWithCovarianceStamped oldest_orb_msg, newest_orb_msg;

          // process orb
          if ( orb_data_queue.size() < 2 ) {
               return;
          } else {
               oldest_orb_msg = orb_data_queue.front();
               orb_data_queue.pop();
               while ( !orb_data_queue.empty() ) {
                    newest_orb_msg = orb_data_queue.front();
                    orb_data_queue.pop();
               }

               t_oldest = oldest_orb_msg.header.stamp;
               t_newest = newest_orb_msg.header.stamp;

               double dx, dy, dz;
               dx = newest_orb_msg.pose.pose.position.x - oldest_orb_msg.pose.pose.position.x;
               dy = newest_orb_msg.pose.pose.position.y - oldest_orb_msg.pose.pose.position.y;
               dz = newest_orb_msg.pose.pose.position.z - oldest_orb_msg.pose.pose.position.z;

               orb_displacement = tf::Vector3 ( dx,dy,dz );

               // process nav data
               try {
                    tf_listener.lookupTransform ( "odom", "/ardrone_base_link", t_oldest, old_odom_to_base_link );
                    tf_listener.lookupTransform ( "odom", "/ardrone_base_link", t_newest, new_odom_to_base_link );

               } catch ( tf2::ExtrapolationException e ) {
                    cout << e.what() << endl;
               }

               nav_data_displacement = new_odom_to_base_link.getOrigin() - old_odom_to_base_link.getOrigin();
		
	       // add new point estimate
               ScaleStruct s = ScaleStruct(orb_displacement, nav_data_displacement);
	       cout << "tol = " << s.pi/s.ptamNorm << endl;
	       if (s.pi > dot_prod_tol * s.ptamNorm) 
	       {
		 cout << "taking scale" << endl;
		 scale_vector.push_back(s);
	       }
          }
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



          void publish_scale ( ros::Publisher &publisher ) {
               publisher.publish ( scale );
          }

          void publish_scaled_pose ( ros::Publisher &publisher ) {
               publisher.publish ( filt_pose );
          }

          void orb_callback ( geometry_msgs::PoseWithCovarianceStamped msg ) {
               orb_data_queue.push ( msg );
          }

          void set_initial_nav_position ( tf::Vector3 pos ) {
               init_displacement = pos;
          }

     };


     int main ( int argc, char **argv ) {
          ros::init ( argc, argv, "scale_estimator" );
          ros::NodeHandle nh;

          // estimates scale
          ScaleEstimator scale_est;
	  
	  // reset command
	  //ros::ServiceServer s = nh.advertiseService("reset_scale_estimator", &ScaleEstimator::hard_reset, &scale_est);

          // subscribe to orb pose and accelerometer data from imu
          ros::Subscriber orb_sub = nh.subscribe ( "/orb/pose_unscaled", 10, &ScaleEstimator::orb_callback, &scale_est );

          // publish scale and filtered orb pose
          ros::Publisher filt_orb_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ( "/orb/pose_scaled",2 );

          int rate = 20;
          int counter = 0;
          ros::Rate loop_rate ( rate );

          scale_est.reset();

          while ( ros::ok() ) {
               // work through all messages received
               ros::spinOnce();
	       
               if ( counter % 10 == 0 ) {
		 // computes displacements for nav_data msgs and orb poses
                    scale_est.process_queue(); 
                    scale_est.estimate_scale();
		    scale_est.reset();
                    counter = 0;
               }
               // set latest orb pose
               scale_est.set_orb_pose();
               scale_est.print_all();

               scale_est.publish_scaled_pose ( filt_orb_pub );

               loop_rate.sleep();
               counter++;
          }

          return 0;
     }

