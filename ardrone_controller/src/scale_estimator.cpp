#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ardrone_autonomy/Navdata.h>

#include <std_msgs/Float32.h>

#include <tf/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath//Vector3.h>
#include <tf/tfMessage.h>

using namespace std;

class ScaleEstimator
{
public:
    float scale = 1; // has units m^-1
    float sxx=0, syy=0, sxy=0, orb_noise=0.2, nav_noise=0.6; // noise params must be tuned (init vals from tum)
    
    bool good_estimate;

    tf::Vector3 orb_displacement = tf::Vector3(0,0,0);
    tf::Vector3 nav_data_displacement = tf::Vector3(0,0,0);
    
    tf::Vector3 tot_orb_displacement = tf::Vector3(0,0,0);
    tf::Vector3 init_displacement = tf::Vector3(0,0,0);
    tf::Vector3 tot_nav_data_displacement = tf::Vector3(0,0,0);;
        
    bool nav_init = false;
    bool orb_init = false;
    
    geometry_msgs::PoseWithCovarianceStamped filt_pose;
    tf::Vector3 filt_position;
    tf::Quaternion orb_orientation;

    tf::TransformListener tf_listener;

    tf::Vector3 curr_orb_position; 
    
    queue<ardrone_autonomy::Navdata> nav_data_queue;
    queue<geometry_msgs::PoseWithCovarianceStamped> orb_data_queue;

    void reset_all() {
      cout << "====================== reset ====================" << endl;
        nav_data_displacement = tf::Vector3 ( 0.0, 0.0, 0.0 );
	orb_displacement = tf::Vector3 (0.0, 0.0, 0.0 );
	
	// clear queue
	queue<geometry_msgs::PoseWithCovarianceStamped> empty1;
	swap(orb_data_queue, empty1);
	
	queue<ardrone_autonomy::Navdata> empty;
        swap(nav_data_queue, empty);
    }
	    
    geometry_msgs::PoseWithCovarianceStamped get_scaled_pose() {
            geometry_msgs::PoseWithCovarianceStamped pose_out;

            pose_out.pose.pose.position.x = curr_orb_position.x() / scale;
            pose_out.pose.pose.position.y = curr_orb_position.y() / scale;
            pose_out.pose.pose.position.z = curr_orb_position.z() / scale;

            pose_out.pose.pose.orientation.x = orb_orientation.getX();
	    pose_out.pose.pose.orientation.y = orb_orientation.getY();
	    pose_out.pose.pose.orientation.z = orb_orientation.getZ();
	    pose_out.pose.pose.orientation.w = orb_orientation.getW();

            pose_out.header.frame_id = "odom";

            pose_out.header.stamp = ros::Time::now();

            return pose_out;
        }

        std_msgs::Float32 get_scale() {
            std_msgs::Float32 s;
            s.data = scale;
            return s;
        }

        void estimate_scale() {	
	    
	    
            // update sxx, sxy, syy
            sxx += nav_noise * nav_noise * orb_displacement.length2();
            syy += orb_noise * orb_noise * nav_data_displacement.length2();
            sxy += nav_noise * orb_noise * orb_displacement.dot ( nav_data_displacement );

            // update scale and filtered position
            scale = ( sxx - syy + copysign ( 1.0, sxy ) * sqrt ( pow ( sxx - syy,2 ) + 4*pow ( sxy,2 ) ) ) / ( 2*nav_noise / orb_noise * sxy );
            if (sxy == 0) scale = 1;
	    
	    cout << "non-linear scale: " << scale << endl;
	}
        
        void estimate_pose() {
	
	  double f1 = (scale * nav_noise * nav_noise)/(scale * scale * nav_noise * nav_noise + orb_noise * orb_noise);
	  double f2 = (orb_noise * orb_noise)/(scale * scale * nav_noise * nav_noise + orb_noise * orb_noise);
	  
	  tot_nav_data_displacement += nav_data_displacement;
	  tot_orb_displacement += orb_displacement;
	  
	  filt_position += f1*tot_orb_displacement + f2*tot_nav_data_displacement;
	  
	  filt_pose.header.frame_id = "odom";
	  filt_pose.header.stamp = ros::Time::now();
	  
	  // orientation
	  filt_pose.pose.pose.orientation.x = orb_orientation.getX();
	  filt_pose.pose.pose.orientation.y = orb_orientation.getY();
	  filt_pose.pose.pose.orientation.z = orb_orientation.getZ();
	  filt_pose.pose.pose.orientation.w = orb_orientation.getW();
	  
	  // position
	  filt_pose.pose.pose.position.x = filt_position.getX();
	  filt_pose.pose.pose.position.y = filt_position.getY();
	  filt_pose.pose.pose.position.z = filt_position.getZ();
	  
	}
	
        bool ready()
	{
	  cout << orb_init << " " << nav_init << endl;
	  return orb_init && nav_init;
	}
  
	
	
  void process_queue()
  {

    ros::Time t_oldest, t_newest;
    tf::StampedTransform old_odom_to_base_link, new_odom_to_base_link;
    geometry_msgs::PoseWithCovarianceStamped oldest_orb_msg, newest_orb_msg;
	  
    cout << "orb msgs: " << orb_data_queue.size()<<endl<<"nav msgs: " <<nav_data_queue.size()<<endl;
     
    // process orb
    if (orb_data_queue.size() < 2)
    {
      orb_init = false;//do smth
      return;
    } else {    
      orb_init = true;
      
      oldest_orb_msg = orb_data_queue.front();
      orb_data_queue.pop();
      while (!orb_data_queue.empty())
      {
	newest_orb_msg = orb_data_queue.front();
	orb_data_queue.pop();
      }
      
      t_oldest = oldest_orb_msg.header.stamp;
      t_newest = newest_orb_msg.header.stamp;
      
      double dx, dy, dz;
      dx = newest_orb_msg.pose.pose.position.x - oldest_orb_msg.pose.pose.position.x; 
      dy = newest_orb_msg.pose.pose.position.y - oldest_orb_msg.pose.pose.position.y; 
      dz = newest_orb_msg.pose.pose.position.z - oldest_orb_msg.pose.pose.position.z; 

           
      orb_displacement = tf::Vector3(dx,dy,dz);    
      orb_orientation = tf::Quaternion(newest_orb_msg.pose.pose.orientation.x,
				       newest_orb_msg.pose.pose.orientation.y,
				       newest_orb_msg.pose.pose.orientation.z,
				       newest_orb_msg.pose.pose.orientation.w);
    }

   // process nav data
   if (nav_data_queue.size() < 2)
    {
      nav_init = false;//do smth
      return;
    } else {
      nav_init = true;

      try {
      tf_listener.lookupTransform ( "odom", "/ardrone_base_link", t_oldest, old_odom_to_base_link );
      tf_listener.lookupTransform ( "odom", "/ardrone_base_link", t_newest, new_odom_to_base_link );

      } catch (tf2::ExtrapolationException e) {
	cout << e.what() << endl;
      }   
      
      nav_data_displacement = new_odom_to_base_link.getOrigin() - old_odom_to_base_link.getOrigin();
      
      if (13 < nav_data_displacement.angle(orb_displacement) * 180 /3.14159)
      {
	good_estimate = true;
      } else {
	estimate_scale();
	good_estimate = false;
      }
      
      cout << "angle: " << endl << nav_data_displacement.angle(orb_displacement) * 180 /3.14159 << endl;
      cout << "scale: " << endl << scale;
      cout << "orb: " << orb_displacement.length() << "\t" << "nav: " << nav_data_displacement.length() << endl;
      
    }
    
  }
  

  void publish_scale(ros::Publisher &publisher)
    {
     publisher.publish ( scale );
    }
    
  void publish_scaled_pose(ros::Publisher &publisher)
    {
      publisher.publish ( filt_pose );
    }
  
  void orb_callback ( geometry_msgs::PoseWithCovarianceStamped msg ) {
      orb_data_queue.push(msg);
    }
    
  void nav_data_callback ( ardrone_autonomy::Navdata msg ) {
	nav_data_queue.push(msg);
    }
    
  void set_initial_nav_position(tf::Vector3 pos)
  {
    init_displacement = pos;
  }
  
};
    
    
    int main ( int argc, char **argv ) {
        ros::init ( argc, argv, "scale_estimator" );
        ros::NodeHandle nh;

        // estimates scale
        ScaleEstimator scale_est;

        // subscribe to orb pose and accelerometer data from imu
        ros::Subscriber imu_sub = nh.subscribe ( "/ardrone/navdata", 50,  &ScaleEstimator::nav_data_callback, &scale_est );
        ros::Subscriber orb_sub = nh.subscribe ( "/orb/pose_unscaled", 10, &ScaleEstimator::orb_callback, &scale_est );

        // publish scale and filtered orb pose
        ros::Publisher filt_orb_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ( "/orb/pose_scaled",2 );
        ros::Publisher orb_scale_pub = nh.advertise<std_msgs::Float32> ( "/orb/scale", 2 );
	
	int rate = 2;
        ros::Rate loop_rate ( rate );
	cout << "Started scale estimation with rate " << rate << endl; 
	
	scale_est.reset_all();
	
        while ( ros::ok() ) {	  
            // work through all messages received
            ros::spinOnce();

	    scale_est.process_queue(); // computes displacements for nav_data msgs and orb poses
	    
	    if (not scale_est.orb_init or not scale_est.nav_init)
	    {
	      loop_rate.sleep();
	      scale_est.reset_all();
	      cout << " orb init: " << scale_est.orb_init << " nav data init: " << scale_est.nav_init << endl;
	      continue;
	    }
	    
	    scale_est.estimate_pose(); // estimates a filtered pose
	
	    scale_est.publish_scale(orb_scale_pub);
	    
	    scale_est.publish_scaled_pose(filt_orb_pub);
	    
	    cout << "***************reseting***************" << endl;
	    scale_est.reset_all();
	    
            // TODO: debug
            std_msgs::Float32 scale = scale_est.get_scale();
	    geometry_msgs::PoseWithCovarianceStamped scaled_orb_pose = scale_est.get_scaled_pose();
	    
            cout << "SCALE: " << scale << endl;
            cout << "SCALED POSE: " 
	         << scaled_orb_pose.pose.pose.position.x << " "
                 << scaled_orb_pose.pose.pose.position.y << " "
                 << scaled_orb_pose.pose.pose.position.z << endl;
	   // debug


            loop_rate.sleep();
        }

        return 0;
    }

