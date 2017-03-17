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
    float sxx=0, syy=0, sxy=0, orb_noise=0.2, nav_noise=0.1; // noise params must be tuned (init vals from tum)

    tf::Vector3 orb_displacement = tf::Vector3(0,0,0);
    tf::Vector3 nav_data_displacement = tf::Vector3(0,0,0);
    
    tf::Vector3 tot_orb_displacement = tf::Vector3(0,0,0);
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
    

//     {
//       
//         ros::Duration dt = ros::Time::now() - msg.header.stamp;
// 	cout << "navdata callback with msg at time " << msg.header.stamp << " now " << ros::Time::now() <<  " delay is " << dt.toSec() << endl;
// 
// 	tf::StampedTransform base_link_to_odom;
//         tf_listener.lookupTransform ( "/ardrone_base_link", "odom", ros::Time(0), base_link_to_odom );
// 
//         float dx = msg.vx * dt.toSec() / 1000;
//         float dy = msg.vy * dt.toSec() / 1000;
//         float dz = ( msg.altd - curr_altd ) / 1000 ;
// 
//         curr_altd = msg.altd;
// 
//         tf::Vector3 displacement_odom = base_link_to_odom ( tf::Vector3 ( dx,dy,dz ) );
// 	
// 	nav_data_displacement += displacement_odom;
//     cout << "current displacement is " << nav_data_displacement.getX() << " " <<nav_data_displacement.getY() << " "<<nav_data_displacement.getZ() << endl;
//       
// 
//     }
//     

    /*
      ros::Duration dt = ros::Time::now()-msg.header.stamp;
      cout << "orb callback with msg from " << msg.header.stamp << " delay is " << dt.toSec() << endl; 
	tf::Vector3 orb_position = tf::Vector3(msg.pose.pose.position.x, 
						msg.pose.pose.position.y, 
						msg.pose.pose.position.z);
	
	orb_displacement += orb_position - curr_orb_position;
	curr_orb_position = orb_position;
	orb_orientation = tf::Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
					 msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
      cout << "current displacement is " << orb_displacement.getX() << " " <<orb_displacement.getY() << " "<<orb_displacement.getZ() << endl;
      
    }*/

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
	    // DEBUG naive scales in xyz directions
	    double s_x = orb_displacement.getX()/nav_data_displacement.getX();
	    double s_y = orb_displacement.getY()/nav_data_displacement.getY();
	    double s_z = orb_displacement.getZ()/nav_data_displacement.getZ();
	  
            // update sxx, sxy, syy
            sxx += nav_noise * nav_noise * orb_displacement.length2();
            syy += orb_noise * orb_noise * nav_data_displacement.length2();
            sxy += nav_noise * orb_noise * orb_displacement.dot ( nav_data_displacement );

            // update scale and filtered position
            scale = ( sxx - syy + copysign ( 1.0, sxy ) * sqrt ( pow ( sxx - syy,2 ) + 4*pow ( sxy,2 ) ) ) / ( 2*nav_noise / orb_noise * sxy );
            if (sxy == 0) scale = 1;
	    
	    cout << "non-linear scale: " << scale << endl;
	    cout << "naive scale x: " << s_x << " y: " << s_y << " z: " << s_z << endl; 
        }
        
        void estimate_pose() {
	
	  double f1 = (scale * nav_noise * nav_noise)/(scale * scale * nav_noise * nav_noise + orb_noise * orb_noise);
	  double f2 = (orb_noise * orb_noise)/(scale * scale * nav_noise * nav_noise + orb_noise * orb_noise);
	  
	  tot_nav_data_displacement += nav_data_displacement;
	  tot_orb_displacement += orb_displacement;
	  
	  filt_position = f1*tot_orb_displacement + f2*tot_nav_data_displacement;
	  
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
    ardrone_autonomy::Navdata curr_msg, old_msg;
    tf::StampedTransform old_base_link_to_odom, new_base_link_to_odom;
    
    cout << "orb msgs: " << orb_data_queue.size()<<endl<<"nav msgs: " <<nav_data_queue.size()<<endl;
     
    // process orb
    if (orb_data_queue.size() < 2)
    {
      orb_init = false;//do smth
      return;
    } else {
      cout << "processing orb msgs: " << endl; 
    
      orb_init = true;
      
      geometry_msgs::PoseWithCovarianceStamped oldest_orb_msg, newest_orb_msg;
      oldest_orb_msg = orb_data_queue.front();
      orb_data_queue.pop();
      while (!orb_data_queue.empty())
      {
	newest_orb_msg = orb_data_queue.front();
// 	cout << "next newest orb pose at " << t_newest.toSec() << " x:" <<  newest_orb_msg.pose.pose.position.x <<
//        " y:" <<  newest_orb_msg.pose.pose.position.y << " z:" <<   newest_orb_msg.pose.pose.position.z << endl;
   
	orb_data_queue.pop();
      }
      
      t_oldest = oldest_orb_msg.header.stamp;
      t_newest = newest_orb_msg.header.stamp;
      
      cout << "t_oldest: " << fmod(t_oldest.toSec() , 1000) << endl << "t_newest: " << fmod(t_newest.toSec() , 1000) << 
      endl << " duration: " << t_newest.toSec() -t_oldest.toSec() <<endl;
      
      double dx, dy, dz;
      dx = newest_orb_msg.pose.pose.position.x - oldest_orb_msg.pose.pose.position.x; 
      dy = newest_orb_msg.pose.pose.position.y - oldest_orb_msg.pose.pose.position.y; 
      dz = newest_orb_msg.pose.pose.position.z - oldest_orb_msg.pose.pose.position.z; 
      
//       cout << "oldest orb pose at " << t_oldest.toSec() << " x:" <<  oldest_orb_msg.pose.pose.position.x <<
//        " y:" <<  oldest_orb_msg.pose.pose.position.y << " z:" <<   oldest_orb_msg.pose.pose.position.z << endl;
//       cout << "newest orb pose at " << t_newest.toSec() << " x:" <<  newest_orb_msg.pose.pose.position.x <<
//        " y:" <<  newest_orb_msg.pose.pose.position.y << " z:" <<   newest_orb_msg.pose.pose.position.z << endl;
       
      cout << "displacement orb: " <<endl<< "x: " << dx << endl << "y: " << dy << endl <<"z: " << dz << endl;
      
      orb_displacement = tf::Vector3(dx,dy,dz);    
      orb_orientation = tf::Quaternion(newest_orb_msg.pose.pose.orientation.x,
				       newest_orb_msg.pose.pose.orientation.y,
				       newest_orb_msg.pose.pose.orientation.z,
				       newest_orb_msg.pose.pose.orientation.w);
    }
       tf::Vector3 ground_truth = tf::Vector3(0,0,0);
    // process nav data
   if (nav_data_queue.size() < 2)
    {
      nav_init = false;//do smth
      return;
    } else {
      nav_init = true;
      
      
      
      old_msg = nav_data_queue.front();
      nav_data_queue.pop();
      int counter = 0;
      while (!nav_data_queue.empty())
      {
	curr_msg = nav_data_queue.front();
	nav_data_queue.pop();
	
	if (curr_msg.header.stamp.toSec() < t_oldest.toSec() /*&& curr_msg.header.stamp.toSec() > t_newest.toSec()*/)
	{
	  old_msg = curr_msg;
	  counter++;
// 	  cout << "throwing out msg " << counter << endl;
	  
	  continue;
	} else {
	  //cout << "keeping msg with stamp: " << fmod(curr_msg.header.stamp.toSec() , 1000) << endl;
	  
	  try {
	  tf_listener.lookupTransform ( "/ardrone_base_link", "odom", old_msg.header.stamp, old_base_link_to_odom );
	  tf_listener.lookupTransform ( "/ardrone_base_link", "odom", curr_msg.header.stamp, new_base_link_to_odom );
	  
	    
	  } catch (tf2::ExtrapolationException e) {
	   tf_listener.lookupTransform ( "/ardrone_base_link", "odom", ros::Time(0), old_base_link_to_odom );
	   tf_listener.lookupTransform ( "/ardrone_base_link", "odom", ros::Time(0), new_base_link_to_odom );
	  } 
	  
	  ground_truth += new_base_link_to_odom.getOrigin() - old_base_link_to_odom.getOrigin(); 
	  
	  old_base_link_to_odom.setOrigin(tf::Vector3(0,0,0));
	  new_base_link_to_odom.setOrigin(tf::Vector3(0,0,0));
	  
	  
	  // figure out times to interpolate between
	  ros::Duration max_duration = old_msg.header.stamp - curr_msg.header.stamp; // duration between current and old msg
	  ros::Time t_old = old_msg.header.stamp; // time stamp of old message
	  ros::Time t_curr = curr_msg.header.stamp; // time stamp of curr message
	  double interpol;
	  ros::Duration dt = max_duration;
	  
	  if (t_curr.toSec() > t_newest.toSec())
	  {
	    interpol = (t_newest.toSec() - t_old.toSec()) / (t_curr.toSec() - t_old.toSec()); 
	    dt = t_newest - t_old; 
	    
	  } else if (t_old.toSec() < t_oldest.toSec())
	  {
	    interpol = (t_oldest.toSec() - t_old.toSec()) / (t_curr.toSec() - t_old.toSec()); 
	    dt = t_curr - t_oldest; 
	  } else {
	    
	    interpol = 0.5;
	  }	    
	  
	  double v_x_avg = old_msg.vx + (curr_msg.vx - old_msg.vx) * interpol;
	  double v_y_avg = old_msg.vy + (curr_msg.vy - old_msg.vy) * interpol;
	    
	  cout << "average v_x: " << v_x_avg << " v_y: " << v_y_avg << endl;
	  
	  tf::Matrix3x3 R = old_base_link_to_odom.getBasis().inverse();
	  
	  tf::Vector3 old_altd = old_base_link_to_odom(tf::Vector3(0,0,old_msg.altd));
	  tf::Vector3 new_altd = new_base_link_to_odom(tf::Vector3(0,0,curr_msg.altd));
	  
	  double dz = new_altd[2] - new_altd[2];  

	  dz *= dt.toSec()/max_duration.toSec();
	  
	  double v_z_avg;
	  if (dt.toSec() < 1e-6 or R[2][2] < 1e-6) {
	    v_z_avg = 0;
	  } else {
	    v_z_avg = (dz/dt.toSec() - R[2][0] * v_x_avg - R[2][1] * v_y_avg ) / R[2][2]; 
	  }
	  
	  nav_data_displacement += R * tf::Vector3(dt.toSec()*v_x_avg, dt.toSec()*v_y_avg, dt.toSec()*v_z_avg);
	  
	  old_msg = curr_msg;

	  if (t_curr.toSec() > t_newest.toSec()) break; 
        }
      } 
          
    
    }
    cout << "displacement nav_data: " <<endl<< "x: " << nav_data_displacement.getX() << endl << 
    "y: " << nav_data_displacement.getY() << endl <<"z: " << nav_data_displacement.getZ() << endl;
    cout << "displacement orb " <<endl << "x: " << orb_displacement.getX() << endl << 
    "y: " << orb_displacement.getY() << endl <<"z: " << orb_displacement.getZ() << endl;
    
    cout << "Angle between orb and nav: " << orb_displacement.angle(nav_data_displacement) << endl;
    cout << "Norm ratio: " << orb_displacement.length() / nav_data_displacement.length() << endl;
    cout << "ground truth: " << ground_truth.getX() << " " << ground_truth.getY() << " " << ground_truth.getZ() << endl;
    
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
	
	int rate = 5;
        ros::Rate loop_rate ( rate );
	cout << "Started scale estimation with rate " << rate << endl; 
	
	scale_est.reset_all();
	
        while ( ros::ok() ) {	  
            // work through all messages received
	    cout << "***************Storing messages***************" << endl;
            ros::spinOnce();
	    cout << "***************Finished***************" << endl;
	    
	    cout << "***************processing messages***************" << endl;
	    scale_est.process_queue(); // computes displacements for nav_data msgs and orb poses
	    cout << "***************finished***************" << endl;
	    
	    if (not scale_est.orb_init or not scale_est.nav_init)
	    {
	      loop_rate.sleep();
	      scale_est.reset_all();
	      cout << " orb init: " << scale_est.orb_init << " nav data init: " << scale_est.nav_init << endl;
	      continue;
	    }
	    
	    
	    cout << "***************estimating scale***************" << endl;
	    scale_est.estimate_scale(); // estimates scale based on new displacements
	    cout << "***************finished***************" << endl;
	    
	    cout << "***************estimating pose***************" << endl;
	    scale_est.estimate_pose(); // estimates a filtered pose
	    cout << "***************finished***************" << endl;
	    
	    cout << "***************publishing scale messages***************" << endl;
	    scale_est.publish_scale(orb_scale_pub);
	    cout << "***************finished***************" << endl;
	    
	    cout << "***************publishing orb pose***************" << endl;
	    scale_est.publish_scaled_pose(filt_orb_pub);
	    cout << "***************finished***************" << endl;
	    
	    cout << "***************reseting***************" << endl;
	    scale_est.reset_all();
	    cout << "***************finished***************" << endl;
	    
            // TODO: debug
            std_msgs::Float32 scale = scale_est.get_scale();
	    geometry_msgs::PoseWithCovarianceStamped scaled_orb_pose = scale_est.get_scaled_pose();
	    
            cout << "SCALE: " << scale << endl;
            cout << "SCALED POSE: " << scaled_orb_pose.pose.pose.position.x << " "
                 << scaled_orb_pose.pose.pose.position.y << " "
                 << scaled_orb_pose.pose.pose.position.z << endl;
	   // debug


            loop_rate.sleep();
        }

        return 0;
    }

