#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ardrone_autonomy/Navdata.h>

#include <std_msgs/Float32.h>

#include <tf/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath//Vector3.h>

using namespace std;

class ScaleEstimator
{
public:
    float scale = 1; // has units m^-1
    float sxx=0, syy=0, sxy=0, orb_noise=0.2, nav_noise=0.1; // noise params must be tuned (init vals from tum)

    tf::Vector3 orb_displacement, nav_data_displacement;

    tf::Quaternion orb_orientation;

    bool nav_init, orb_init;
    
    tf::TransformListener tf_listener;

    int32_t curr_altd;
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

        void update_scale() {
            // update sxx, sxy, syy
            sxx += nav_noise * nav_noise * float ( orb_displacement.length2() );
            syy += orb_noise * orb_noise * float ( nav_data_displacement.length2() );
            sxy += nav_noise * orb_noise * float ( orb_displacement.dot ( nav_data_displacement ) );

            cout << "scale params updated: " << sxx << " " << syy << " " << sxy << endl;

            // update scale and filtered position
            scale = ( sxx - syy + copysign ( 1.0, sxy ) * sqrt ( pow ( sxx - syy,2 ) + 4*pow ( sxy,2 ) ) ) / ( 2*nav_noise / orb_noise * sxy );
            cout << "scale: " << scale << endl;
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
     
    // process orb
    if (orb_data_queue.empty())
    {
      orb_init = false;//do smth
      return;
    } else {
    
      orb_init = true;
      
      geometry_msgs::PoseWithCovarianceStamped * newest_orb_msg = orb_data_queue.back();
      geometry_msgs::PoseWithCovarianceStamped * oldest_orb_msg = orb_data_queue.front();
      
      t_oldest = oldest_orb_msg->header.stamp;
      t_newest = newest_orb_msg->header.stamp;
      
      double dx, dy, dz;
      dx = newest_orb_msg->pose.pose.position.x - oldest_orb_msg->pose.pose.position.x; 
      dy = newest_orb_msg->pose.pose.position.y - oldest_orb_msg->pose.pose.position.y; 
      dz = newest_orb_msg->pose.pose.position.z - oldest_orb_msg->pose.pose.position.z; 
      
      orb_displacement = tf::Vector3(dx,dy,dz);    
    }
    
    
    // process nav data
   if (nav_data_queue.empty())
    {
      nav_init = false;//do smth
      return;
    } else {
    
      nav_init = true;
      
      
      while (!nav_data_queue.empty())
      {
	ardrone_autonomy::Navdata curr_msg = nav_data_queue.pop();
	
	if (curr_msg.header.stamp.toSec() < old_msg.header.stamp.toSec())
	{
	  old_msg = curr_msg;
	  continue;
	} else {
	
	  ros::Duration dt = curr_msg.header.stamp.toSec() - old_msg.header.stamp.toSec();
	  
	  try {
	  tf_listener.lookupTransform ( "/ardrone_base_link", "odom", old_msg.header.stamp, old_base_link_to_odom );
	  tf_listener.lookupTransform ( "/ardrone_base_link", "odom", curr_msg.header.stamp, new_base_link_to_odom );
	  
	    
	  } catch (tf::LookupException e) {
	   tf_listener.lookupTransform ( "/ardrone_base_link", "odom", ros::Time(0), old_base_link_to_odom );
	   tf_listener.lookupTransform ( "/ardrone_base_link", "odom", ros::Time(0), new_base_link_to_odom );
	   
	   cout << e.what() << endl;
	  }
	  
	  tf::Matrix3x3 R = old_base_link_to_odom.getBasis();
	  
	  tf::Vector3 old_altd = old_base_link_to_odom(tf::Vector3(0,0,old_msg.altd));
	  tf::Vector3 new_altd = new_base_link_to_odom(tf::Vector3(0,0,curr_msg.altd));
	  
	  double dz = new_altd[2] - new_altd[2];  
	  
	  if (curr_msg.header.stamp.toSec() > t_newest.toSec())
	  {
	    dz = dz * (t_newest.toSec() - old_msg.header.stamp.toSec()) / dt.toSec();
	    double v_z = (dz/dt.toSec() - R[2][0] * old_msg.vx - R[2][1] * old_msg.vy ) / R[2][2]; 
	 
	    nav_data_displacement += dt.toSec() * R * tf::Vector3(old_msg.vx,old_msg.vy,v_z);
	    break;
	  } else {
	  
	  double v_z = (dz/dt.toSec() - R[2][0] * old_msg.vx - R[2][1] * old_msg.vy ) / R[2][2]; 
	 
	  nav_data_displacement += dt.toSec() * R * tf::Vector3(old_msg.vx,old_msg.vy,v_z);
	  
	  
	  old_msg = curr_msg;

      }
       
      
      
      geometry_msgs::PoseWithCovarianceStamped * newest_orb_msg = orb_data_queue.back();
      geometry_msgs::PoseWithCovarianceStamped * oldest_orb_msg = orb_data_queue.front();
      
      ros::Time t_old = oldest_orb_msg->header.stamp;
      ros::Time t_new = newest_orb_msg->header.stamp;
      
      double dx, dy, dz;
      dx = newest_orb_msg->pose.pose.position.x - oldest_orb_msg->pose.pose.position.x; 
      dy = newest_orb_msg->pose.pose.position.y - oldest_orb_msg->pose.pose.position.y; 
      dz = newest_orb_msg->pose.pose.position.z - oldest_orb_msg->pose.pose.position.z; 
      
      orb_displacement = tf::Vector3(dx,dy,dz);    
    }
    
    
    
    
    
    
  }

  void publish_scale(ros::Publisher &publisher)
    {
      if (orb_init && nav_init)  publisher.publish ( scale );
    }
    
  void publish_scaled_pose(ros::Publisher &publisher)
    {
      if (orb_init && nav_init) publisher.publish ( pose );
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
        ros::Subscriber imu_sub = nh.subscribe ( "/ardrone/navdata", 12,  &ScaleEstimator::nav_data_callback, &scale_est );
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
	    cout << "***************Finished storing messages***************" << endl;
	    
	    scale_est.process_queue();
	    
	    scale_est.publish_scale(orb_scale_pub);
	    scale_est.publish_scaled_pose(filt_orb_pub);
	    
	    
	    scale_est.reset_all();
	    
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
