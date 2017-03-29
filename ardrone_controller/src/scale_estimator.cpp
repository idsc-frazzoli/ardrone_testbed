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
    float sxx=0, syy=0, sxy=0, orb_noise=2, nav_noise=0.5, xx, xy; // noise params must be tuned (init vals from tum)

    tf::Vector3 orb_displacement = tf::Vector3(0,0,0);
    tf::Vector3 nav_data_displacement = tf::Vector3(0,0,0);
    
    tf::Vector3 tot_orb_displacement = tf::Vector3(0,0,0);
    tf::Vector3 tot_nav_data_displacement = tf::Vector3(0,0,0);;
        
    bool isBadReading = false;
    bool isDominantVy = true;
    
    
    geometry_msgs::PoseWithCovarianceStamped filt_pose;
    tf::Vector3 filt_position;
    tf::Quaternion orb_orientation;

    tf::TransformListener tf_listener;
    vector<double> scale_vector;
    vector<int> messageIndex;
    tf::Vector3 curr_orb_position; 
    
    vector<ardrone_autonomy::Navdata> nav_data_queue;
    vector<geometry_msgs::PoseWithCovarianceStamped> orb_data_queue;
    geometry_msgs::PoseWithCovarianceStamped newest_orb_msg;
  

    void reset_all() {
      cout << "====================== reset ====================" << endl;
        nav_data_displacement = tf::Vector3 ( 0.0, 0.0, 0.0 );
	orb_displacement = tf::Vector3 (0.0, 0.0, 0.0 );
	
	// clear queue
// 	queue<geometry_msgs::PoseWithCovarianceStamped> empty1;
// 	swap(orb_data_queue, empty1);
// 	
// 	queue<ardrone_autonomy::Navdata> empty;
//         swap(nav_data_queue, empty);
	
	nav_data_queue.clear();
	orb_data_queue.clear();
	
	isBadReading = false;
	messageIndex.clear();
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
            sxx += pow(nav_noise,2) * orb_displacement.length2();
            syy += pow(orb_noise,2) * nav_data_displacement.length2();
            sxy += nav_noise * orb_noise * orb_displacement.dot ( nav_data_displacement );

            // update scale and filtered position
	    if (not sxy == 0){
	      scale = ( sxx - syy + copysign ( 1.0, sxy ) * sqrt ( pow ( sxx - syy,2 ) + 4 * pow ( sxy,2 ) ) ) / ( 2*nav_noise / orb_noise * sxy );
	      cout << "non-linear scale: " << scale << endl;
	    }
	    else{
	      cout << "Encountered numerical instability - no scale update!" << endl;
	    }


// 	  orb_displacement.setZ(0);
// 	  nav_data_displacement.setZ(0);
// 	  
// 	  xx += orb_displacement.dot(orb_displacement);
// 	  xy += orb_displacement.dot(nav_data_displacement);
	  
	  
// 	double scale = xx / xy ;
    
    
// 	    if (s > 0)
// 	scale_vector.push_back(s);
// 
// 	std::sort(scale_vector.begin(), scale_vector.end());
// 	
// 	scale = scale_vector[ scale_vector.size() / 2 ];
// 	cout << "scale vector size: " << scale_vector.size() << endl;
//  	cout << "naive scale: " << scale << endl;
//         
	}

	
        void estimate_pose() {
	
// 	  double f1 = (scale * pow(nav_noise,2))/(pow(scale,2) * pow(nav_noise,2) + pow(orb_noise,2));
// 	  double f2 = (pow(orb_noise,2))/(pow(scale,2) * pow(nav_noise,2) + pow(orb_noise,2));
// 	  
// 	  tot_nav_data_displacement += nav_data_displacement;		//drifting!
// 	  tot_orb_displacement += orb_displacement;
// 	  
	  //filt_position = f1*tot_orb_displacement + f2*tot_nav_data_displacement;
	  filt_position.setX( newest_orb_msg.pose.pose.position.x);
	  filt_position.setY( newest_orb_msg.pose.pose.position.y);
	  filt_position.setZ( newest_orb_msg.pose.pose.position.z);
	  
	  filt_pose.header.frame_id = "odom";
	  filt_pose.header.stamp = ros::Time::now();
	  
	  // orientation
	  filt_pose.pose.pose.orientation.x = orb_orientation.getX();
	  filt_pose.pose.pose.orientation.y = orb_orientation.getY();
	  filt_pose.pose.pose.orientation.z = orb_orientation.getZ();
	  filt_pose.pose.pose.orientation.w = orb_orientation.getW();
	  
	  // position
	  filt_pose.pose.pose.position.x = filt_position.getX() / scale;
	  filt_pose.pose.pose.position.y = filt_position.getY() / scale;
	  filt_pose.pose.pose.position.z = filt_position.getZ() / scale;
	  
	  cout << "SCALED POSITION: " << endl 
	<< "X: " << filt_pose.pose.pose.position.x << endl
	<< "Y: " << filt_pose.pose.pose.position.y << endl 		//debug
	<< "Z: " << filt_pose.pose.pose.position.z << endl;
	}
	
	
  void process_queue()
  {
    ros::Time t_oldest, t_newest, t_mid;
    ardrone_autonomy::Navdata oldest_navdata_msg, newest_navdata_msg;
    tf::StampedTransform old_base_link_to_odom, new_base_link_to_odom;
    
    // process orb
      
      geometry_msgs::PoseWithCovarianceStamped oldest_orb_msg, mid_orb_msg;
      
      //orb: simply get the oldest and newest message in the queue
      //in the optimal case there are only two messages but this covers all cases anyway
      
      oldest_orb_msg = orb_data_queue.front();
      mid_orb_msg = orb_data_queue[orb_data_queue.size() / 2];		//median
      newest_orb_msg = orb_data_queue.back();
      
      //nav-data: try to average over an intervall of 4 messages around the orb message
      //to reduce the chance of hitting an outlier
      //for such a small intervall (20 millisec) the velocity can be assumed to be constant 
      
      //ugly but only for testing
//       int orb_messages = 0;
//       for (int index = 0; index < messageIndex.size() ; index++)
//       {
// 	if(messageIndex[index] == 1)
// 	  orb_messages++;
//       }
//       
// 
//       oldest_navdata_msg = nav_data_queue[];
      orb_accel = /
      
      t_oldest = oldest_orb_msg.header.stamp;
      t_mid = mid_orb_msg.header.stamp;
      t_newest = newest_orb_msg.header.stamp;
         
      double x, y, z, xnew, ynew, znew, xold, yold, zold;
      xnew = newest_orb_msg.pose.pose.position.x;
      ynew = newest_orb_msg.pose.pose.position.y; 
      znew = newest_orb_msg.pose.pose.position.z;
      
      x = mid_orb_msg.pose.pose.position.x;
      y = mid_orb_msg.pose.pose.position.y; 
      z = mid_orb_msg.pose.pose.position.z;
      
      xold = oldest_orb_msg.pose.pose.position.x;
      yold = oldest_orb_msg.pose.pose.position.y; 
      zold = oldest_orb_msg.pose.pose.position.z;
      

      orb_displacement = tf::Vector3(dx,dy,dz);    
      orb_orientation = tf::Quaternion(newest_orb_msg.pose.pose.orientation.x,
				       newest_orb_msg.pose.pose.orientation.y,
				       newest_orb_msg.pose.pose.orientation.z,
				       newest_orb_msg.pose.pose.orientation.w);

       //process navda queue

// 	try {
// 
// 	tf_listener.lookupTransform ( "odom", "/ardrone_base_link", t_oldest, old_base_link_to_odom );
// 	tf_listener.lookupTransform ( "odom", "/ardrone_base_link", t_newest, new_base_link_to_odom );
// 	
// 	  
// 	} catch (tf2::ExtrapolationException e) {
// 	  cout << e.what() << endl << "Transform lookup failed: Taking latest transformation" << endl;
// 	  tf_listener.lookupTransform ( "odom", "/ardrone_base_link", t_oldest, old_base_link_to_odom );
// 	  tf_listener.lookupTransform ( "odom", "/ardrone_base_link", ros::Time(0), new_base_link_to_odom );
// 	}  
// 
// 	tf::Vector3 navda_displacement = tf::Vector3(0,0,0);
// 	navda_displacement = new_base_link_to_odom.getOrigin() - old_base_link_to_odom.getOrigin();
// 	
// 	if( ( abs(navda_displacement.dot(orb_displacement)) / orb_displacement.length() ) < 0.05  )
// 	  isBadReading = true;
// 	
// 	nav_data_displacement = navda_displacement;

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
      orb_data_queue.push_back(msg);
      messageIndex.push_back(1);
    }
    
  void nav_data_callback ( ardrone_autonomy::Navdata msg ) {
	nav_data_queue.push_back(msg);
	messageIndex.push_back(0);
	
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
	
	int rate = 60;
        ros::Rate loop_rate ( rate );
	cout << "Started scale estimation with rate " << rate << " Hz" << endl; 
	
	scale_est.reset_all();

        while ( ros::ok() ) {	  
            ros::spinOnce();

	    if (scale_est.orb_data_queue.size() > 2){	//minimum orb poses for estimation is 2
	      loop_rate.sleep();
	      ros::spinOnce();
	      
	      loop_rate.sleep();
	      ros::spinOnce();				//spin twice to make sure enough nav_data messages are added
	      
	      scale_est.process_queue();
	    
	      scale_est.estimate_scale(); 
	      
	      scale_est.estimate_pose(); 
	      
	      scale_est.publish_scale(orb_scale_pub);
	      
	      scale_est.publish_scaled_pose(filt_orb_pub);
	      
	      cout << "Message indices: " << scale_est.messageIndex.size()  << endl;
	      for (int i = 0 ; i < scale_est.messageIndex.size() ;i++ )
		cout << scale_est.messageIndex[i] << endl;
	      
	      scale_est.reset_all();
	      
	      loop_rate.sleep();			//spin to make sure enough nav_data messages are saved before first orb pose
	      ros::spinOnce();
	    }
	    
            // TODO: debug
            std_msgs::Float32 scale = scale_est.get_scale();
	    geometry_msgs::PoseWithCovarianceStamped scaled_orb_pose = scale_est.get_scaled_pose();

            loop_rate.sleep();
        }

        return 0;
    }

