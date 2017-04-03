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

    tf::Vector3 orb_displacement = tf::Vector3(0,0,0);
    tf::Vector3 nav_data_displacement = tf::Vector3(0,0,0);
    
    tf::Vector3 tot_orb_displacement = tf::Vector3(0,0,0);
    tf::Vector3 tot_nav_data_displacement = tf::Vector3(0,0,0);;		
    
    geometry_msgs::PoseWithCovarianceStamped filt_pose;
    tf::Vector3 filt_position;
    tf::Quaternion orb_orientation;

    tf::TransformListener tf_listener;
    vector<double> scale_vector;
		vector<double> orbAverages;
		vector<double> altAverages;
    tf::Vector3 curr_orb_position; 
    
    vector<ardrone_autonomy::Navdata> nav_data_queue;
    vector<geometry_msgs::PoseWithCovarianceStamped> orb_data_queue;
    geometry_msgs::PoseWithCovarianceStamped newest_orb_msg;
  

    void reset_all() {
      cout << "====================== reset ====================" << endl;
      
      nav_data_displacement = tf::Vector3 ( 0.0, 0.0, 0.0 );
      orb_displacement = tf::Vector3 (0.0, 0.0, 0.0 );
	
      nav_data_queue.clear();
      orb_data_queue.clear();
	
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
     
     if (scale_vector.size() > 120) return; //enough scales for accuracy
     
     if(orbAverages.size() != altAverages.size()) return;

     cout << "reestimating scale..." << endl; 
     
     double deltaX = ( (orbAverages[0] + orbAverages[1] + orbAverages[2] ) - (orbAverages[3] + orbAverages[4] + orbAverages[5] ) ) / 3 ;
     double deltaY = ( (altAverages[0] + altAverages[1] + altAverages[2] ) - (altAverages[3] + altAverages[4] + altAverages[5] ) ) / 3 ;
     double naive_scale = deltaX / deltaY;

      if (naive_scale == naive_scale && naive_scale > 0){
      scale_vector.push_back( naive_scale );    //naive scale
      std::sort(scale_vector.begin() ,scale_vector.end() );
     }
          
     if ( scale_vector.size() > 50 ){
      double scale_accumulated = 0.0;
      double samples = 16;
      for (int index = ((scale_vector.size() / 2) - (samples / 2)) ; index < ((scale_vector.size() / 2) + (samples / 2) ) ; index++){
       scale_accumulated += scale_vector[index]; 
      }
       scale = scale_accumulated / samples;
     }
     else{
      scale = scale_vector[scale_vector.size() / 2]; //return simple median for first few readings
     }
     cout << "naive scale: " << scale << endl; 
    }

	
    void estimate_pose() {
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
	
	double get_average_z(vector<ardrone_autonomy::Navdata> &queue){
		double data = 0;
		int counter = 0;
		for(counter ; counter < queue.size() ; counter++ ){
			data += queue[counter].altd;
		}
		return data / counter / 1000;
	}
	
	double get_average_z(vector<geometry_msgs::PoseWithCovarianceStamped> &queue){
		double data = 0;
		int counter = 0;
		for(counter ; counter < queue.size() ; counter++ ){
			data += queue[counter].pose.pose.position.z;
		}
		return data / counter;
	}
	
  void process_queue()
  {
		orbAverages.push_back(get_average_z(orb_data_queue));
		altAverages.push_back(get_average_z(nav_data_queue));
    
    newest_orb_msg = orb_data_queue.back();
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
    }
    
  void nav_data_callback ( ardrone_autonomy::Navdata msg ) {
	nav_data_queue.push_back(msg);
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

        while ( ros::ok() ) {	  

	    if (scale_est.orb_data_queue.size() > 0){
	      
				ros::spinOnce();
				loop_rate.sleep();
				
				ros::spinOnce();
				loop_rate.sleep();
										
	      scale_est.process_queue();
	    
				if (scale_est.orbAverages.size() > 5){
					scale_est.estimate_scale(); 
					scale_est.estimate_pose(); 
					
					scale_est.publish_scale(orb_scale_pub);
					scale_est.publish_scaled_pose(filt_orb_pub);
					
					scale_est.orbAverages.clear();
					scale_est.altAverages.clear();
					
		      scale_est.reset_all();
				}
	    }
	    ros::spinOnce();
			loop_rate.sleep();
	    
            // TODO: debug
      std_msgs::Float32 scale = scale_est.get_scale();
	    geometry_msgs::PoseWithCovarianceStamped scaled_orb_pose = scale_est.get_scaled_pose();

    }

    return 0;
	}

// kate: indent-mode cstyle; indent-width 1; replace-tabs on; ;
