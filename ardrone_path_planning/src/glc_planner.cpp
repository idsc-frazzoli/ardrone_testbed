#include <glc_planner.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <ardrone_glc_adapter.h>

int main(int argc, char **argv) 
{
  RealTimeMotionPlanner rtmp;
  
  ros::init(argc, argv, "motion_planner");
  ros::NodeHandle nh;
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>( "quad_pose", 10,  &RealTimeMotionPlanner::update_pose, &rtmp);
  ros::Publisher planner_pub = nh.advertise<visualization_msgs::Marker>("reference_trajectory", 2);
  
  ros::Rate loop_rate(2);
  
  
  //TODO publish trajectory, subscribe to environment, publish controls as well
  while (ros::ok()) {
    
    rtmp.replan(rtmp.current_state);//TODO this needs to be a predicted state in the future
    ros::spinOnce();
    planner_pub.publish(rtmp.traj_marker);
    loop_rate.sleep();
  }
  
  
  return 0;
}
