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
  ros::NodeHandle node_handle;
  ros::Subscriber point_cloud_sub = node_handle.subscribe("orb/point_cloud", 1, &RealTimeMotionPlanner::updateEnvironment, &rtmp);
  
  ros::Subscriber pose_sub = node_handle.subscribe<geometry_msgs::Pose>( "quad_pose", 1,  &RealTimeMotionPlanner::update_pose, &rtmp);
  ros::Publisher planner_pub = node_handle.advertise<visualization_msgs::Marker>("reference_trajectory", 2);
  
  ros::Rate loop_rate(8);
  
  
  //TODO publish trajectory, subscribe to environment, publish controls as well
  while (ros::ok()) {
    
    rtmp.replan(rtmp.current_state);//TODO this needs to be a predicted state in the future
    ros::spinOnce();
    planner_pub.publish(rtmp.traj_marker);
    loop_rate.sleep();
  }
  
  
  return 0;
}
