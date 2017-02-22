/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>

#include<sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include <tf/transform_broadcaster.h>


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM), pc(){
        pc.header.frame_id= "world";
    }
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    
    ORB_SLAM2::System* mpSLAM;
    
    sensor_msgs::PointCloud pc;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    
    ImageGrabber igb(&SLAM);
    
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Publisher pub = nodeHandler.advertise<sensor_msgs::PointCloud>("/environment/point_cloud", 2);
    
    ros::Rate loop_rate(30);
    
    while (ros::ok()) {
        
        ros::spinOnce();
        pub.publish(igb.pc);
        loop_rate.sleep();
    }
    
    
    // Stop all threads
    SLAM.Shutdown();
    
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    
    ros::shutdown();
    
    return 0;
}

//callback
void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //     Main slam routine. Extracts new pose
    cv::Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
    // if points can be tracked then broadcast the pose 
    if (not pose.empty()) {
        
        
        tf::Vector3 origin;
        tf::Quaternion tfqt;
        tf::Matrix3x3 tf3d;
        
        origin.setValue(pose.at<float>(0,3), 
                        pose.at<float>(1,3), 
                        pose.at<float>(2,3));
        
        tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), 
                      pose.at<float>(0,2), pose.at<float>(1,0), 
                      pose.at<float>(1,1), pose.at<float>(1,2), 
                      pose.at<float>(2,0), pose.at<float>(2,1), 
                      pose.at<float>(2,2));
        
        tf3d.getRotation(tfqt);
        
        transform.setOrigin(tf3d.transpose() * origin * -1.0);
        transform.setRotation(tfqt);
        
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "level", "odom"));
        
    }
    
    // gets points from most recent frame
    // gets all points
    const std::vector<ORB_SLAM2::MapPoint*> &point_cloud = mpSLAM->mpMap->GetAllMapPoints();
    // TODO: make efficient (use mpSLAM->GetTrackedMapPoints() to get most recent points)
    pc.points.clear();
    for(size_t i=0; i<point_cloud.size();i++)
    {
        if(point_cloud[i]->isBad()/* or spRefMPs.count(vpMPs[i])*/)
            continue;
        cv::Mat pos = point_cloud[i]->GetWorldPos();
        geometry_msgs::Point32 pp;
        pp.x=pos.at<float>(0);
        pp.y=pos.at<float>(1);
        pp.z=pos.at<float>(2);
        
        pc.points.push_back(pp);
    }
    
}


