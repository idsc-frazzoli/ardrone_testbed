#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <kdtree.h>
#include <glc_interface.h>
#include <time.h>

class PointCloudEnvironment : public glc::Obstacles
{
    kdtree::Kdtree* kdtree;
    const int dimension = 3;
    const double radius  = 0.2;//20cm radius around quadrotor
    
public:
    //The pointcloud is 3D 
    void updatePointCloud(const sensor_msgs::PointCloud& new_point_cloud)//TODO copy necessary?
    {
        double t = time(NULL);
        std::cout << "Received new point cloud of size " << new_point_cloud.points.size() << " from SLAM alg. " << std::endl;
        //rebuild kdtree for each new point cloud
        if(kdtree!=nullptr)
            delete kdtree;
        kdtree = new kdtree::Kdtree(3);
        //incremental build of kdtree
        for(int i=0;i<new_point_cloud.points.size();i++)
        {
            
            kdtree::point p({new_point_cloud.points[i].x,
                new_point_cloud.points[i].y,
                new_point_cloud.points[i].z});
            
            kdtree::vertexPtr vtx(new kdtree::Vertex(p));
            kdtree->insert(vtx);
        }
        std::cout << "Built Kdtree in " << time(NULL)-t << " seconds" << std::endl;
    }
    bool collisionFree(const glc::vctr& state, const double& t)//TODO state will not have the same dimension as kdtree points
    {
        
        //Don't query an empty tree
        if(kdtree)
        {
            if(not kdtree->isEmpty())
            {
                kdtree::query_results<kdtree::vertexPtr, kdtree::numT> nearest = kdtree->query(state,1);
                if(kdtree::norm2(nearest.BPQ.queue.top().vtx_ptr->coord - state)<radius)
                    return false;
            }
        }
        return true;
    }
    
};

int main(int argc, char **argv)
{
    PointCloudEnvironment obstacles;
    ros::init(argc, argv, "collision_detection");
    ros::start();
    
    ros::NodeHandle node_handle;
    ros::Subscriber point_cloud_sub = node_handle.subscribe("environment/point_cloud", 2, &PointCloudEnvironment::updatePointCloud, &obstacles);
    
    ros::Rate loop_rate(2);
    while(ros::ok())
    {
        
        glc::vctr x({0.0,1.0,2.0});
        std::cout << "Collision Free? " << obstacles.collisionFree(x,0.0) << std::endl;
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
