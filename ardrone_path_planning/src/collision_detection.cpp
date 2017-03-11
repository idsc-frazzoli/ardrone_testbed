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
    void updatePointCloud(const sensor_msgs::PointCloud& new_point_cloud)
    {
        std::cout << "Received new point cloud of size " << new_point_cloud.points.size() << " from SLAM alg. " << std::endl;
        //rebuild kdtree for each new point cloud
        if(kdtree!=nullptr)
            delete kdtree;
        kdtree = new kdtree::Kdtree(3);
        //incremental build of kdtree
        clock_t t = clock();
        
        std::vector<kdtree::vertexPtr> data;
        for(int i=0;i<new_point_cloud.points.size();i++)
        {
            
            kdtree::point p({new_point_cloud.points[i].x,
                new_point_cloud.points[i].y,
                new_point_cloud.points[i].z});
            kdtree::vertexPtr vtx(new kdtree::Vertex(p));
            data.push_back(vtx);
        }
            kdtree->batchBuild(data);
        
        std::cout << " Built kdtree in " << ((float)(clock()-t))/CLOCKS_PER_SEC << " seconds " << std::endl;
    }
    bool collisionFree(const glc::vctr& state, const double& t)//TODO state will not have the same dimension as kdtree points
    {
        
        //Don't query an empty tree
        if(kdtree!=nullptr)
        {
            if(not kdtree->isEmpty())
            {
                kdtree::query_results<kdtree::vertexPtr, kdtree::numT> nearest = kdtree->query(state,1);
                std::cout << "Depth of tree " << kdtree->depth() << std::endl;
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
    
    ros::Rate loop_rate(1.5);
    while(ros::ok())
    {
        
        glc::vctr x({0.0,0.0,0.0});
        clock_t t=clock();
        std::cout << "Collision Free? " << obstacles.collisionFree(x,0.0) << std::endl;
        std::cout << "Checked for collision in " << ((float)(clock()-t))/CLOCKS_PER_SEC << std::endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
