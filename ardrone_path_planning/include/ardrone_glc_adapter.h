//This file provides the interface between the glc planner and the ardrone model

#ifndef ARDRONE_GLC_ADAPTER_H
#define ARDRONE_GLC_ADAPTER_H

#include <kdtree.h>
#include <glc_interface.h>
#include <glc_adapter.h>

//Control input space discretization for the ardrone
class ControlInputs3D: public glc::Inputs
{
  glc::vctr linearSpace(const double& start, const double& end, const int points)
  {
    double step = (end-start)/points;
    glc::vctr lin_space(points);
    lin_space[0]=start;
    for(int i=1;i<points;i++){lin_space[i]=lin_space[i-1]+step;}
    return lin_space;
  }
  
public:
  //uniformly spaced points on a circle
  ControlInputs3D(int num_inputs)
  {
    glc::vctr vert_speed = linearSpace(-1.0,1.0,3);//TODO depends on num_inputs 
    glc::vctr u(3);
    for(int j=0;j<vert_speed.size();j++){
      for(int i=0;i<num_inputs;i++){
        u[0]=sin(2.0*i*M_PI/num_inputs);
        u[1]=cos(2.0*i*M_PI/num_inputs);
        u[2]=vert_speed[j];
        addInputSample(u);
      }
    }
  }
};


//Collision checking module
class PointCloudEnvironment : public glc::Obstacles
{
  kdtree::Kdtree* kdtree;
  const int dimension = 3;
  const double radius  = 0.2;//20cm radius around quadrotor
  //difference of a n-vector and an m-vector returning a vector of dim min(n,m)
  glc::vctr diff_dim_mismatch(const glc::vctr& a, const glc::vctr& b){
    if(a.size()<b.size()){
      glc::vctr a_minus_b(a.size());
      for(int i=0;i<a.size();i++){
        a_minus_b[i]=a[i]-b[i];
      }
      return a_minus_b;
    }
    glc::vctr a_minus_b(b.size());
    for(int i=0;i<b.size();i++){
      a_minus_b[i]=a[i]-b[i];
    }
    return a_minus_b;
  }
  
public:
  //The pointcloud is 3D 
  void updatePointCloud(const sensor_msgs::PointCloud& new_point_cloud)//TODO seg faults if point cloud is empty
  {
    std::cout << "\nNew Point Cloud Size: " << new_point_cloud.points.size() << std::endl;
    //rebuild kdtree for each new point cloud
    if(kdtree!=nullptr)
      delete kdtree;
    kdtree = new kdtree::Kdtree(3);
    
    //Build kdtree
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
    glc::vctr query_point(kdtree->dimension());
    std::copy(std::begin(state),std::begin(state)+kdtree->dimension(),std::begin(query_point));//TODO inefficient
    //Don't query an empty tree
    if(kdtree!=nullptr)
    {
      if(not kdtree->isEmpty())
      {
        
        kdtree::query_results<kdtree::vertexPtr, kdtree::numT> nearest = kdtree->query(query_point,1);
        if(kdtree::norm2( nearest.BPQ.queue.top().vtx_ptr->coord - query_point ) < radius)
          return false;
      }
    }
    return true;
  }
  
  bool collisionFree(const glc::Trajectory& x, int* last=NULL) override
  { 
    for(int i=0;i<x.size();i++) {
      if(not collisionFree(x.getState(i), x.getTime(i)))
      {
        if(last)
          *last = i;
        return false;
      }
    }
    return true;
  }
  
};

class ArDroneModel: public glc::EulerIntegrator
{
public:
  ArDroneModel(const double& _max_time_step): EulerIntegrator(5,_max_time_step) {}
  
  void flow(glc::vctr& dx, const glc::vctr& x, const glc::vctr& u) override {
    
    
    dx[0]=x[3];
    dx[1]=x[4];
    dx[2]=0.25*u[2];
    dx[3]=1.5*u[0];
    dx[4]=1.5*u[1];
  }
  
  double getLipschitzConstant() override {
    return 1.41;
  }
};

//Wrapper around the glc algorithm for real-time functionality
class RealTimeMotionPlanner
{
  glc::Parameters parameters;
  glc::DynamicalSystem* dynamic_model;
  glc::CostFunction* performance_objective;
  glc::Obstacles* obstacles;
  glc::Heuristic* heuristic;
  glc::SphericalGoal* goal;
  glc::Inputs* controls;
  
public:
  glc::PlannerOutput out;
  glc::Trajectory current_plan;
  glc::vctr current_state;
  
  visualization_msgs::Marker traj_marker;
  geometry_msgs::Point p;
  
  
  RealTimeMotionPlanner():current_state({0.0, 0.0, 2.0, 0.0, 0.0})
  {
    parameters.res=8;
    parameters.control_dim = 3;
    parameters.state_dim = 5;
    parameters.depth_scale = 100;
    parameters.dt_max = 0.2;
    parameters.max_iter = 5000;
    parameters.time_scale = 10;
    parameters.partition_scale = 6.0;
    parameters.x0 = glc::vctr({0.0,0.0,2.0,0.0,0.0});
    double goal_radius = 2.0;
    glc::vctr xg({10.0,0.0,2.0,0.0,0.0});
    
    dynamic_model = new ArDroneModel(parameters.dt_max);
    controls = new ControlInputs3D(parameters.res);
    performance_objective = new glc::MinTimeCost();
    glc::SphericalGoal* sphericalGoal = new glc::SphericalGoal(parameters.state_dim,goal_radius);
    sphericalGoal->setGoal(xg);
    goal = sphericalGoal;
    
    obstacles = new glc::NoObstacles();
    heuristic = new glc::ZeroHeuristic();
    
    //Visualization stuff
    traj_marker.header.frame_id = "world";
    traj_marker.ns = "trajectory_visualisation";
    traj_marker.id = 2;
    traj_marker.type = visualization_msgs::Marker::CUBE_LIST;
    traj_marker.scale.x = 0.1;
    traj_marker.scale.y = 0.1;
    traj_marker.scale.z = 0.1;
    traj_marker.color.a = 1.0;
    traj_marker.color.r = 1.0;
    traj_marker.color.g = 1.0;
    traj_marker.color.b = 1.0;
  }
  
  //Pose msg callback
  void update_pose(geometry_msgs::Pose pose_msg)
  {
    current_state[0]=pose_msg.position.x;
    current_state[1]=pose_msg.position.y;
    return;
  }
  
  void replan(glc::vctr from_here,glc::vctr to_here)
  {
    parameters.x0=from_here;
    goal->setGoal(to_here);
    glc::trajectory_planner motion_planner(obstacles, 
                                           goal, 
                                           dynamic_model, 
                                           heuristic,
                                           performance_objective,
                                           parameters,
                                           controls->points);
    motion_planner.plan(out);
    glc::print_traj(current_plan);
    std::cout << "GLC running time: " << out.time << std::endl;
    current_plan = motion_planner.recover_traj( motion_planner.path_to_root(true) );
    
    //Send Trajectory markers to rviz
    traj_marker.points.clear();
    glc::vctr x;
    for(int i=0;i<current_plan.size();i++)
    {
      x=current_plan.getState(i);
      p.x=x[0];
      p.y=x[1];
      p.z=x[2];
      
      traj_marker.points.push_back(p);
    }
    return;
  }
  
  void replan(glc::vctr _from_here)
  {
    replan(_from_here, goal->getGoal());
    return;
  }
};




#endif
