#include <glc_planner.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>

#include <glc_adapter.h>

// 2) Create an approximation of input space 
class ControlInputs2D: public glc::Inputs
{
public:
    //uniformly spaced points on a circle
    ControlInputs2D(int num_inputs)
    {
        glc::vctr u(num_inputs);
        for(int i=0;i<num_inputs;i++)
        {
            u[0]=sin(2.0*i*M_PI/num_inputs);
            u[1]=cos(2.0*i*M_PI/num_inputs);
            addInputSample(u);
        }
    }
};


//Wrapper around the glc algorithm for real-time functionality
class real_time_motion_planner
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
    glc::traj current_plan;
    glc::vctr current_state;
    
    visualization_msgs::Marker traj_marker;
    geometry_msgs::Point p;
    

    real_time_motion_planner():current_state({0.0,0.0})
    {
        parameters.res=8;
        parameters.control_dim = 2;
        parameters.state_dim = 2;
        parameters.depth_scale = 100;
        parameters.dt_max = 0.2;
        parameters.max_iter = 20000;
        parameters.time_scale = 6;
        parameters.partition_scale = 1.5;
        parameters.x0.push_back(0.0);parameters.x0.push_back(0.0); 
        double goal_radius = 1.0;
        glc::vctr xg({0.0,10.0});

        dynamic_model = new glc::SingleIntegrator(parameters.dt_max);
        controls = new ControlInputs2D(parameters.res);
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
        
        //Send traj markers to rviz
        traj_marker.points.clear();
        for(int i=0;i<current_plan.states.size();i++)
        {
            p.x=current_plan.states[i][0];
            p.y=current_plan.states[i][1];
            p.z=0.0;
            
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



int main(int argc, char **argv) 
{
    real_time_motion_planner rtmp;
    
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>( "quad_pose", 10,  &real_time_motion_planner::update_pose, &rtmp);
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
