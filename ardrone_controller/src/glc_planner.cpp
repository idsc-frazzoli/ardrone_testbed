#include <glc_planner.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>


// 1) Create a model of the system's mobility (i.e. x'(t)=f(x(t),u(t) )
class single_integrator: public glcm::dynamical_system
{
public:
    //The function f in x'(t)=f(x(t),u(t)) 
    void flow(glcm::vctr& dx, const glcm::vctr& x, const glcm::vctr& u)
    {
        //Single Integrator Model
        for(int i=0;i<n;i++)
        {
            // f(x(t),u(t))=u(t)
            dx.at(i)=u.at(i);
        }
        return;
    }
    
    single_integrator(const int& state_dim)
    {
        sim_counter=0;
        Lip_flow=0.0;
        dt_max=1.0;
        n = state_dim;
        m = state_dim;
        xdot.resize(n);
        k1.resize(n);k2.resize(n);k3.resize(n);k4.resize(n);
        temp.resize(n);
    }
};

// 2) Create an approximation of input space 
class control_inputs: public glcm::inputs
{
public:
    std::deque<glcm::vctr> points;
    
    //uniformly spaced points on a circle
    control_inputs(int num_inputs,int dimension): inputs(dimension)
    {
        if(not (dimension == 2))
        {
            std::cout << "ERROR: Input diminsion is not 2." << std::endl;
        }
        points.clear();
        for(int i=0;i<num_inputs;i++)
        {
            u[0]=sin(2.0*i*M_PI/num_inputs);
            u[1]=cos(2.0*i*M_PI/num_inputs);
           points.push_back(u);
        }
        return;
    }
};

// 3) Create a performance objective to be minimized (i.e. min integral of g(x(t),u(t)) )
class min_time_objective: public glcm::costFunction
{
public:
    min_time_objective()
    {
        n=999;
        m=999;
        Lip_cost=0.0;
    }
    min_time_objective(const int& _n, const int& _m)
    {
        n=_n;
        m=_m;
        Lip_cost=0.0;
    }
    //This is the function g(x(t),u(t)) in the performance objective
    double cost(const glcm::traj& p, const glcm::vctr& u)
    {
        return p.time.back()-p.time.front();
    }
};

// 4) Create an admissible heuristic for the problem
class zero_heuristic: public glcm::heuristic//TODO redundant
{
public:
    
    zero_heuristic()
    {}
    //Define a ball centered at x_g of radius _rad containing the goal
    zero_heuristic(const glcm::vctr& _x_g,const double _rad)
    {
        radius=_rad;
        center=_x_g;
    }
    
    //The estimated cost to reach the goal is zero
    double cost_to_go(const glcm::vctr& x0)
    {
        return 0;
    }
    
    void setGoal(glcm::vctr& _x_g)
    {
        center=_x_g;
    }
};

// 5) Create a goal region
class simple_goal: public glcm::goalRegion //Origin centered ball
{
    double goal_radius, goal_radius_sqr;
    glcm::vctr error;
public:
    
    simple_goal(const int& _state_dim, const double& _goal_radius)
    {
        state_dim=_state_dim;
        goal_radius=_goal_radius;
        goal_radius_sqr=glcm::sqr(goal_radius);
        x_g.clear();
        error.resize(state_dim);
        std::vector<double> zeros(state_dim, 0.0);
        x_g = zeros;
    }
    
    bool inGoal(const glcm::vctr& state, const double& t){
        error=glcm::diff(x_g,state); // TODO subtracting two vectors with different sizes
        return glcm::dot(error,error)<goal_radius_sqr;
    }
    
    void setGoal(const glcm::vctr& _x_g)
    {
        x_g=_x_g;
    }
    
    std::vector<double> getGoal()
    {
        return x_g;
    }
    
    void setRadius(double r)
    {
        goal_radius = r;
        goal_radius_sqr = r*r;
    }
    
    double getRadius()
    {
        return goal_radius;
    }
    
};

// 6) Create free space
class no_obstacles: public glcm::obstacles 
{
public:
    
    no_obstacles()
    {
    }
    //Returns true always
    bool collisionFree(const glcm::traj& p)
    {
        counter++;
        for(int i=0;i<p.states.size();i++)
        {
            //Check collision here
            if(false)
            {
                return false;
            }
        }
        return true;
    }
};


//Wrapper around the glc algorithm for real-time functionality
class real_time_motion_planner
{
    glcm::parameters alg_params;
    single_integrator* dynamic_model;
    min_time_objective* performance_objective;
    no_obstacles* obstacles;
    zero_heuristic* heuristic;
    simple_goal* goal;
    control_inputs* controls;
    
public:
    glcm::plannerOutput out;
    glcm::traj current_plan;
    glcm::vctr current_state;

    real_time_motion_planner():current_state({0.0,0.0})
    {
        alg_params.res=8;
        alg_params.control_dim = 2;
        alg_params.state_dim = 2;
        alg_params.depth_scale = 100;
        alg_params.dt_max = 0.2;
        alg_params.max_iter = 20000;
        alg_params.time_scale = 6;
        alg_params.partition_scale = 1.5;
        alg_params.x0.push_back(0.0);alg_params.x0.push_back(0.0); 
        double goal_radius = 1.0;
        glcm::vctr xg({0.0,10.0});

        dynamic_model = new single_integrator(alg_params.state_dim);
        controls = new control_inputs(alg_params.res,alg_params.control_dim);
        performance_objective = new min_time_objective;
        goal = new simple_goal(alg_params.state_dim,goal_radius);
        goal->setGoal(xg);
        obstacles = new no_obstacles;
        heuristic = new zero_heuristic(goal->getGoal(),goal->getRadius());
    }
    
    void update_pose(geometry_msgs::Pose pose_msg)
    {
        current_state[0]=pose_msg.position.x;
        current_state[1]=pose_msg.position.y;
        return;
    }
    
    void replan(const glcm::vctr& from_here, const glcm::vctr& to_here)
    {
        alg_params.x0=from_here;
        goal->setGoal(to_here);
        glcm::trajectory_planner motion_planner(obstacles, 
                                                goal, 
                                                dynamic_model, 
                                                heuristic,
                                                performance_objective,
                                                alg_params,
                                                controls->points);

        motion_planner.plan(out);
        current_plan = motion_planner.recover_traj( motion_planner.path_to_root(true) );
        glcm::print_traj(current_plan);
        std::cout << "GLC running time: " << out.time << std::endl;
        return;
    }
    
    void replan(const glcm::vctr& _from_here)
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
    ros::Publisher planner_pub = nh.advertise<geometry_msgs::Pose>("reference_trajectory", 2);
    
    
    ros::Rate loop_rate(2);
    
    //TODO publish trajectory, subscribe to environment, publish controls as well
    while (ros::ok()) {
        
        rtmp.replan(rtmp.current_state);//TODO this needs to be a predicted state in the future
        ros::spinOnce();
//         planner_pub.publish(traj_msg);
        loop_rate.sleep();
    }
    
    
    return 0;
}
