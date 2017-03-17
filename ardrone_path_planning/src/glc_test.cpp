#include <glc_planner.h>
#include <glc_adapter.h>

//Control input space discretization for the ardrone
class ControlInputs2D: public glc::Inputs
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
  ControlInputs2D(int num_inputs)
  {
    glc::vctr u(3);
      for(int i=0;i<num_inputs;i++){
        u[0]=sin(2.0*i*M_PI/num_inputs);
        u[1]=cos(2.0*i*M_PI/num_inputs);
        addInputSample(u);
        std::cout << "Control[" << i << "]=(" << u[0] << "," << u[1] << ")" << std::endl;
    }
  }
};

class EuclideanHeuristic : public glc::Heuristic
{
public:
  EuclideanHeuristic(glc::vctr& _goal){goal=_goal;}
  double costToGo(const glc::vctr& state){
    return std::max(0.0,sqrt(glc::sqr(goal[0]-state[0])+glc::sqr(goal[1]-state[1]))-1.01);//offset by goal radius
  }
};

int main() 
{
    //Resolution for generalized label correcting method
    glc::Parameters alg_params;
    alg_params.res=8;
    alg_params.control_dim = 2;
    alg_params.state_dim = 2;
    alg_params.depth_scale = 100;
    alg_params.dt_max = 0.2;
    alg_params.max_iter = 10000;
    alg_params.time_scale = 8;
    alg_params.partition_scale = 10;
    alg_params.x0 = glc::vctr({0.0,0.0});
    
    //Create a dynamic model
    glc::SingleIntegrator dynamic_model(alg_params.state_dim,alg_params.dt_max);
    
    //Create the control inputs
    ControlInputs2D controls(alg_params.res);
    
    //Create the cost function
    glc::MinTimeCost performance_objective;
    
    //goal radius
    double goal_radius = 1.0;
    //create goal object
    glc::SphericalGoal goal(alg_params.state_dim,goal_radius);
    //set goal location
    glc::vctr xg({10.0,10.0});
    goal.setGoal(xg);
    
    //Create the obstacles
    glc::NoObstacles obstacles;
    
    //Create a heuristic for the current goal
//     glc::ZeroHeuristic heuristic;
    EuclideanHeuristic heuristic(xg);
    glc::GLCPlanner planner(&obstacles,
                                     &goal,
                                     &dynamic_model,
                                     &heuristic,
                                     &performance_objective,
                                     alg_params,
                                     controls.points);
    
    glc::PlannerOutput out;
    planner.Plan(out);
    std::cout << "running time " << out.time << std::endl; 
    //Dig out the solution and print it
    glc::printTraj( planner.recoverTraj( planner.pathToRoot(true) ) );
    return 0;
}
