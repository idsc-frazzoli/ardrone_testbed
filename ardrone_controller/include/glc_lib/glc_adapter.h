#ifndef GLC_ADAPTER_H
#define GLC_ADAPTER_H

#include "glc_interface.h"

namespace glc{
    
    class ZeroHeuristic: public Heuristic
    {
    public:    
        double costToGo(const vctr& x0) override
        {
            return 0;
        }
    };
    
    class MinTimeCost: public CostFunction //Min time 
    {
    public:
        MinTimeCost():CostFunction(0.0){}
        
        double cost(const Trajectory& p, const vctr& u) override 
        {
            return p.getDuration();
        }
    };
    
    class SphericalGoal: public GoalRegion //Origin centered ball
    {
        double goal_radius, goal_radius_sqr;
        vctr error;
        vctr x_g;
    public:
        
        SphericalGoal(const int& _state_dim, const double& _goal_radius):x_g(_state_dim,0.0)
        {
            //state_dim=_state_dim;
            goal_radius=_goal_radius;
            goal_radius_sqr=sqr(goal_radius);
            error.resize(_state_dim);
        }
        
        bool inGoal(const vctr& state, const double& t) override
        {
            error=diff(x_g,state); // TODO subtracting two vectors with different sizes
            return dot(error,error)<goal_radius_sqr;
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
        
        void setGoal(vctr& _x_g)
        {
            x_g=_x_g;
        }
        
        vctr getGoal()
        {
            return x_g;
        }
    };
    
    class NoObstacles: public Obstacles //No obs 
    {
    public:        
        bool collisionFree(const Trajectory& x, int* last=NULL) override
        {
            return true;
        }
        bool collisionFree(const glc::vctr& x, const double& t) override
        {
            return true;
        }
        
    };
    
}//end namespace

#endif