
#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

//Includes
#include "math_utils.h"
#include "glc_utils.h"

//Pick numerical integration with preprocessor directive
#define RK4 0
#define WRAP 0
#define SIXTH 0.166666666666666

namespace glcm{
    
    ////////////////////////////////////////////////////////////////////////
    //////////////////////// Inputs Omega_R  ///////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    class inputs
    {
    public:
        std::deque<vctr> points;
        vctr u;
        
        inputs(int control_dim)
        {
            u.resize(control_dim);
            points.clear();
            return;
        }
    };
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////    Heuristic    ///////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    class heuristic
    {
    public:
        std::vector<double> center;
        double dist,radius;
        heuristic()
        {
            return;
        }
        virtual double cost_to_go(const vctr& x0)=0; 
    }; 
    
    class default_heuristic: public heuristic//TODO redundant
    {
    public:
        
        default_heuristic()
        {}
        default_heuristic(const vctr& _x_g, const double& _rad)
        {
            radius=_rad;
            center=_x_g;
        }
        
        double cost_to_go(const vctr& x0)
        {
            return 0;
        }
    };
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////  Cost function  ///////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    class costFunction
    {
    public:
        int n,m;
        double Lip_cost;
        
        
        virtual double cost(const traj& candidate, const vctr& u)=0;
    };
    
    class default_cost: public costFunction //Min time 
    {
    public:
        
        default_cost()
        {
            n=999;
            m=999;
            Lip_cost=0.0;//Min time 
        }
        default_cost(const int& _n, const int& _m)
        {
            n=_n;
            m=_m;
        }
        
        double cost(const traj& p, const vctr& u)
        {
            return p.time.back()-p.time.front();
        }
    };
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////////      Goal       ///////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    class goalRegion
    {
        
    public:
        int state_dim;
        glcm::vctr x_g;
        goalRegion(){};
        
        virtual bool inGoal(const traj& x, int* last=NULL){ 
            for(int i=0;i<x.time.size();i++) {
                if(inGoal(x.states[i], x.time[i])){
                    if(last)
                        *last = i;
                    return true;
                }
            }
            return false;
        }
        
        virtual bool inGoal(const glcm::vctr& x, const double& t)=0;
        
    };
    
    class default_goal: public goalRegion //Origin centered ball
    {
        double goal_radius, goal_radius_sqr;
        std::vector<double> error;
        std::vector<double> x_g;
    public:
        
        default_goal(const int& _state_dim, const double& _goal_radius)
        {
            state_dim=_state_dim;
            goal_radius=_goal_radius;
            goal_radius_sqr=sqr(goal_radius);
            x_g.clear();
            error.resize(state_dim);
            std::vector<double> zeros(state_dim, 0.0);
            x_g = zeros;
        }
        
        bool inGoal(const vctr& state, const double& t){
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
        
        std::vector<double> getGoal()
        {
            return x_g;
        }
    };
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////  Pointwise Constraints  ///////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    class obstacles 
    {
    public:
        int counter=0;
        //check pointwise for collision    
        virtual bool collisionFree(const traj& p)=0;
        
    };
    
    class default_obs: public obstacles //No obs 
    {
    public:
        
        default_obs()
        {
        }
        bool collisionFree(const traj& p)
        {
            counter++;
            return true;
        }
    };
    
    ////////////////////////////////////////////////////////////////////////
    ////////////////////    Dynamical System     ///////////////////////////
    ////////////////////////////////////////////////////////////////////////
    
    //Vector field with numerical integration
    class dynamical_system
    {
    public:
        //Solution to controlled ode
        traj sol;
        //maximum time step
        double dt_max;
        //dimension of the state space
        unsigned int n;
        //dimension of input space
        unsigned int m;
        //Lipschitz constant in state of vector field
        double Lip_flow;
        //temporary variables for integration
        vctr k1,k2,k3,k4,temp;
        vctr xdot;
        //usage counter
        int sim_counter;
        
        virtual void flow(vctr& dx, const vctr& x,const vctr& u)=0;
        
        //Forward integration step
        void step(vctr& x_plus, const vctr& x, const vctr& u, const double dt)
        {  
            #if RK4
            flow(k1,x,u);
            temp=x+0.5*dt*k1;
            flow(k2,temp,u);
            temp=x+0.5*dt*k2;
            flow(k3,temp,u);
            temp=x+dt*k3;
            flow(k4,temp,u);
            temp=k1+2.0*k2+2.0*k3+k4;
            
            x_plus = x+dt*SIXTH*temp;
            #else
            flow(xdot,x,u);
            x_plus = x+dt*xdot;
            #endif
            
            #if WRAP
            if(x_plus[0]<-M_PI)
                x_plus[0]+=2*M_PI;
            else if(x_plus[0]>M_PI)
                x_plus[0]-=2*M_PI;
            if(x_plus[1]<-M_PI)
                x_plus[1]+=2*M_PI;
            else if(x_plus[1]>M_PI)
                x_plus[1]-=2*M_PI;
            #endif
            
        }
        
        //integrate forward for tspan from x0 with constant u
        traj sim(double t0, double tf, const vctr& x0, const vctr& u)//TODO return a pointer so traj isn't copied so much
        {
            assert(tf>t0);
            //compute minimum number of steps to satisfy dt<dt_max
            double num_steps=ceil((tf-t0)/dt_max);
            double dt=(tf-t0)/num_steps;
            
            //resize solution
            sol.states.resize(num_steps+1);
            sol.time.resize(num_steps+1);
            sol.time[0]=t0;
            
            //set initial condition
            sol.states.at(0)=x0;
            //integrate
            for(int i=0;i<num_steps;i++)
            {
                step(sol.states.at(i+1),sol.states.at(i),u,dt);
                sol.time.at(i+1)=sol.time.at(i)+dt;
            }
            sim_counter++;
            return sol;
        }
        
        void sim(glcm::traj& sol, double t0, double tf, const vctr& x0, const vctr& u)//TODO return a pointer so traj isn't copied so much
        {
            //compute minimum number of steps to satisfy dt<dt_max
            double num_steps=ceil((tf-t0)/dt_max);
            double dt=(tf-t0)/num_steps;
            //resize solution
            sol.states.resize(num_steps+1);
            sol.time.resize(num_steps+1);
            
            //set initial condition
            sol.states.at(0)=x0;
            //integrate
            for(int i=0;i<num_steps;i++)
            {
                step(sol.states.at(i+1),sol.states.at(i),u,dt);
                sol.time.at(i+1)=sol.time.at(i)+dt;
            }
            sim_counter++;
            return;
        }
        
        //constructors
        dynamical_system()
        {
            if(WRAP)
            {
                printf("Warning. Using S^1 x R^(n-1) instead of R^n for state space \n");
            }
        }
        dynamical_system(glcm::parameters& params)
        {
            dt_max=params.dt_max;
            n=params.state_dim;
            m=params.control_dim;
            if(WRAP)
            {
                printf("Warning. Using S^1 x R^(n-1) instead of R^n for state space \n");
            }
        }
    };
    
    
    class default_sys: public glcm::dynamical_system
    {
    public:
        void flow(vctr& dx, const vctr& x, const vctr& u)
        {
            //Single Integrator Model
            for(int i=0;i<n;i++)
            {
                dx.at(i)=u.at(i);
            }
            return;
        }
        
        default_sys(const int& state_dim)
        {
            sim_counter=0;
            Lip_flow=1.0;
            dt_max=1.0;
            n = state_dim;
            m = state_dim;
            xdot.resize(n);
            k1.resize(n);k2.resize(n);k3.resize(n);k4.resize(n);
            temp.resize(n);
        }
    };
    
}

#endif