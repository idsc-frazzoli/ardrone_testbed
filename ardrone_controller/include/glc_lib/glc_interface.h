#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

//Includes
#include "math_utils.h"
#include "glc_utils.h"

namespace glc{
    
    class Inputs
    {
    public:
        std::deque<vctr> points;
        
        void addInputSample(vctr& _input)
        {
            points.push_back(_input);
        }
    };
    
    class Heuristic
    {
    public:
        
        virtual double costToGo(const vctr& x0)=0; 
    }; 
    
    class CostFunction
    {
        const double lipschitz_constant;
    public:
        CostFunction(double _lipschitz_constant):lipschitz_constant(_lipschitz_constant){}
        
        virtual double cost(const traj& candidate, const vctr& u)=0;
        
        double getLipschitzConstant()
        {
            return lipschitz_constant;
        }
    };
    
    class GoalRegion
    {
        
    public:
        virtual bool inGoal(const traj& x, int* last=NULL)
        { 
            for(int i=0;i<x.time.size();i++) {
                if(inGoal(x.states[i], x.time[i])){
                    if(last)
                        *last = i;
                    return true;
                }
            }
            return false;
        }
        
        virtual bool inGoal(const glc::vctr& x, const double& t)=0;
    };
    
    class Obstacles 
    {
    public:
        int counter=0;
        //check pointwise for collision    
        virtual bool collisionFree(const traj& x, int* last=NULL)
        { 
            for(int i=0;i<x.time.size();i++) {
                if(not collisionFree(x.states[i], x.time[i]))
                {
                    if(last)
                        *last = i;
                    return false;
                }
            }
            return true;
        }
        
        virtual bool collisionFree(const glc::vctr& x, const double& t)=0;
        
        
    };
    

    
    //Vector field with numerical integration
    class DynamicalSystem
    {
    public:
        //maximum time step
        const double max_time_step;
        //usage counter
        int sim_counter=0;
        
        DynamicalSystem(double _max_time_step):max_time_step(_max_time_step){}
        
        virtual void flow(vctr& dx, const vctr& x,const vctr& u)=0;
        
        virtual double getLipschitzConstant()=0;
        
        //Forward integration step
        virtual void step(vctr& x_plus, const vctr& x, const vctr& u, const double dt)=0;
        
        
        void sim(glc::traj& sol, double t0, double tf, const vctr& x0, const vctr& u)
        {
            assert(tf>t0);
            
            //compute minimum number of steps to satisfy dt<dt_max
            double num_steps=ceil((tf-t0)/max_time_step);
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
        
        //integrate forward for tspan from x0 with constant u
        traj sim(double t0, double tf, const vctr& x0, const vctr& u)
        {
            traj sol;
            sim(sol, t0, tf, x0, u);
            
            return sol;
        }
        
    };
    
    class EulerIntegrator : public DynamicalSystem
    {
        vctr xdot;
    public:
        EulerIntegrator(double _max_time_step):DynamicalSystem(_max_time_step){}
        void step(vctr& x_plus, const vctr& x, const vctr& u, const double dt) override
        {  
            flow(xdot,x,u);
            x_plus = x+dt*xdot;
        }
    };
    
    class RungeKutta4 : public DynamicalSystem
    {
        vctr k1,k2,k3,k4,temp;
    public:
        RungeKutta4(double _max_time_step):DynamicalSystem(_max_time_step){}
        
        void step(vctr& x_plus, const vctr& x, const vctr& u, const double dt) override
        {  
            static double SIXTH = 0.166666666666666;
            
            flow(k1,x,u);
            temp=x+0.5*dt*k1;
            flow(k2,temp,u);
            temp=x+0.5*dt*k2;
            flow(k3,temp,u);
            temp=x+dt*k3;
            flow(k4,temp,u);
            temp=k1+2.0*k2+2.0*k3+k4;
            
            x_plus = x+dt*SIXTH*temp;
        }
    };
    
    
    class SingleIntegrator: public glc::EulerIntegrator
    {
    public:
        SingleIntegrator(const double& _max_time_step): EulerIntegrator(_max_time_step) {}
        
        void flow(vctr& dx, const vctr& x, const vctr& u) override
        {
            dx=u;
        }
        
        double getLipschitzConstant() override {
            return 1.0;
        }
    };
    
}

#endif