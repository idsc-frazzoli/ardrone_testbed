#ifndef GLC_UTILS_H
#define GLC_UTILS_H

#include "math_utils.h"
#include <boost/lexical_cast.hpp>
#include <memory>
#include <chrono>
#include <random>
#include <set>
#include <deque>
#include <algorithm> 


namespace glc{
    
    class Trajectory
    {
        std::vector<vctr> states;
        std::vector<double> time;
    public:
        
        void clear()
        {
            states.clear();
            time.clear();
        }
        
        void resize(int n)
        {
            states.resize(n);
            time.resize(n);
        }
        
        void reserve(int n)
        {
            states.reserve(n);
            time.reserve(n);
        }
        
        void push(const Trajectory& tail)
        {
            //states.reserve(states.size()+tail.states.size());
            states.insert(states.end(), tail.states.begin(), tail.states.end() );
            //time.reserve(time.size()+tail.time.size());
            time.insert(time.end(), tail.time.begin(), tail.time.end() );
        }
        
        void pop_back()
        {
            states.pop_back();
            time.pop_back();
            //return;
        }
        
        //get last element
        void back(vctr& x, double& t) {
            x=states.back();
            t=time.back();
        }
        
        int size() const
        {
            return time.size();
        }
        
        void get(int index, vctr& x, double& t) const
        {
            x=states[index];
            t=time[index];
        }
        const vctr& getState(int index) const
        {
            return states[index];
        }
        
        void set(int index, const vctr& x, double t) {
            states[index]=x;
            time[index]=t;
        }
        
        void push_back(const vctr& x, const double t)
        {
            states.push_back(x);
            time.push_back(t);
        }
        const double& getTime(int index) const
        {
            return time[index];
        }
        
        double getDuration() const 
        {
            return time.back() - time.front();
        }
        
        double getEndTime() const 
        {
            return time.back();
        }
        
        double getDurationFrom(int index) const
        {
            return time.back() - time.at(index);
        }
    };

// TODO move to glc_debug.h
    //Write trajectory out on the screen
    void printTraj(const Trajectory& sol)
    {
        printf("\n*****   Trajectory   *****\n");
        printf("time:  state:");
        printf("\n");
        for(int i=0;i<sol.size();i++)
        {
            vctr x;
            double t;
            sol.get(i, x, t);
            printf("%4.2f:  (",t);
            for(int j=0;j<x.size();j++)
            {
                printf("%4.2f, ",x[j]);
            }
            printf(")\n");
        }
        printf("**********************************\n");
    }
    
    class Parameters
    {
    public:
        //Initial condition
        vctr x0;
        //Discretization resolution
        int res;
        //State space dimension
        int state_dim;
        //Input space dimension
        int control_dim;
        //Maximum iterations
        int max_iter;
        //Change time coordinate to be appropriate
        double time_scale;
        //Initial partition size
        double partition_scale;
        //Adjust initial depth limit
        int depth_scale;
        //integration step
        double dt_max;
        //scaling of grid
        
        void print_params()
        {
            std::cout << "state_dim " << state_dim << std::endl;
            std::cout << "control_dim " << control_dim << std::endl;
            std::cout << "res " << res << std::endl;
            std::cout << "max_iter " << max_iter << std::endl;
            std::cout << "time_scale " << time_scale << std::endl;
            std::cout << "partition_scale " << partition_scale << std::endl;
            std::cout << "depth_scale " << depth_scale << std::endl;
            std::cout << "dt_max_scale " << partition_scale << std::endl;
            std::cout << "size of x0 " << x0.size() << std::endl;
            
            return;        
        }
    };
    
    class node; // TODO rename to Node
    typedef std::shared_ptr<node> nodePtr; 
    
    class node
    {
    public:
        static const nodePtr inf_cost_node;
        
        nodePtr parent;
        std::vector< nodePtr > children;//TODO maybe std::unordered_map?
        vctr state;
        double time;
        double cost;
        double merit;
        int u_idx; // index of control input from parent
        int depth;
        
        //Constructors
        node(int card_omega, double cost_=0.0, double t_=0.0, double merit_=0.0)
        {
            parent=nullptr;
            cost=cost_;
            time=t_;
            merit=merit_;
            children.resize(card_omega);
        }
        
        node(int _card_omega, int _control_index, double _cost, double _cost_to_go, vctr& _state, double _time) : children(_card_omega), cost(_cost), merit(_cost_to_go+_cost), time(_time), parent(nullptr), state(_state),u_idx(_control_index){}
        
        node(const vctr& _state, const int control, 
             double rootCost, double time, int card_omega):
             node(card_omega, rootCost, time, 0.0)
             {
                 state=_state;
                 u_idx=control;
             }
             
        //root constructor
        node(Parameters params,int card_omega):children(card_omega),state(params.x0),depth(0){}
    };
    
    const nodePtr node::inf_cost_node(new node(0, DBL_MAX/2, 0.0, DBL_MAX/2));//TODO used?
    
    bool compareMerit(const nodePtr& node1, const nodePtr& node2){
      return node1->merit<node2->merit;
    }
    class NodeMeritOrder
    {
        
    public:
        bool operator()(const nodePtr& node1, const nodePtr& node2){
            return not compareMerit(node1,node2);//negation so top of queue is min not max     
        }
        
    };
    
    class Domain
    {
    public:
        //serves as index of partition domain
        std::vector<int> coordinate;
        nodePtr label;
        
        std::priority_queue<nodePtr, std::vector<nodePtr>, NodeMeritOrder> candidates;
        
        Domain(){
            label = node::inf_cost_node;//Initialize to nullptr and don't use this thing
        }
        
        Domain(const nodePtr& _label){
            label = _label;
            coordinate=vec_floor(label->state);
        }
        
        bool empty(){
            return label == node::inf_cost_node;
        }
        
        //Lexicographical order of integer tuple for sorting stl set of domains
        bool operator<(const Domain& y) const{
            assert(coordinate.size()==y.coordinate.size());
            return std::lexicographical_compare <std::vector<int>::const_iterator, std::vector<int>::const_iterator>
            (coordinate.begin(), coordinate.end(), y.coordinate.begin(), y.coordinate.end());
        }
    };
    
    struct PlannerOutput
    {
        double cost;
        double time;
    };
  
}//end namespace
#endif
