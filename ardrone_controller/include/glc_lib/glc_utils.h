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
    
    class traj
    {
    public:
        std::vector<vctr> states;
        vctr time;
        
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
        
        void push(const traj& tail)
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
    };

// TODO move to glc_debug.h
    //Write trajectory out on the screen
    void print_traj(const traj& sol)
    {
        printf("\n*****   Trajectory   *****\n");
        printf("time:  state:");
        printf("\n");
        for(int i=0;i<sol.time.size();i++)
        {
            printf("%4.2f:  (",sol.time.at(i));
            for(int j=0;j<(sol.states[0]).size();j++)
            {
                printf("%4.2f, ",sol.states[i][j]);
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
        vctr umin;
        vctr umax;
        //scaling of grid
        std::vector<double> ugrid;
        
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
            std::cout << "size of umin " << umin.size() << std::endl;
            std::cout << "size of umax " << umin.size() << std::endl;
            
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
        std::vector< nodePtr > children;
        vctr x;
        double t;
        double cost;
        double merit;
        int u_idx; // index of control input from parent
        int depth;
        
        //Constructors
        node(int card_omega, double cost_=0.0, double t_=0.0, double merit_=0.0)
        {
            parent=nullptr;
            cost=cost_;
            t=t_;
            merit=merit_;
            children.resize(card_omega);
        }
        
        node(const vctr& state, const int control, 
             double rootCost, double time, int card_omega):
             node(card_omega, rootCost, time, 0.0)
             {
                 x=state;
                 u_idx=control;
             }
             
        //root constructor
        node(Parameters params,int card_omega):node(card_omega)
             {
                 children.resize(card_omega);
                 x.resize(params.state_dim);
                 x=params.x0;
                 depth = 0;
             }
             
             //              //Uniform cost order, default comparison of nodes
             //              bool operator<(node other)
             //              {
             //                  return merit>=other.merit;
             //                  return cost>=other.cost;
             //              }
             
    };
    
    const nodePtr node::inf_cost_node(new node(0, DBL_MAX/2, 0.0, DBL_MAX/2));
    
    class QueueOrder
    {
        
    public:
        bool operator()(const nodePtr& node1, const nodePtr& node2)
        {
            return node1->merit>=node2->merit;//A*     
        }
        
    };
    
    // struct candidate{
    //     glcm::node path;
    // //   glcm::traj traj;
    // //   double pathCost;
    // //   double edgeCost;//TODO needed?
    // };
    
    class cptrcompare{ // TODO rename
    public:
        bool operator() (const nodePtr& l, const nodePtr& r)
        {
            return l->cost > r->cost; // TODO ensure this yields top being smallest cost
        }
    };
    
    class Domain
    {
    public:
        //serves as index of partition domain
        std::vector<int> coordinate;
        nodePtr label;
        
        std::priority_queue<nodePtr, std::vector<nodePtr>, cptrcompare> candidates;
        
        Domain(){
            label = node::inf_cost_node;
        }
        
        Domain(const nodePtr& _label)
        {
            label = _label;
            coordinate=vec_floor(label->x);
        }
        
        bool empty(){
            return label == node::inf_cost_node;
        }
        
        //Lexicographical order of integer tuple for sorting stl set of domains
        bool operator<(const Domain& y) const
        {
            assert(coordinate.size()==y.coordinate.size());
            return std::lexicographical_compare <std::vector<int>::const_iterator, std::vector<int>::const_iterator>
            (coordinate.begin(), coordinate.end(), y.coordinate.begin(), y.coordinate.end());
        }
    };
    
    //Create a grid over the hyper-rectancle defined by umin and umax
    // TODO 
//     void grid_control(std::deque<vctr>& controls, vctr& umin, vctr& umax, std::vector<int>& RES, int open)
//     {
//         std::deque<vctr> bucket;
//         bucket.clear();
//         vctr u(umin.size());
//         int count=1;
//         for(int i=0;i<=RES[0]-open;i++)
//         {
//             u[0]=umin.at(0)+((double) i)*(umax.at(0)-umin.at(0))/RES[0];
//             bucket.push_back(u);
//         }
//         
//         for(int k=1;k<umin.size();k++)//repeat for each channel
//         {
//             //         count*=RES[k];
//             count =bucket.size();
//             for(int j=0;j<count;j++)//go through each element in bucket
//             {
//                 u=bucket.front();
//                 bucket.pop_front();
//                 for(int i=0;i<=RES[k]-open;i++)//and tack on all convex combos
//                 {
//                     u[k]=umin.at(k)+((double) i)*(umax.at(k)-umin.at(k))/RES[k];
//                     bucket.push_back(u);
//                 }
//             }
//         }
//         controls.clear();
//         std::copy(bucket.begin(), bucket.end(), std::back_inserter(controls));
//     }
    
    struct PlannerOutput
    {
        double cost;
        double time;
    };
    
//     std::vector< std::vector<double> > omegaR(const int m, const int exp, const int R)
//     {
//         
//         unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
//         std::default_random_engine generator(seed);
//         std::normal_distribution<double> distribution(0.0,1.0);
//         std::vector< vctr > controls(0);
//         double normal_rv,nearest;
//         vctr input(0);
//         for(int j=0;j<100*pow(R,m);j++)
//         {
//             for(int i=0;i<m;i++)
//             {
//                 normal_rv=0.0;
//                 while(normal_rv == 0.0)
//                 {
//                     normal_rv = (double) distribution(generator);
//                 }
//                 input.push_back(normal_rv);
//             }
//             input = (1.0/norm2(input))*input;
//             nearest=DBL_MAX/2.0;
//             for(int k=0;k<controls.size();k++)
//             {
//                 nearest=std::min( nearest,norm2(diff(controls[k],input)) );
//             }
//             if(nearest>=0.2/ (double) R)
//             {
//                 controls.push_back(input);
//             }
//             input.clear();
//         }
//         
//         return controls;
//     }
    
}//end namespace
#endif
