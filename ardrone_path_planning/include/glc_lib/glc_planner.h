#ifndef GLC_PLANNER_H
#define GLC_PLANNER_H

#include <map>
#include <limits>
#include <memory>
#include <stack>
#include <queue>
#include <deque>
#include "glc_interface.h"
#include "glc_utils.h"

namespace glc{  
  
  class GLCPlanner
  {
  public:
    int num;
    //best path to goal
    nodePtr best;
    //initial condition
    nodePtr root_ptr;
    //pointer to the ode integrator object
    DynamicalSystem* dynamics;
    // goal region
    GoalRegion* goal;
    // obstacle
    Obstacles* obs;
    // cost function
    CostFunction* cf;
    // heuristic 
    Heuristic* h;
    //queue of nodes
    std::priority_queue<nodePtr,
    std::vector<nodePtr>,
    QueueOrder> queue;
    //create the set with type and compare function
    std::set<Domain> domain_labels;
    //iterator type for sets
    std::set<Domain>::iterator it;
    //upper bound on known optimal cost
    double UPPER;
    //tolerance on cost difference between two related controls
    double eps;
    //maximum search depth
    int depth_limit;
    //variable for the simulation time of the expand function
    double expand_time;
    //initial scaling of partition size
    double partition_scale;
    //Timer
    clock_t run_time, tstart;
    //iteration count
    int32_t iter;
    //If a solution is found
    bool foundGoal,live;
    //parameters
    Parameters params;
    //control grid
    std::deque<vctr> controls;
    //children expanded
    int sim_count = 0;
    int coll_check = 0;
    //eta function
    double eta;
    
    //PLanner tree handling functions
    void add_child(nodePtr parent, nodePtr child);//TODO move to node class
    void switch_root(); // temp
    void remove_subtree(nodePtr& root); // TODO domains should clear if empty
    std::vector<nodePtr> path_to_root(bool forward=false);
    Trajectory recoverTraj(const std::vector<nodePtr>& path);
    
    
    //Planner methods
    void expand();
    void Plan(PlannerOutput& out);
    void Plan();
    bool get_solution(Trajectory& traj_out);
    
    
    //ctor
    GLCPlanner(Obstacles* _obs, 
                       GoalRegion* _goal, 
                       DynamicalSystem* _dynamics, 
                       Heuristic* _h,
                       CostFunction* _cf,
                       const Parameters& _params,
                       const std::deque<vctr>& _controls
    ):
    params(_params), 
    dynamics(_dynamics),
    obs(_obs), 
    goal(_goal),
    cf(_cf),
    h(_h)
    {
      controls=_controls;
      best = node::inf_cost_node;
      UPPER=DBL_MAX/2.0;
      foundGoal=false;
      live=true;
      root_ptr = nodePtr(new node(params,controls.size()));
      root_ptr->merit=h->costToGo(root_ptr->x);
      Domain d0(root_ptr);
      //Add root to search containers
      queue.push(root_ptr);
      domain_labels.insert(d0);
      ////////////*Scaling functions*//////////////
      // 1/R
      expand_time=params.time_scale/(double)params.res;
      //h(R)
      depth_limit=params.depth_scale*params.res*floor(log(params.res));
      //eta(R) \in \little-omega (log(R)*R^L_f)
      eta = log(params.res)*pow(params.res,dynamics->getLipschitzConstant())/( params.partition_scale );
      partition_scale=eta/( params.partition_scale );
      //eps(R)
      if(cf->getLipschitzConstant()>0)
      {
        eps = (sqrt(params.state_dim)/partition_scale)*
        (dynamics->getLipschitzConstant())/(cf->getLipschitzConstant())*
        (params.res*exp(dynamics->getLipschitzConstant())-1.0);
      }
      else
      {
        eps=0;
      }
      
      /////////*Monitor Performance*/////////////
      std::cout << "\n\n\n\nPre-search summary:\n" << std::endl;
      std::cout << "        Threshold: " << eps << std::endl;
      std::cout << "      Expand time: " << expand_time << std::endl;
      std::cout << "      Depth limit: " << depth_limit <<  std::endl;
      std::cout << "      Domain size: " << 1.0/eta << std::endl;
      std::cout << "   Max iterations: " << params.max_iter << std::endl;
      
      iter=0;
      sim_count=0;
      tstart = clock();
    }
    
    
  };
  
  
  // TODO make member of node    
  void GLCPlanner::add_child(nodePtr parent, nodePtr child){
    child->parent = parent;
    child->depth = parent->depth+1;
    child->t = (parent->t+expand_time);
    parent->children[child->u_idx] = child;
  }
  
  void GLCPlanner::expand()
  {
    
    iter++;
    if(queue.empty()){
      std::cout << "---The queue is empty. Finished planning---" << std::endl;
      live=false;//TODO return this instead
      return;
    }
    
    nodePtr current_node = queue.top();
    queue.pop();
    
    if(current_node->depth >=depth_limit or iter>params.max_iter)
    {
      std::cout << "---exceeded depth or iteration limit---" << std::endl;
      live=false;
      return;
    }
    
    //A set of domains visited by new nodes made by expand
    std::set<Domain*> domains_needing_update; 
    std::map<nodePtr, Trajectory> traj_from_parent;
    
    //Expand top of queue and store arcs in 
    for(int i=0;i<controls.size();i++)
    {
      Trajectory new_traj;
      dynamics->sim(new_traj, current_node->t, current_node->t+expand_time , current_node->x, controls[i]);
      nodePtr new_arc(new node(controls.size()));//Move all this into constructor of new_arc
      new_arc->cost = cf->cost(new_traj, controls[i])+current_node->cost;
      
      new_arc->u_idx = i;
      new_traj.back(new_arc->x, new_arc->t);
      new_arc->merit = new_arc->cost + h->costToGo(new_arc->x);//comes after assigning x
      
      traj_from_parent[new_arc] = new_traj;
      
      vctr w = partition_scale * new_arc->x;
      Domain d_new;
      d_new.coordinate = vec_floor( w );
      
      
      Domain& bucket = const_cast<Domain&>( *(domain_labels.insert(d_new).first) );
      domains_needing_update.insert(&bucket);
      
      if(new_arc->cost <= bucket.label->cost+eps /* and mew_arc->t <= bucket.label->t*/)
      {
        bucket.candidates.push(new_arc);
      }     
    }
    
    for(auto& open_domain: domains_needing_update)
    {
      
      Domain& current_domain = *open_domain; // HACK[?]
      
      //We go through the queue of candidates for relabeling/pushing in each set
      bool found_best = false;
      while( not current_domain.candidates.empty())
      {
        //Check that the top of the domain's candidate queue is within the current threshold
        if(current_domain.candidates.top()->cost < current_domain.label->cost+eps){
          //                 if(current_domain.candidates.top()->merit < current_domain.label->merit+eps){
          
          const nodePtr& curr = current_domain.candidates.top(); 
          //Anything collision free within the threshold has to stay. Push to queue. 
          //The first one that's coll free becomes new label 
          if(obs->collisionFree(traj_from_parent[curr]))
          {
            add_child(current_node, curr);
            if(not foundGoal)
            {
              queue.push(curr);//anything coll free at this point goes to queue
            }
            //the cheapest one relabels the domain
            if(!found_best){
              found_best = true;
              current_domain.label = curr;
            }
            
            //Goal checking
            if(goal->inGoal(traj_from_parent[curr],&num) and curr->cost < best->cost)
            {
              run_time = clock() - tstart; // TODO 
              foundGoal=true;
              live=false;
              std::cout << "\n\nFound goal at iter: " << iter << std::endl;
              std::cout << "Simulation count: " << dynamics->sim_counter << std::endl;
              std::cout << "Collision checks: " << obs->collision_counter << std::endl;
              best=curr;
              //TODO not consistent with anything other than min-time cost here
              double tail_cost = traj_from_parent[curr].getDurationFrom(num-1)*
              (1.0+(cf->getLipschitzConstant())*sqr(controls[best->u_idx][0]));
              std::cout << "         Tail cost: " << tail_cost << std::endl;
              std::cout << "    cost from root: " << curr->cost << std::endl;
              std::cout << "          End time: " <<  traj_from_parent[curr].getEndTime() << std::endl;
              UPPER=curr->cost-tail_cost;
              
            }    
          }
          
        }
        current_domain.candidates.pop();//TODO if current_domain.label = NULL (or whatever the default is) delete the domain.
      }
      
      if(current_domain.empty()){
        // Prevent domains without a path 
        domain_labels.erase(current_domain);
      }
    }
    
    
    return;
  }
  
  void GLCPlanner::plan(PlannerOutput& out)
  {
    while(live)
    {
      expand();
    }
    out.cost=UPPER;
    out.time=(float) run_time/ (float) CLOCKS_PER_SEC;
    return;
  }
  
  void GLCPlanner::plan()
  {
    while(live)
    {
      expand();
    }
    
    return;
  }
  
  //get the nodePtr path to the root with the order specified by foward
  std::vector<nodePtr> GLCPlanner::path_to_root(bool forward)
  {
    nodePtr currentNode = best;
    std::vector<nodePtr> path;
    while( not (currentNode->parent == nullptr) )
    {
      path.push_back(currentNode);
      currentNode=currentNode->parent;
    }
    path.push_back(currentNode);
    
    if(forward)
      std::reverse(path.begin(),path.end());
    return path;
  }
  
  //return the planned trajectory
  Trajectory GLCPlanner::recoverTraj(const std::vector<nodePtr>& path)
  {
    
    Trajectory opt_sol;
    if(path.size()<2)
    {
      opt_sol.clear();
      return opt_sol;
    }
    
    //recalculate arcs connecting nodes
    for(int i=0; i<path.size()-1;i++)
    {
      double t0=path[i]->t;
      double tf=t0+expand_time;
      Trajectory arc;
      dynamics->sim(arc, t0, tf, path[i]->x,controls[path[i+1]->u_idx]);
      if(i<path.size()-2)
      {
        arc.pop_back();
      }
      opt_sol.push(arc);
    }
    
    return opt_sol;
  }
}//close namespace
#endif