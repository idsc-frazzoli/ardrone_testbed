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
    //Interface
    nodePtr best = node::inf_cost_node;
    nodePtr root_ptr;
    DynamicalSystem* dynamics;
    GoalRegion* goal;
    Obstacles* obs;
    CostFunction* cf;
    Heuristic* h;
    //Primitive data structures for GLC method
    std::priority_queue<nodePtr,std::vector<nodePtr>,NodeMeritOrder> queue;
    std::set<Domain> domain_labels;
    std::set<Domain>::iterator it;
    //
    int depth_limit;
    double partition_scale;
    double eta;
    double expand_time;
    //Run statistics
    bool foundGoal=false; bool live=true;
    Parameters params;
    std::deque<vctr> controls;
    int sim_count = 0;
    int coll_check = 0;
    int iter=0;
    clock_t run_time, tstart;
    double UPPER=DBL_MAX;
    
    //Planner tree handling functions
    void add_child(nodePtr parent, nodePtr child);//TODO move to node class
    void switch_root(); // temp
    void remove_subtree(nodePtr& root); // TODO domains should clear if empty
    std::vector<nodePtr> pathToRoot(bool forward=false);
    Trajectory recoverTraj(const std::vector<nodePtr>& path);
    
    
    //Planner methods
    void Expand();
    void Plan(PlannerOutput& out);
    void Plan();
    bool GetSolution(Trajectory& traj_out);
    
    
    //Constructor
    GLCPlanner(Obstacles* _obs, GoalRegion* _goal, DynamicalSystem* _dynamics, Heuristic* _h, CostFunction* _cf, const Parameters& _params, const std::deque<vctr>& _controls):
    params(_params), controls(_controls), dynamics(_dynamics), obs(_obs), goal(_goal), cf(_cf), h(_h){
      root_ptr = nodePtr(new node(params,controls.size()));
      root_ptr->merit=h->costToGo(root_ptr->state);//note the cost is zero
      
      Domain d0(root_ptr);
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
      if(dynamics->getLipschitzConstant()==0.0){
        eta = params.res*log(params.res)*log(params.res)/params.partition_scale;
      }
      else{
        eta = pow(params.res,1+dynamics->getLipschitzConstant())/params.partition_scale;
      }
      
      /////////*Print Parameters*/////////////
      std::cout << "\n\n\n\nPre-search summary:\n" << std::endl;
      std::cout << "      Expand time: " << expand_time << std::endl;
      std::cout << "      Depth limit: " << depth_limit <<  std::endl;
      std::cout << "      Domain size: " << 1.0/eta << std::endl;
      std::cout << "   Max iterations: " << params.max_iter << std::endl;
      
      tstart = clock();
    }
  };
  
  void GLCPlanner::add_child(nodePtr parent, nodePtr child){
    child->parent = parent;
    child->depth = parent->depth+1;
    child->time = (parent->time+expand_time);
    parent->children[child->u_idx] = child;
  }
  
  void GLCPlanner::Expand(){
    iter++;
    if(queue.empty()){
      std::cout << "---The queue is empty. Finished planning---" << std::endl;
      live=false;//TODO return this instead
      return;
    }
    nodePtr current_node = queue.top();
    queue.pop();
    
    //Goal checking
    if(goal->inGoal(current_node->state,current_node->time) and current_node->cost < best->cost)
    {
      run_time = clock() - tstart; // TODO 
      best=current_node;
      UPPER=current_node->cost;
      foundGoal=true;
      live=false;//only for best-first search
      std::cout << "\n\nFound goal at iter: " << iter << std::endl;
      std::cout << "   solution cost: " << UPPER << std::endl;
      std::cout << "    running time: " << (float) run_time/ (float) CLOCKS_PER_SEC << std::endl;
      std::cout << "Simulation count: " << dynamics->sim_counter << std::endl;
      std::cout << "Collision checks: " << obs->collision_counter << std::endl;
      std::cout << "     Size of set: " << domain_labels.size() << std::endl;
      std::cout << "   Size of queue: " << queue.size() << std::endl;
    }
    
    if(current_node->depth >=depth_limit or iter>params.max_iter)
    {
      std::cout << "---exceeded depth or iteration limit---" << std::endl;
      live=false;
      return;
    }
    
    //A set of domains visited by new nodes made by expand
    std::set<Domain*> domains_needing_update; 
    std::map<nodePtr, Trajectory> traj_from_parent;
    
    //Expand top of queue and store arcs in set of domains
    for(int i=0;i<controls.size();i++){
      Trajectory new_traj;
      dynamics->sim(new_traj, current_node->time, current_node->time+expand_time , current_node->state, controls[i]);
      glc::vctr terminal_state;
      double terminal_time;
      new_traj.back(terminal_state,terminal_time);//assigns terminal_state and time to args
      
      nodePtr new_arc(new node(controls.size(),i,cf->cost(new_traj, controls[i])+current_node->cost, h->costToGo(new_arc->state), terminal_state, terminal_time));
      
      traj_from_parent[new_arc] = new_traj;
      
      vctr w = eta * new_arc->state;
      Domain d_new;
      d_new.coordinate = vec_floor( w );
      
      //Get the domain for the coordinate or create it.
      Domain& bucket = const_cast<Domain&>( *(domain_labels.insert(d_new).first) );
      //Add to a queue of domains that need inspection
      domains_needing_update.insert(&bucket);
      if(compareMerit(new_arc,bucket.label)){
        bucket.candidates.push(new_arc);
      }     
    }
    //Go through the new trajectories and see if there is a possibility for relabeling before collcheck
    for(auto& open_domain : domains_needing_update){
      Domain& current_domain = *open_domain; // HACK[?]
      //We go through the queue of candidates for relabeling/pushing in each set
      bool found_best = false;
      while( (not found_best) and (not current_domain.candidates.empty()))
      {
        //If the top of the candidate queue is cheaper than the label we should coll check it
        if(compareMerit(current_domain.candidates.top(),current_domain.label)){
          const nodePtr& best_relabel_candidate = current_domain.candidates.top(); 
          if(obs->collisionFree(traj_from_parent[best_relabel_candidate])){
            add_child(current_node, best_relabel_candidate);
            queue.push(best_relabel_candidate);//anything coll free at this point goes to queue
            if(!found_best){
              found_best = true;
              current_domain.label = best_relabel_candidate;
            }
          }
        }
        current_domain.candidates.pop();//TODO if current_domain.label = NULL (or whatever the default is) delete the domain.
      }
      
      if(current_domain.empty()){
        domain_labels.erase(current_domain);
      }
    }
    
    return;
  }
  
  
  void GLCPlanner::Plan(){
    while(live){
      Expand();
    }
    
    return;
  }
  
  void GLCPlanner::Plan(PlannerOutput& out){
    GLCPlanner::Plan();
    out.cost=UPPER;
    out.time=(float) run_time/ (float) CLOCKS_PER_SEC;
    return;
  }
  
  //get the nodePtr path to the root with the order specified by foward
  std::vector<nodePtr> GLCPlanner::pathToRoot(bool forward){
    nodePtr currentNode = best;
    std::vector<nodePtr> path;
    while( not (currentNode->parent == nullptr) ){
      path.push_back(currentNode);
      currentNode=currentNode->parent;
    }
    path.push_back(currentNode);
    
    if(forward){std::reverse(path.begin(),path.end());}
    
    return path;
  }
  
  //return the planned trajectory
  Trajectory GLCPlanner::recoverTraj(const std::vector<nodePtr>& path)
  {
    
    Trajectory opt_sol;
    if(path.size()<2){
      opt_sol.clear();
      return opt_sol;
    }
    
    //recalculate arcs connecting nodes
    for(int i=0; i<path.size()-1;i++){
      double t0=path[i]->time;
      double tf=t0+expand_time;
      Trajectory arc;
      dynamics->sim(arc, t0, tf, path[i]->state,controls[path[i+1]->u_idx]);
      if(i<path.size()-2){
        arc.pop_back();
      }
      opt_sol.push(arc);
    }
    
    return opt_sol;
  }
}//close namespace
#endif