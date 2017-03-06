#ifndef _KDTREE_H_
#define _KDTREE_H_

#include <memory>
#include <utility>
#include <vector>
#include <chrono>
#include <algorithm>
#include <set>
#include <queue>
#include <ostream>


#include "../utils/utils.h"

namespace kdtree{
    
    typedef utils::numT numT;
    typedef utils::vctr vctr;
    typedef utils::path path;  
    
    
    
    //~~~~~~~~~~~~~~~~~~~vertex class~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    class vertex
    {
    public:
        
        vctr coord;
        vctr normal;
        
        size_t depth;
        size_t id;
        
        // Tree pointers
        std::shared_ptr<vertex> parent;
        std::shared_ptr<vertex> children[2];
        
        vertex(const vctr& _coord)
        {
            coord=_coord;
            normal.resize(coord.size());
            return;
        }
        
        vertex(const vctr& _coord, const vctr& _normal)
        {
            coord=_coord;
            normal=_normal;
            return;
        }
        
        ~vertex(){ // HACK!!
//             std::cout << "Deleting node with coords " << utils::toString(coord) << std::endl;
        }
        
        
        bool isLeaf(){
            return !(children[0].get() || children[1].get());
        }
        
        bool isRoot(){
            return !parent.get();
        }
        
        std::string toString(){
            return utils::toString(coord);
        }
        
        //needed for std::map
        bool operator<(const vertex& b) const{
            return std::lexicographical_compare(coord.begin(),coord.end(),b.coord.begin(),b.coord.end());
        }
    };
    typedef std::shared_ptr<vertex> vertexPtr;
    //~~~~~~~~~~~~~~~~~~~vertex class~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    
    
    
    //~~~~~~~~~~~~~~~~~~~~~~~queue for nn query~~~~~~~~~~~~~~~~~~~~~~~//
    template <class vertexPtr, class numT>
    struct query_node
    {
        numT score;
        vertexPtr vtx_ptr;
        
        query_node(){return;}
        
        query_node(const numT& _score, const vertexPtr& _vtx_ptr)//TODO inherit numT from kdtree
        {
            score=_score;
            vtx_ptr=_vtx_ptr;
        }
        // Comparison
        bool operator<(const query_node& b) const{
            return  b.score > score;
        }
    };
    
    template <class vertexPtr, class numT>
    class query_queue
    {
        numT queue_score;
    public:
        int query_size;
        std::priority_queue< query_node<vertexPtr,numT> > queue;
        query_queue()
        {
            query_size=1;
            queue_score = 0.0;
            return;
        }
        void set_size(int size)
        {
            query_size=size;
            return;
        }
        //Distance of most distant point in queue
        numT get_score()
        {
            if(queue.size()<query_size)
            {
                return std::numeric_limits<numT>::max();
            }
            return queue_score;            
        }
        void insert(numT score, vertexPtr v)
        {
            query_node<vertexPtr,numT> newnode(score, v);
            queue.push(newnode);
            //if too big, get rid of most distant point
            if(queue.size()>query_size)
            {
                queue.pop();
            }
            queue_score = queue.top().score;
            return;
        }
        void clear()
        {
            queue=std::priority_queue< query_node<vertexPtr,numT> >();
        }
        
        
    };
    
    template <class vertexPtr, class numT>
    struct query_results {
        vctr querypoint;
        query_queue<vertexPtr,numT> BPQ;
    };
    //~~~~~~~~~~~~~~~~~~~~~~~queue for nn query~~~~~~~~~~~~~~~~~~~~~~~//
    
    
    //~~~~~~~~~~~~~~~~~kD-tree class(insertion and nn query)~~~~~~~~~~//
    class Kdtree{
        
        bool inPositiveHalfspace(const vctr& center, const vctr &pivot, const vctr& n) const{
            
            return utils::dot_product(utils::vec_diff(center, pivot), n)>0;
        }
        
        bool ballHyperplane(const vctr& center, const numT& radius, const vctr& pivot, const vctr& n)
        {
            d_count++;
            
            bool check_sibling = fabs(utils::dot_product(utils::vec_diff(center, pivot), n))<=radius;
            return fabs(utils::dot_product(utils::vec_diff(center, pivot), n))<=radius;
        }
        
        void addChild(vertexPtr parent, vertexPtr child, bool which){
            parent->children[which] = child;
            child->parent = parent;
            child->depth = parent->depth+1;
        }
        
        //First call in recursive function
        void descend(vctr& coord, vertexPtr& parent_o, bool& side_o) const {
            descend(coord, root, parent_o, side_o);
        }

        //Recursive function to descend binary tree
        void descend(vctr& x, const vertexPtr& node, vertexPtr& parent_o, bool& side_o) const {
            bool which = inPositiveHalfspace(x, node->coord, node->normal);
            if(!node->children[which].get()){
                parent_o = node;
                side_o = which;
                return;
            }
            descend(x, node->children[which], parent_o, side_o);
        }
        
        
    public:
        vertexPtr root;//TODO change to template vertex object
        int dimension;
        unsigned int d_count;

        Kdtree()
        {
            d_count=0;
            return;
        }
        
        Kdtree(const int& _dimension)
        {
            d_count=0;
            dimension=_dimension;
            return;
        }
        

        
        void insert(vertexPtr& v){
            if(!root.get())
            {
                assert(v->coord.size()==dimension);
                vctr node_normal(dimension,0.0);//TODO template on numT as well
                node_normal[0]=1.0;
                v->normal=node_normal;
                root = v; 
                dimension = root->coord.size();
            }
            else
            {
                
                vertexPtr parent; bool side;
                //descend the kdtree to find the leaf to be the parent
                descend(v->coord, parent, side);
                //Select the normal direction to assign based on parent
                vctr node_normal(dimension,0.0);
                node_normal[(parent->depth+1)%dimension]=1.0;
//                 printf("normal: (%f,%f)\n",node_normal[0],node_normal[1]);
                v->normal=node_normal;
                
                //make parent the parent in v and v the child of parent according to side
                vertexPtr child = v;
                addChild(parent, child, side);
            }
        }
        
            
            query_results<vertexPtr,numT> query(const vctr& qp, size_t k){
            query_results<vertexPtr,numT> R;
            //Can't query empty tree
//             printf("--------------New query--------\n");
            
            if(!root.get()){return R;}
            
            R.querypoint = qp;
            R.BPQ.clear();
            R.BPQ.set_size(k);
            query(qp, root, R);      
            
            return R;
        }
        
        void query(const vctr& qp, vertexPtr& current, query_results<vertexPtr,numT>& R){ 
            //Determine which side of the current nodes hyperplane the qp is on
            bool side = inPositiveHalfspace(qp, current->coord, current->normal);
            
            //Descend recursively
            if(current->children[side].get())
            {
                query(qp, current->children[side], R);
            }
                
            //Pass recursive function once leaf is reached
            //Get distance to leaf node in cubicle
            numT point_score = utils::norm(utils::vec_diff(qp, current->coord));
            R.BPQ.insert(point_score, current);
            
            //Check if nearest ball intersects leaf hyperplane
            if(ballHyperplane(qp, R.BPQ.get_score(), current->coord, current->normal)/* or R.BPQ.queue.size()<R.BPQ.query_size*/)
            {
                if(current->children[1-side].get())
                    query(qp, current->children[1-side], R);
            }
        }
    };
    //~~~~~~~~~~~~~~~~~kD-tree class(insertion and nn query)~~~~~~~~~~//
    
}
#endif