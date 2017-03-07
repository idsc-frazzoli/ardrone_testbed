#ifndef KDTREE_UTILS_H
#define KDTREE_UTILS_H

#include <valarray>
#include <iostream>
#include <cassert>

namespace kdtree{
    typedef double numT;
    typedef std::valarray<numT> point;
    
    //Usual dot product on R*n
    numT innerProduct(const point& x, const point& y)
    {
        assert(x.size()==y.size() && "dimension mismatch in kdtree");
        
        return (x*y).sum();
    }
    
    //Euclidan 2-norm 
    double norm2(const point& x)
    {
        return sqrt(innerProduct(x,x));
    }
    
}// end kdtree namespace

#endif