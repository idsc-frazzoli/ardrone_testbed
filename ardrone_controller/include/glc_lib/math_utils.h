#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <iostream>
#include <cstdio>
#include <vector>
#include <cassert>
#include <ctime>
#include <cmath>
#include <float.h>
#include <string.h>
#include <time.h>
#include <sstream>
#include <fstream>
#include <string>


namespace glc{
typedef std::vector<double> vctr;

double sqr(const double x)
{
    return x*x;
}

double normSquare(const vctr& x)
{
    double y=0.0;
    for(int i=0;i<x.size();i++)
    {
        y+=x[i]*x[i];
    }
    
    return y;
}

void normSquare(const vctr& x, double& y)
{
    y=0.0;
    for(int i=0;i<x.size();i++)
    {
        y+=x[i]*x[i];
    }
    return;    
}

/*** Vector Operations ***/

vctr operator+(const vctr& x, const vctr& y)
{
    assert(x.size()==y.size());
    vctr z(x.size());
    for(int i=0;i<x.size();i++)
    {
        z.at(i)=x.at(i)+y.at(i);
    }
    
    return z;
}

vctr diff(const vctr& x, const vctr& y)
{
    //assert(x.size()==y.size()); // HACK put this back in later
    vctr z(x.size());
    for(int i=0;i<x.size();i++)
    {
        z.at(i)=x.at(i)-y.at(i);
    }
    
    return z;
}

vctr operator*(const double& c, const vctr& x)
{
    vctr z(x.size());
    for(int i=0;i<x.size();i++)
    {
        z.at(i)=c*x.at(i);
    }
    
    return z;
}

std::vector<int> vec_floor(vctr& x)
{
    std::vector<int> floored(x.size());
    for(int i=0;i<x.size();i++)
    {
        floored.at(i)=(int)floor(x.at(i));
    }
    return floored;
}

//inner product
double dot(const vctr& x,const vctr& y)
{
    assert(x.size()==y.size());
    double z=0;
    for(int i=0;i<y.size();i++)
    {
        z += x.at(i)*y.at(i);
    }
    
    return z;
}

double norm2(const vctr& x)
{
    double norm=0;
    for(int i=0; i<x.size(); i++)
    {
        norm=norm+sqr(x.at(i));
    }
    return std::sqrt(norm);
}

double norm_sqr(const vctr& x)
{
    double norm=0;
    for(int i=0; i<x.size(); i++)
    {
        norm=norm+sqr(x.at(i));
    }
    return norm;
}

}



#endif