// dh: this file can probably be removed. it is superseded by more recent version of linear_system.h
#ifndef LINEAR_SYSTEM_H
#define LINEAR_SYSTEM_H
#include<cmath>
#include<stdio.h>
#include<iostream>
#include<vector>
#include<valarray>

namespace sys{
typedef std::valarray<double> array;
  
//CT-LTI-SISO system that is updated based on (input,time) pairs
//denominator[0]y(s)+denominator[1]y'(s)+...+denominator[k]y^(k)(s) = numerator[0]u(x)+...
class SisoSystem{
  array numerator;
  array denominator;
  array input_times;
  array inputs;
  
  //Time domain stuff (e.g. (y(0),y'(0),y''(0)...)
  array input_derivatives;
  array output_derivatives;
public:
  SisoSystem(const array& _numerator, const array& _denominator):numerator(_numerator),denominator(_denominator), input_derivatives(0.0,_numerator.size()),output_derivatives(0.0,_numerator.size()){}//TODO ctor with init. cond.
    
  void printState(){
    std::cout << "Input derivatives: (";
    for(int i=0;i<numerator.size();i++){
      std::cout << input_derivatives[i] << ",";
    }
    std::cout << ")" << std::endl;
    std::cout << "Output derivatives: (";
    for(int i=0;i<denominator.size();i++){
      std::cout << output_derivatives[i] << ",";
    }
    std::cout << ")" << std::endl;
  }  
  
};
}

#endif
