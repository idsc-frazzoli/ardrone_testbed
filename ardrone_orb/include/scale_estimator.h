/*
 * (c) 2017 
 * 
 * Authors: 
 * Daniel Gehrig (ETH Zurich)
 * Maximilian Goettgens (ETH Zurich)
 * Brian Paden (MIT)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining 
 * a copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR 
 * IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef ScaleEstimator_H
#define ScaleEstimator_H

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <ardrone_autonomy/Navdata.h>
#include <std_msgs/Float32.h>

#include <scale.h>
#include <linear_system.h>

using namespace std;

class ScaleEstimator
{

public:
    shared_ptr<LTI::SisoSystem> nav_z_;
    shared_ptr<LTI::SisoSystem> orb_z_;
    double orb_signal_=0;
    double nav_signal_=0;
    double filter_pole_ = 30 * M_PI * 2; // in radians

    deque<geometry_msgs::PoseStamped> orb_data_queue_;
    deque<ardrone_autonomy::Navdata> nav_data_queue_;
    bool first_orb_msg_=true;
    bool first_nav_msg_=true;

    // tuning parameters
    const double ransac_tolerance_ = 0.1;
    const double orb_tol_min_ = 0;
    const double nav_tol_min_ = 0.005;
    const double orb_tol_max_ = 0.1;
    const double nav_tol_max_ = 0.5;
    const double std_orb_=0.2;
    const double std_nav_=0.1;

    bool fixed_scale_ = false;
    double scale_ = 1; // has units m^-1
    vector<ScaleStruct> scale_vector_;
    const int max_counter_ = 350;
    int counter_ = 0;

    double filterScale ( vector<ScaleStruct> scale_vctr, double ratio, double std_orb, double std_nav , int cut_off );
    void estimateScale();
    void processQueue();
    void orbCallback(geometry_msgs::PoseStamped msg) {orb_data_queue_.push_front ( msg );};
    void navCallback(ardrone_autonomy::Navdata msg) {nav_data_queue_.push_front ( msg );};
    bool hasFixedScale() {return fixed_scale_;};
    double getScale() {return scale_;};
};

#endif
