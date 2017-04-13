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

#include <scale_estimator.h>

using namespace std;
typedef geometry_msgs::PoseStamped poseMsgStamped;
typedef geometry_msgs::Pose poseMsg;

double ScaleEstimator::filterScale ( vector<ScaleStruct> scale_vctr, double ratio, double std_orb, double std_nav , int cut_off ) {

    sort ( scale_vctr.begin(), scale_vctr.end() );
    counter_ = 0;

    double median = ( scale_vctr.size() < 5 ) ? 1 : scale_vctr[ ( scale_vctr.size() +1 ) /2].scale_;

    double S_yy_z, S_xx_z, S_xy_z;
    S_yy_z = S_xx_z = S_xy_z = 0;

    for ( int i=0; i<scale_vctr.size(); i++ ) {

        ScaleStruct s = scale_vctr[i];

        if ( scale_vctr.size() < 5 || ( s.scale_ > median * ratio && s.scale_ < median / ratio ) ) {
            S_yy_z += s.nav_z_ * s.nav_z_;
            S_xy_z += s.nav_z_ * s.orb_z_;
            S_xx_z += s.orb_z_ * s.orb_z_;

            counter_++;
        }
    }


    if ( counter_ > cut_off ) {
        string info = "Scale converged to " + to_string ( scale_ );
        ROS_INFO ( info.c_str() );
        fixed_scale_ = true;
    }

    return ScaleStruct::computeEstimator ( S_xx_z, S_yy_z, S_xy_z, std_orb, std_nav );
}

void ScaleEstimator::estimateScale() {
    scale_ = filterScale ( scale_vector_, ransac_tolerance_, std_orb_, std_nav_ , max_counter_ );
}

void ScaleEstimator::processQueue() {
    //Only process orb data if batch size is greater than 100
    if ( orb_data_queue_.size() <1 ) {
        return;
    }

    //numerator and denominator of transfer function
    LTI::array num ( 2 ),den ( 3 );
    num[0]=0;
    num[1]=filter_pole_ * filter_pole_;

    den[0]=filter_pole_ * filter_pole_;
    den[1]=filter_pole_ + filter_pole_;
    den[2]=1;

    while ( not orb_data_queue_.empty() ) {

        // get first orb msg
        geometry_msgs::PoseStamped orb_msg = orb_data_queue_.back();
        orb_data_queue_.pop_back();

        double t_orb = orb_msg.header.stamp.toSec();

        if ( first_orb_msg_ ) {
            first_orb_msg_ = false;

            orb_z_ = shared_ptr<LTI::SisoSystem> ( new LTI::SisoSystem ( num,den, t_orb, 0.00005 ) );
        }


        while ( not nav_data_queue_.empty() )  {
            double t_nav = nav_data_queue_.back().header.stamp.toSec();

            if ( t_nav > t_orb ) break;

            ardrone_autonomy::Navdata nav_msg = nav_data_queue_.back();
            nav_data_queue_.pop_back();

            if ( first_nav_msg_ ) {
                first_nav_msg_ = false;
                nav_z_ = shared_ptr<LTI::SisoSystem> ( new LTI::SisoSystem ( num,den, t_nav, 0.00005 ) );
            }

            double alt = nav_msg.altd / 1000.0;
            nav_z_->timeStep ( t_nav, alt );
        }

        orb_z_->timeStep ( t_orb, orb_msg.pose.position.z );

        if (not first_nav_msg_ and not first_orb_msg_) {
            orb_signal_ = orb_z_->getOutput ( t_orb );
            nav_signal_ = nav_z_->getOutput ( t_orb );

            ScaleStruct s = ScaleStruct ( orb_signal_, nav_signal_, std_orb_, std_nav_ );
            if ( s.isInlier ( orb_tol_min_, orb_tol_max_, nav_tol_min_, nav_tol_max_ ) ) scale_vector_.push_back ( s );
        }
    }
}
