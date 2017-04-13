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


#include <queue>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>

#include <ardrone_autonomy/Navdata.h>

#include <linear_system.h>

using namespace std;
typedef geometry_msgs::PoseStamped poseMsgStamped;
typedef geometry_msgs::Pose poseMsg;

// directly copied from tum
class ScaleStruct {
public:
    double orb_z_;
    double nav_z_;
    double scale_;
    double std_orb_;
    double std_nav_;

    static inline double computeEstimator ( double s_xx, double s_yy, double s_xy, double std_orb, double std_nav ) {
        double sII = std_orb * std_orb * s_yy;
        double sPP = std_nav * std_nav * s_xx;
        double sPI = std_nav * std_orb * s_xy;

        int sgn = copysign ( 1, s_xy );

        double tmp = ( sII-sPP ) * ( sII-sPP ) + 4*sPI*sPI;
        if ( tmp <= 0 ) {
            tmp = 1e-5; // numeric issues
        }
        return 0.5* ( - ( sII-sPP ) +sgn*sqrt ( tmp ) ) / ( std_nav * std_nav * s_xy );
    }

    inline ScaleStruct ( double orb_z, double nav_z, double std_orb, double std_nav ) {
        orb_z_ = orb_z;
        nav_z_ = nav_z;

        std_orb_ = std_orb;
        std_nav_ = std_nav;

        scale_ = computeEstimator ( orb_z_*orb_z_, nav_z_*nav_z_, nav_z_*orb_z_, std_orb, std_nav );
    }

    inline bool isInlier ( float orb_tol, float orb_max_tol, float nav_tol, float nav_max_tol ) {
        bool is_nav_inlier = abs ( nav_z_ ) > nav_tol && abs ( nav_z_ ) < nav_max_tol;
        bool is_orb_inlier = abs ( orb_z_ ) > orb_tol && abs ( orb_z_ ) < orb_max_tol;
        return   is_nav_inlier && is_orb_inlier;
    }

    inline bool operator < ( const ScaleStruct& comp ) const {
        return scale_ < comp.scale_;
    }
};


class ScaleEstimator {

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

    double filterScale ( vector<ScaleStruct> scale_vctr, double ratio, double std_orb, double std_nav , int cut_off ) {

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

    void estimateScale() {
        scale_ = filterScale ( scale_vector_, ransac_tolerance_, std_orb_, std_nav_ , max_counter_ );
    }

    void processQueue() {
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

    void orbCallback ( geometry_msgs::PoseStamped msg ) {
        orb_data_queue_.push_front ( msg );
    }

    void navCallback ( ardrone_autonomy::Navdata msg ) {
        nav_data_queue_.push_front ( msg );
    }

    bool hasFixedScale() {
        return fixed_scale_;
    }

    float getScale() {
        return scale_;
    }
    
};

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "scale_estimator" );
    ros::NodeHandle nh;

    // estimates scale
    ScaleEstimator scale_est;

    // subscribe to orb pose and accelerometer data from nav
    ros::Subscriber orb_sub = nh.subscribe ( "/orb/pose_unscaled", 10, &ScaleEstimator::orbCallback, &scale_est );

    // subscribe to orb pose and accelerometer data from nav
    ros::Subscriber nav_sub = nh.subscribe ( "/ardrone/navdata", 30, &ScaleEstimator::navCallback, &scale_est );

    // publish scale
    ros::Publisher scale_pub = nh.advertise<std_msgs::Float32> ( "/scale_estimator/scale", 1 );

    int rate = 50;
    ros::Rate loop_rate ( rate );

    string info = "Beginning scale estimation at " + to_string ( rate ) + " Hz";
    ROS_INFO ( info.c_str() );

    while ( ros::ok() ) {

        ros::spinOnce();

        if ( not scale_est.hasFixedScale() ) {
            scale_est.processQueue();//doesn't do anything if queue size less than 50
            scale_est.estimateScale();
        }

        std_msgs::Float32 scale_for_publish;
        scale_for_publish.data = scale_est.getScale();

        scale_pub.publish ( scale_for_publish );

        loop_rate.sleep();
    }

    return 0;
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 

