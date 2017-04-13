/*
 * Copyright 2017 ...
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


#include <queue>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ardrone_autonomy/Navdata.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath//Vector3.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <linear_system.h>

using namespace std;
typedef geometry_msgs::PoseWithCovarianceStamped poseMsgStamped;
typedef geometry_msgs::Pose poseMsg;

// Adapted from tum
class ScaleStruct {
public:
    double orb_z_;
    double nav_z_;
    double realDisplacement_;
    double orbNorm_;
    double navNorm_;
    double lambdaPointEstimate_;
    double s_xx_, s_yy_, s_xy_;
    double log_likelihood_;
    double angle_; // in deg
    double orb_noise_;
    double nav_noise_;
    
    // inlier parameters
    float dot_prod_tol_ = 0.05;
    
    static inline double computeEstimator ( double s_xx, 
                                            double s_yy, 
                                            double s_xy, 
                                            double slam_std_dev = 0.2, 
                                            double acoustic_sensor_std_dev = 0.1 ) {
        double sII = slam_std_dev * slam_std_dev * s_yy;
        double sPP = acoustic_sensor_std_dev * acoustic_sensor_std_dev * s_xx;
        double sPI = acoustic_sensor_std_dev * slam_std_dev * s_xy;
        
        int sgn = copysign ( 1, s_xy );
        
        double tmp = ( sII-sPP ) * ( sII-sPP ) + 4*sPI*sPI;
        if ( tmp <= 0 ) {tmp = 1e-5;} // numeric issues
        return 0.5* ( - ( sII-sPP ) +sgn*sqrt ( tmp ) ) / ( acoustic_sensor_std_dev * acoustic_sensor_std_dev * s_xy );
        
    }
    
    //Equation (6) in TUM paper: generates most likely position given the two measurements
    static inline double computeRealDisplacement ( double orb, 
                                                   double nav, 
                                                   double lambda, 
                                                   double slam_std_dev = 0.01, 
                                                   double acoustic_sensor_std_dev = 0.2 ) {
        
        double d = lambda * lambda * acoustic_sensor_std_dev * acoustic_sensor_std_dev + slam_std_dev * slam_std_dev;
        double f1 = lambda * acoustic_sensor_std_dev * acoustic_sensor_std_dev / d;
        double f2 = slam_std_dev * slam_std_dev / d;
        
        return orb * f1 + nav * f2;
    }
    
    static inline double computeLogLikelihood ( double orb, 
                                                double nav, 
                                                double realDisplacement, 
                                                double lambda, 
                                                double slam_std_dev, 
                                                double acoustic_sensor_std_dev ) {
        
        double delta_p = orb - realDisplacement * lambda;
        double delta_i = nav - realDisplacement;
        
        return delta_p / ( slam_std_dev * slam_std_dev ) + delta_i / ( acoustic_sensor_std_dev * acoustic_sensor_std_dev );
    }
    
    inline ScaleStruct ( double orb_z, 
                         double nav_z, 
                         double slam_std_dev, 
                         double acoustic_sensor_std_dev ) {
        orb_z_ = orb_z;
        nav_z_ = nav_z;
        
        orb_noise_ = slam_std_dev;
        nav_noise_ = acoustic_sensor_std_dev;
        
        lambdaPointEstimate_ = computeEstimator ( orb_z_*orb_z_, 
                                                  nav_z_*nav_z_, 
                                                  nav_z_*orb_z_, 
                                                  orb_noise_, 
                                                  nav_noise_ );
        
        realDisplacement_ = computeRealDisplacement ( orb_z_, 
                                                      nav_z_, 
                                                      lambdaPointEstimate_, 
                                                      orb_noise_, nav_noise_ );
        log_likelihood_ = computeLogLikelihood ( orb_z_, 
                                                 nav_z_, 
                                                 realDisplacement_, 
                                                 lambdaPointEstimate_, 
                                                 orb_noise_, 
                                                 nav_noise_ );
    }
    
    //Data-point acceptance criteria
    inline bool isInlier ( float orb_tol, float orb_max_tol, float nav_tol ) {
        return abs ( nav_z_ ) > nav_tol && abs ( orb_z_ ) > orb_tol && abs ( orb_z_ ) < orb_max_tol && abs ( nav_z_ ) < .5;
    }
    
    //Orders scale estimates on single data-points for median filter
    inline bool operator < ( const ScaleStruct& comp ) const {
        return lambdaPointEstimate_ < comp.lambdaPointEstimate_;
    }
};

class ScaleEstimator {
    std::shared_ptr<LTI::SisoSystem> nav_x_, nav_y_, nav_z_;
    std::shared_ptr<LTI::SisoSystem> orb_x_,orb_y_,orb_z_;
    bool fixed_scale_ = false;
    float scale_ = 1; // has units m^-1
    float scale_ubound_ = 1;
    float scale_lbound_ = 1;
    
    // nav data correction
    bool correction_init_ = false;
    bool correction_made_ = false;
    tf::StampedTransform T_init_, T_curr_, T_prev_;
    tf::Transform T_correction_;
    tf::TransformBroadcaster br_;
    
    // tuning parameters
    const float orb_noise_=0.2, nav_noise_=0.1; // noise params must be tuned (init vals from tum)
    const float dot_prod_tol_ = 0.05;
    const float ratio_ = 0.2;
    const int scale_samples_ = 10;
    double orb_signal_;
    double nav_signal_;
    tf::Vector3 init_displacement_ = tf::Vector3 ( 0,0,0 );
    tf::TransformListener tf_listener_;
    deque<geometry_msgs::PoseWithCovarianceStamped> orb_data_queue_;
    deque<ardrone_autonomy::Navdata> nav_data_queue_;
    geometry_msgs::PoseWithCovarianceStamped filt_pose_, latest_pose_;
    vector<ScaleStruct> scale_vector_;
    
    std_msgs::Float32MultiArray data_array_;
    
    
    // recursive least squares
    
    double Theta_lower_bound_inverse_ = 15;
    double Theta_upper_bound_ = .2;
    double P_lower_bound_ = 1;
    double P_upper_bound_ = 1;
    
    double f = 1.0;
    double pole_location = 2*M_PI*f;
    
    vector<double> x_stream_, y_stream_, time_x_, time_y_;
    int scores[2500];
    string scores_path_="scores_01_1";
    float true_scale_ = 0.1104;
    
    int max_counter_ = 300;
    
    
    
public:
    
    ScaleEstimator() :orb_signal_ ( 0 ),nav_signal_ ( 0 ) {
        T_init_.setIdentity();
        T_curr_.setIdentity();
        T_correction_.setIdentity();
        filt_pose_.pose.pose.position.x = filt_pose_.pose.pose.position.y = filt_pose_.pose.pose.position.z = 0;
        filt_pose_.pose.pose.orientation.x = filt_pose_.pose.pose.orientation.y = filt_pose_.pose.pose.orientation.z =filt_pose_.pose.pose.orientation.w = 0;
        for (int i=0; i<2500; i++){
            scores[i] = 0;
        }
    }
    
    double filterScale (vector<ScaleStruct>& scale_vctr, double ratio, double orb_tol, double orb_max_tol, double nav_tol, double orb_noise, double nav_noise , int cut_off ) {
        
        vector<ScaleStruct> temp;
        for ( int i=0; i<scale_vctr.size(); i++ ) {
            
            if ( scale_vctr[i].isInlier ( orb_tol, orb_max_tol, nav_tol ) ) {
                temp.push_back ( scale_vctr[i] );
            }
        }
        
        sort ( scale_vctr.begin(), scale_vctr.end() );
        int counter = 0;
        
        double median = ( scale_vctr.size() < 5 ) ? 1 : scale_vctr[ ( scale_vctr.size() +1 ) /2].lambdaPointEstimate_;
        
        // find sums and median.
        double S_yy_z(0), S_xx_z(0), S_xy_z(0);
        
        for ( unsigned int i=0; i<temp.size(); i++ ) {
            
            ScaleStruct s = temp[i];
            
            if ( temp.size() < 5 || ( s.lambdaPointEstimate_ > median * ratio && s.lambdaPointEstimate_ < median / ratio ) ) {
                S_yy_z += s.nav_z_ * s.nav_z_;
                S_xy_z += s.nav_z_ * s.orb_z_;
                S_xx_z += s.orb_z_ * s.orb_z_;
                
                counter++;
            }
        }
        
        if ( counter > cut_off ) fixed_scale_ = true;
        
        return ScaleStruct::computeEstimator ( S_xx_z, S_yy_z, S_xy_z, orb_noise, nav_noise );
    }
    
    void estimateScale() {//TODO ugly code
        
        //o: 0.05 n: 0.01 r: 0.5 v: 0.7 x: 1.59891 y: -0.609929 z: 0.580346 id: 49
        
        
        vector<float> ratios = {/*0.0001, */0.1/*, 0.2, 0.4,0.6, 0.9*/};
        vector<float> variances = {/*0.1, 0.2, 0.3,0.4, 0.5, 1,*/ 2/*, 5, 10*/}; // orb_noise / nav_noise
        vector<float> orb_tol = { 0/*, 0.005, 0.01*/};
        vector<float> nav_tol = { /*0, */0.005/*, 0.01*/};
        vector<float> orb_max_tol = {/* .5, .4, .3, .2,*/.1};
        
        //plotBatchScales ( orb_tol, orb_max_tol, nav_tol, ratios, variances,  true_scale_);
        if (hasFixedScale()) return;
        else {scale_ = filterScale ( scale_vector_, ratios[0], orb_tol[0], orb_max_tol[0], nav_tol[0], nav_noise_ * variances[0], nav_noise_ , max_counter_ );}
        
    }
    
    void processQueue() {
        //Only process orb data if batch size is greater than 100
        if ( orb_data_queue_.size() <1 ) {return;}
        
        //numerator and denominator of pre-filter transfer function
        LTI::array num_orb ( 2 ),den_orb ( 3 );
        num_orb[0]=0;
        num_orb[1]=sqrt(pow(pow(pole_location,2.0)-1.0,2.0)+4.0*pow(pole_location,2));
        
        den_orb[0]=pole_location * pole_location;
        den_orb[1]=2.0*pole_location;
        den_orb[2]=1;
        
        //numerator and denominator of transfer function
        LTI::array num_nav ( 2 ),den_nav ( 3 );
        num_nav[0]=0;
        num_nav[1]=sqrt(pow(pow(pole_location,2.0)-1.0,2.0)+4.0*pow(pole_location,2));
        
        den_nav[0]=pow(pole_location,2);
        den_nav[1]=2.0*pole_location;
        den_nav[2]=1;
        
        while ( not orb_data_queue_.empty() ) {
            
            // get first orb msg
            geometry_msgs::PoseWithCovarianceStamped orb_msg = orb_data_queue_.back();
            orb_data_queue_.pop_back();
            
            if ( orb_z_.get()==nullptr ) {
                orb_z_ = std::shared_ptr<LTI::SisoSystem> ( new LTI::SisoSystem ( num_orb,den_orb, orb_msg.header.stamp.toSec(), 0.00005 ) );
            }
            
            double t = orb_msg.header.stamp.toSec();
            time_x_.push_back(t);
            x_stream_.push_back(orb_msg.pose.pose.position.z);
            
            while ( not nav_data_queue_.empty() )  {
                
                if ( nav_data_queue_.back().header.stamp.toSec() > t ) break;
                
                ardrone_autonomy::Navdata nav_msg = nav_data_queue_.back();
                nav_data_queue_.pop_back();
                
                if ( nav_z_.get()==nullptr) {
                    nav_z_ = std::shared_ptr<LTI::SisoSystem> ( new LTI::SisoSystem ( num_nav,den_nav, nav_msg.header.stamp.toSec(), 0.00005 ) );
                }
                
                time_y_.push_back(nav_msg.header.stamp.toSec());
                y_stream_.push_back(double(nav_msg.altd)/1000);
                
                
                nav_z_->timeStep ( nav_msg.header.stamp.toSec(), double ( nav_msg.altd ) /1000.0 );
            }
            
            orb_z_->timeStep ( t, orb_msg.pose.pose.position.z );

            //Extrapolate orb and nav measurements at time t        
            orb_signal_ = orb_z_->getOutput ( t );
            nav_signal_ = nav_z_->getOutput ( t );

            ScaleStruct s = ScaleStruct ( orb_signal_, nav_signal_, orb_noise_, nav_noise_ );
            scale_vector_.push_back ( s );
            
            data_array_.data.clear();
            data_array_.data.push_back ( nav_signal_ );      
            estimateScale();
            
            updateLSScale ( orb_signal_, nav_signal_, Theta_upper_bound_, P_upper_bound_ );
            
            for (int i = 1; i < data_array_.data.size(); i++){
                data_array_.data[i] = orb_signal_ / data_array_.data[i];
            }
            data_array_.data.push_back(orb_signal_ / Theta_upper_bound_);
        }
    }
    
    //Simple recursive least squares
    void updateLSScale ( const double& y,const double& phi, double& theta, double& P ) {

        double K = P * phi / ( 1 + phi * P * phi );
        theta += K * ( y - phi * theta );
        P -= P * phi * K;
    }
    
    //Copies oldest message in orb msg buffer into internal pose variable
    poseMsgStamped getScaledOrbPose() {
        
        filt_pose_.header.stamp = latest_pose_.header.stamp;
        filt_pose_.header.frame_id = "odom";
        
        filt_pose_.pose.pose.orientation = latest_pose_.pose.pose.orientation;
        filt_pose_.pose.pose.position.x = latest_pose_.pose.pose.position.x /scale_;
        filt_pose_.pose.pose.position.y = latest_pose_.pose.pose.position.y /scale_;
        filt_pose_.pose.pose.position.z = latest_pose_.pose.pose.position.z /scale_;
        
        return filt_pose_;
    }
    
    void orbCallback ( geometry_msgs::PoseWithCovarianceStamped msg ) {
        orb_data_queue_.push_front ( msg );
        latest_pose_ = msg;
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
    
    std_msgs::Float32MultiArray getData() {
        return data_array_;
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
    
    // publish filtered orb pose
    ros::Publisher scaled_orb_pub = nh.advertise<poseMsgStamped> ( "/ardrone/pose_scaled",2 );
    
    // publish scale
    ros::Publisher scale_pub = nh.advertise<std_msgs::Float32> ( "/scale_estimator/scale", 1 );
    
    // publish data
    ros::Publisher data_pub = nh.advertise<std_msgs::Float32MultiArray> ( "/scale_estimator/data", 10 );
    
    int rate = 50;
    ros::Rate loop_rate ( rate );
    
    while ( ros::ok() ) {
        
        ros::spinOnce();
        
        scale_est.processQueue();//doesn't do anything if queue size less than 50
        
        data_pub.publish ( scale_est.getData() );
        
        poseMsgStamped scale_pose_for_publish = scale_est.getScaledOrbPose();
        std_msgs::Float32 scale_for_publish;
        scale_for_publish.data = scale_est.getScale();
        
        scale_pub.publish ( scale_for_publish );
        scaled_orb_pub.publish ( scale_pose_for_publish );
        
        loop_rate.sleep();
    }
    
    return 0;
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 

