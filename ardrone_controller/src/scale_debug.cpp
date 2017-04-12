// dh: authorship
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

// directly copied from tum
// dh: credit more accurately and move to separate file
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

    static inline double computeEstimator ( double s_xx, double s_yy, double s_xy, double stdDevPTAM = 0.2, double stdDevIMU = 0.1 ) {
        double sII = stdDevPTAM * stdDevPTAM * s_yy;
        double sPP = stdDevIMU * stdDevIMU * s_xx;
        double sPI = stdDevIMU * stdDevPTAM * s_xy;

        int sgn = copysign ( 1, s_xy );

        double tmp = ( sII-sPP ) * ( sII-sPP ) + 4*sPI*sPI;
        if ( tmp <= 0 ) {
            tmp = 1e-5; // numeric issues
        }
        return 0.5* ( - ( sII-sPP ) +sgn*sqrt ( tmp ) ) / ( stdDevIMU * stdDevIMU * s_xy );

    }

    static inline double computeRealDisplacement ( double orb, double nav, double alpha, double stdDevPTAM = 0.01, double stdDevIMU = 0.2 ) {
        double d = alpha * alpha * stdDevIMU * stdDevIMU + stdDevPTAM * stdDevPTAM;
        double f1 = alpha * stdDevIMU * stdDevIMU / d;
        double f2 = stdDevPTAM * stdDevPTAM / d;

        return orb * f1 + nav * f2;
    }

    static inline double computeLogLikelihood ( double orb, double nav, double realDisplacement, double alpha, double stdDevPTAM, double stdDevImu ) {
        double delta_p = orb - realDisplacement * alpha;
        double delta_i = nav - realDisplacement;

        return delta_p / ( stdDevPTAM * stdDevPTAM ) + delta_i / ( stdDevImu * stdDevImu );
    }

    inline ScaleStruct ( double orb_z, double nav_z, double stdDevOrb, double stdDevNav ) {
        orb_z_ = orb_z;
        nav_z_ = nav_z;

        orb_noise_ = stdDevOrb;
        nav_noise_ = stdDevNav;

        lambdaPointEstimate_ = computeEstimator ( orb_z_*orb_z_, nav_z_*nav_z_, nav_z_*orb_z_, orb_noise_, nav_noise_ );
        realDisplacement_ = computeRealDisplacement ( orb_z_, nav_z_, lambdaPointEstimate_, orb_noise_, nav_noise_ );
        log_likelihood_ = computeLogLikelihood ( orb_z_, nav_z_, realDisplacement_, lambdaPointEstimate_, orb_noise_, nav_noise_ );
    }

    inline bool isInlier ( float orb_tol, float orb_max_tol, float nav_tol ) {
        return abs ( nav_z_ ) > nav_tol && abs ( orb_z_ ) > orb_tol && abs ( orb_z_ ) < orb_max_tol && abs ( nav_z_ ) < .5;
    }

    inline bool operator < ( const ScaleStruct& comp ) const {
        return lambdaPointEstimate_ < comp.lambdaPointEstimate_;
    }
};
//

// dh: utility functions to separate file(s)
void printVector ( const std::string& str, const tf::Vector3& vctr ) {
    std::cout << str << ": (" << vctr.x() << "," << vctr.y() << "," << vctr.z() << ")" << std::endl;
}

void printVector ( const std::string& str, const geometry_msgs::Pose& vctr ) {
    std::cout << str << ": (" << vctr.position.x << "," << vctr.position.y << "," << vctr.position.z << ")" << std::endl;
}

void printVector ( const std::string& str, const geometry_msgs::PoseWithCovarianceStamped& vctr ) {
    std::cout << str << ": (" << vctr.pose.pose.position.x << "," << vctr.pose.pose.position.y << "," << vctr.pose.pose.position.z << ")" << std::endl;
}

void printVector ( const std::string& str, const tf::StampedTransform& vctr ) {
    std::cout << str << ": (" << vctr.getOrigin().getX() << "," << vctr.getOrigin().getY() << "," << vctr.getOrigin().getZ() << ")" << std::endl;
}

void printScaleStruct ( const std::string& str, const ScaleStruct& s ) {
    std::cout << str << endl;

    std::cout << "scale: " << s.lambdaPointEstimate_ << "\t";
    std::cout << "|x|: " << s.orbNorm_ << "\t";
    std::cout << "|y|: " << s.navNorm_ << "\t";
    std::cout << "l*|y|: " << s.lambdaPointEstimate_ * s.navNorm_ << endl;
    std::cout << "logL: " << s.log_likelihood_ << "\t" << "angle: " << s.angle_ << endl;
}

class ScaleEstimator {
    std::shared_ptr<LTI::SisoSystem> nav_x_, nav_y_, nav_z_;
    std::shared_ptr<LTI::SisoSystem> orb_x_,orb_y_,orb_z_;
    bool first_orb_msg_, first_nav_msg_;
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
    
    double f = 30;
    double left_pole_orb = f*3.14*2;
    double right_pole_orb = f*3.14*2;
    double left_pole_nav = f*3.14*2;
    double right_pole_nav = f*3.14*2;
    
    vector<double> x_stream_, y_stream_, time_x_, time_y_;
// dh: magic const appears in more than one location -> define value once and reuse
// dh: is scores still needed?
    int scores[2500];
    string scores_path_="scores_01_1";
    float true_scale_ = 0.1104;
    
    int max_counter_ = 300;
    


public:

    ScaleEstimator() :orb_signal_ ( 0 ),nav_signal_ ( 0 ),first_orb_msg_ ( true ), first_nav_msg_ ( true ) {
        T_init_.setIdentity();
        T_curr_.setIdentity();
        T_correction_.setIdentity();
        filt_pose_.pose.pose.position.x = filt_pose_.pose.pose.position.y = filt_pose_.pose.pose.position.z = 0;
        filt_pose_.pose.pose.orientation.x = filt_pose_.pose.pose.orientation.y = filt_pose_.pose.pose.orientation.z =filt_pose_.pose.pose.orientation.w = 0;
        for (int i=0; i<2500; i++)
        {
            scores[i] = 0;
        }
    }

    double filterScale ( vector<ScaleStruct> scale_vctr, double ratio, double orb_tol, double orb_max_tol, double nav_tol, double orb_noise, double nav_noise , int cut_off ) {

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
        // do separately for xy and z and xyz-all and xyz-filtered
        double S_yy_z, S_xx_z, S_xy_z;
        S_yy_z = S_xx_z = S_xy_z = 0;

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

    void plotBatchScales ( vector<float> orb_tol, vector<float> orb_max_tol, vector<float> nav_tol, vector<float> ratios, vector<float> variances, double true_scale ) {
        //debug
        int counter = 0;
        for (int om = 0; om<orb_max_tol.size(); om++) {
            for ( int r = 0; r < ratios.size(); r++ ) {
                for ( int v = 0; v < variances.size(); v++ ) {
                    for ( int o =0; o< orb_tol.size(); o++ ) {
                        for ( int n=0; n< nav_tol.size(); n++ ) {

                            double scale = filterScale ( scale_vector_, ratios[r], orb_tol[o], orb_max_tol[om], nav_tol[n], nav_noise_ * variances[v], nav_noise_ , max_counter_ );    
                            
                            data_array_.data.push_back(scale);

//                             cout << " s: " << scale << " r: " << ratios[r] << " v: " << variances[v] << " o: " << orb_tol[o] << " n: " << nav_tol[n] << " id: "<< counter ;
                            double err = abs(true_scale - scale)/true_scale;
                            
//                             if ( err < 0.05) {
//                                 scores[counter] +=5;
//                                 cout << " CONVERGED WITHIN 5%" << endl;
//                             } else if (err < 0.1) {
//                                 scores[counter] +=2;
//                                 cout << " CONVERGED WITHIN 10%" << endl;
//                             } else if (err < 0.2) {
//                                 scores[counter] +=1;
//                                 cout << " CONVERGED WITHIN 20%" << endl;
//                             } else {
//                                 cout << endl;
//                             }
                            
                            counter++;
                        }
                    }
                }
            }
        }
        for (int i=0; i<2500; i++) {
            //cout << i << " : " <<scores[i] << " ";
        }
//         cout << endl << "====================================================" << endl;
        //data_array_ = data_array;
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

    void printAll() {
        //This gets called at every iteration of the ros loop
        float cur_orb_z = latest_pose_.pose.pose.position.x;
        float cur_orb_y = latest_pose_.pose.pose.position.y;
        float cur_orb_x = latest_pose_.pose.pose.position.z;
     
        tf::Vector3 orb_pos = tf::Vector3 ( latest_pose_.pose.pose.position.x, latest_pose_.pose.pose.position.y, latest_pose_.pose.pose.position.z );
        tf::Vector3 pos = orb_pos * 1 / scale_;

        printVector ( "position unscaled", orb_pos );
        printVector ( "position", pos );

        cout <<"s: " << scale_ << endl;
    }

    void processQueue() {
        //Only process orb data if batch size is greater than 100
        if ( orb_data_queue_.size() <1 ) {
            return;
        }

        //numerator and denominator of transfer function
// dh: init of num and den is redundant -> extract to function and reuse
        LTI::array num_orb ( 2 ),den_orb ( 3 );
        num_orb[0]=0;
        num_orb[1]=left_pole_orb * right_pole_orb;
        
        den_orb[0]=left_pole_orb * right_pole_orb;
        den_orb[1]=left_pole_orb + right_pole_orb;
        den_orb[2]=1;
        

        //numerator and denominator of transfer function
        LTI::array num_nav ( 2 ),den_nav ( 3 );
        num_nav[0]=0;
        num_nav[1]=left_pole_nav * right_pole_nav;
        
        den_nav[0]=left_pole_nav * right_pole_nav;
        den_nav[1]=left_pole_nav + right_pole_nav;
        den_nav[2]=1;
        
        while ( not orb_data_queue_.empty() ) {

            // get first orb msg
            geometry_msgs::PoseWithCovarianceStamped orb_msg = orb_data_queue_.back();
            orb_data_queue_.pop_back();

            if ( first_orb_msg_ ) {
                first_orb_msg_ = false;

                orb_z_ = std::shared_ptr<LTI::SisoSystem> ( new LTI::SisoSystem ( num_orb,den_orb, orb_msg.header.stamp.toSec(), 0.00005 ) );
            }

            double t = orb_msg.header.stamp.toSec();
            time_x_.push_back(t);
            x_stream_.push_back(orb_msg.pose.pose.position.z);

            while ( not nav_data_queue_.empty() )  {

                if ( nav_data_queue_.back().header.stamp.toSec() > t ) break;

                ardrone_autonomy::Navdata nav_msg = nav_data_queue_.back();
                nav_data_queue_.pop_back();

                if ( first_nav_msg_ ) {
                    first_nav_msg_ = false;
                    nav_z_ = std::shared_ptr<LTI::SisoSystem> ( new LTI::SisoSystem ( num_nav,den_nav, nav_msg.header.stamp.toSec(), 0.00005 ) );
                }
                
                time_y_.push_back(nav_msg.header.stamp.toSec());
                y_stream_.push_back(double(nav_msg.altd)/1000);


                nav_z_->timeStep ( nav_msg.header.stamp.toSec(), double ( nav_msg.altd ) /1000.0 );
            }

            orb_z_->timeStep ( t, orb_msg.pose.pose.position.z );

            orb_signal_ = orb_z_->getOutput ( t );
// dh: ensure that nav_z_ is initialized
            nav_signal_ = nav_z_->getOutput ( t );
            
            
            
            
//             cout << "orb: " << orb_signal_ << " nav: " << nav_signal_ << endl;

            ScaleStruct s = ScaleStruct ( orb_signal_, nav_signal_, orb_noise_, nav_noise_ );
// dh: growth of scale_vector_ is unbounded
            scale_vector_.push_back ( s );
            
            data_array_.data.clear();
            data_array_.data.push_back ( nav_signal_ );      
            estimateScale();

            updateLSScale ( orb_signal_, nav_signal_, Theta_upper_bound_, P_upper_bound_ );
            
            for (int i = 1; i < data_array_.data.size(); i++)
            {
                data_array_.data[i] = orb_signal_ / data_array_.data[i];
            }
            data_array_.data.push_back(orb_signal_ / Theta_upper_bound_);
            //cout << "LS: " << Theta_upper_bound_ << endl;
            
            
        }
    }

// dh: method can be static
    void updateLSScale ( const double& y,const double& phi, double& theta, double& P ) {
        double K = P * phi / ( 1 + phi * P * phi );

        theta += K * ( y - phi * theta );

        P -= P * phi * K;
    }

    //Copies oldest message in orb msg buffer into internal pose variable
    poseMsgStamped getScaledOrbPose() {
// dh: better style to make filt_pose_ a local variable in this function
        filt_pose_.header.stamp = latest_pose_.header.stamp;
        filt_pose_.header.frame_id = "odom";

        filt_pose_.pose.pose.orientation = latest_pose_.pose.pose.orientation;
        filt_pose_.pose.pose.position.x = latest_pose_.pose.pose.position.x /scale_;
        filt_pose_.pose.pose.position.y = latest_pose_.pose.pose.position.y /scale_;
        filt_pose_.pose.pose.position.z = latest_pose_.pose.pose.position.z /scale_;

        return filt_pose_;

    }

    void orbCallback ( geometry_msgs::PoseWithCovarianceStamped msg ) {
//             cout << "ORB SEQ: " << msg.header.seq << endl;
        orb_data_queue_.push_front ( msg );
        latest_pose_ = msg;
    }

    void navCallback ( ardrone_autonomy::Navdata msg ) {

//             cout << "NAV SEQ: " << msg.header.seq << endl;
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

    void errorToFile ( const std::string& name_x, const std::string& name_y,  const std::string& path) {
        ofstream x;
        x.open ( path+name_x );
        for ( int i=0; i<time_x_.size(); i++ ) {
            x << to_string(x_stream_[i]) << "," << to_string(time_x_[i]) << std::endl;
        }
        x.close();
        
        ofstream y;
        y.open ( path+name_y );
        for ( int i=0; i<time_y_.size(); i++ ) {
            y << to_string(y_stream_[i]) << "," << to_string(time_y_[i]) << std::endl;
        }
        y.close(); 
    }
    
    void scoresToFile (const std::string& path) {
        ofstream f;
        f.open ( path + scores_path_ );
// dh: magic const 2500
        for ( int i=0; i<2500; i++ ) {
            f << scores[i] << "," << endl;
        }
        f.close();
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

    //scale_est.errorToFile("x_stream_bag_2", "y_stream_bag_2", "./../../../data/" );
//     scale_est.scoresToFile("./../../../data/");
    
    return 0;
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 

