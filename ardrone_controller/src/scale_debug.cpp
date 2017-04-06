#include <queue>

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

    inline bool isInlier ( float orb_tol, float nav_tol ) {
        return navNorm_ > nav_tol && orbNorm_ > orb_tol;
    }

    inline bool operator < ( const ScaleStruct& comp ) const {
        return lambdaPointEstimate_ < comp.lambdaPointEstimate_;
    }
};
//

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
    bool first_msg_;
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
    geometry_msgs::PoseWithCovarianceStamped filt_pose_, latest_pose_;
    vector<ScaleStruct> scale_vector_;

    std_msgs::Float32MultiArray data_array_;


public:

    ScaleEstimator() :orb_signal_ ( 0 ),nav_signal_ (0),first_msg_ ( true ) {
        T_init_.setIdentity();
        T_curr_.setIdentity();
        T_correction_.setIdentity();
        filt_pose_.pose.pose.position.x = filt_pose_.pose.pose.position.y = filt_pose_.pose.pose.position.z = 0;
        filt_pose_.pose.pose.orientation.x = filt_pose_.pose.pose.orientation.y = filt_pose_.pose.pose.orientation.z =filt_pose_.pose.pose.orientation.w = 0;
    }

    double filterScale ( vector<ScaleStruct> scale_vctr, double ratio, double orb_tol, double nav_tol, double orb_noise, double nav_noise , int cut_off ) {

        vector<ScaleStruct> temp;
        for ( int i=0; i<scale_vctr.size(); i++ ) {

            if ( scale_vctr[i].isInlier ( orb_tol, nav_tol ) ) {
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

            if ( temp.size() < 5 || ( s.lambdaPointEstimate_ > median * ratio && s.lambdaPointEstimate_ < median / ratio )) {

                S_yy_z += s.nav_z_ * s.nav_z_;
                S_xy_z += s.nav_z_ * s.orb_z_;
                S_xx_z += s.orb_z_ * s.orb_z_;

                counter++;
            }
        }
        
        if ( counter > cut_off ) fixed_scale_ = true;

        return ScaleStruct::computeEstimator ( S_xx_z, S_yy_z, S_xy_z, orb_noise, nav_noise );
    }

    void plotBatchScales ( vector<float> orb_tol, vector<float> nav_tol, vector<float> ratios, vector<float> variances ) {
        //debug
        int counter = 0;
				
        std_msgs::Float32MultiArray data_array;
				float x = latest_pose_.pose.pose.position.x;

        for ( int r = 0; r < ratios.size(); r++ ) {
						for ( int v = 0; v < variances.size(); v++ ) {
								for ( int o =0; o< orb_tol.size(); o++ ) {
										for ( int n=0; n< nav_tol.size(); n++ ) {

												double scale = filterScale ( scale_vector_, ratios[r], orb_tol[o], nav_tol[n], nav_noise_ * variances[v], nav_noise_ , 1e6 );

												float t = fmod ( ros::Time::now().toSec(),1000 );

												counter++;
												data_array.data.push_back ( scale );
												cout << " s: " << scale << " o: " << orb_tol[o] << "n: " << nav_tol[n] <<" t: " << t << " r: " << ratios[r] << " v: " << variances[v]  ;
												cout << " x: " << x / scale << " id: " << counter << endl;
										}
								}
						}
        }
        cout << endl << "====================================================" << endl;
        data_array_ = data_array;
    }

    void estimateScale() {//TODO ugly code
        vector<float> ratios = {0.2/*, 0.4,0.6, /*0.8, 0.9*/};// orb_noise / nav_noise
        vector<float> variances = {0.1, .2, 0.3, .4, 0.5, .6, 0.7, .8, 0.9, 1};
        vector<float> orb_tol = {0.1, 0.01, 0.001, 0.0001, 0};
        vector<float> nav_tol = {0.1, 0.01, 0.001, 0.0001, 0};

        plotBatchScales ( orb_tol, nav_tol, ratios, variances );
        
// 			if (hasFixedScale()) return;
// 			else {scale_ = filterScale ( scale_vector_, ratios, angles, orb_tol, nav_tol, nav_noise_ * variance, nav_noise_ , 1e6 );

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

        //Initialize nav datatype and orb message datatype to add sensor data to fution routine
        tf::StampedTransform nav_tf;
        double t = orb_data_queue_.back().header.stamp.toSec();
        poseMsgStamped orb_msg;

        while ( orb_data_queue_.size() >0 ) {
            //get oldest message
            orb_msg = orb_data_queue_.back();
            orb_data_queue_.pop_back();
            t = orb_msg.header.stamp.toSec();

            if ( first_msg_ ) { //initialize filters
                first_msg_ = false;

                double pole2Hz = 100.0*3.14*0.5;
                double pole5Hz = 100.0*3.14*0.5;

                //numerator and denominator of transfer function
                LTI::array num ( 2 ),den ( 3 );
                num[0]=0;
                num[1]=pole2Hz*pole5Hz;
                den[0]=pole2Hz*pole5Hz;
                den[1]=pole2Hz+pole5Hz;
                den[2]=1;
								
                orb_z_ = std::shared_ptr<LTI::SisoSystem> ( new LTI::SisoSystem ( num,den, t, 0.005 ) );
                nav_z_ = std::shared_ptr<LTI::SisoSystem> ( new LTI::SisoSystem ( num,den, t, 0.005 ) );

            }
            orb_z_->timeStep ( t,orb_msg.pose.pose.position.z );

            if ( tf_listener_.waitForTransform ( "/odom", "/ardrone_base_link_corrected",orb_msg.header.stamp,
                                                 ros::Duration ( 1.0 ),ros::Duration ( 0.1 ) ) ) {
                try {
                    tf_listener_.lookupTransform ( "odom", "/ardrone_base_link_corrected", orb_msg.header.stamp, nav_tf );
                    nav_z_->timeStep ( t,nav_tf.getOrigin().getZ() );

                } catch ( tf2::ExtrapolationException err ) {
                    cout << err.what() << endl;
                    return;
                }
            }
        }

        orb_signal_ = orb_z_->getOutput ( t );
        nav_signal_ = nav_z_->getOutput ( t );

        ScaleStruct s = ScaleStruct ( orb_signal_, nav_signal_, orb_noise_, nav_noise_ );

        scale_vector_.push_back ( s );
        estimateScale();
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

    void navCallback ( ardrone_autonomy::Navdata navdata ) {
        // get current frame
        if ( tf_listener_.waitForTransform ( "/odom", "/ardrone_base_link",navdata.header.stamp,ros::Duration ( 1.0 ),ros::Duration ( 0.1 ) ) ) {
						try {tf_listener_.lookupTransform ( "odom", "/ardrone_base_link", navdata.header.stamp, T_curr_ );} 
						catch ( tf2::ExtrapolationException e ) {cout << e.what() << endl;}
        }
        
        if (correction_init_) {
						correction_init_ = true;
						T_init_ = T_curr_;
				}
				
        // calculate correction transformation
        if ( not correction_made_ && navdata.altd ) {
						correction_made_ = true;	
						T_correction_ = T_curr_.inverse() * T_init_;
        }

        T_init_ = T_curr_;
				
        // publish corrected frame
        br_.sendTransform ( tf::StampedTransform ( T_correction_, navdata.header.stamp, "/ardrone_base_link", "/ardrone_base_link_corrected" ) );
    }

    bool hasFixedScale() {return fixed_scale_;}

    float getScale() {return scale_;}

    std_msgs::Float32MultiArray getData() {return data_array_;}
};


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "scale_estimator" );
    ros::NodeHandle nh;

    // estimates scale
    ScaleEstimator scale_est;

    // subscribe to orb pose and accelerometer data from nav
    ros::Subscriber orb_sub = nh.subscribe ( "/orb/pose_unscaled", 10, &ScaleEstimator::orbCallback, &scale_est );

    // get rotation due to magnetic field
    ros::Subscriber nav_sub = nh.subscribe ( "/ardrone/navdata", 10, &ScaleEstimator::navCallback, &scale_est );

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

