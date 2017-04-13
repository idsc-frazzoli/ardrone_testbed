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
        } else {
            std_msgs::Float32 scale_for_publish;
            scale_for_publish.data = scale_est.getScale();
            scale_pub.publish ( scale_for_publish );
        }

        loop_rate.sleep();
    }

    return 0;
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 

