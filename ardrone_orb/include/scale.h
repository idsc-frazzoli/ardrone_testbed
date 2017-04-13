/*
 * (c) 2017
 *
 * Authors:
 * Daniel Gehrig (ETH Zurich)
 * Maximilian Goettgens (ETH Zurich)
 * Brian Paden (MIT)
 * 
 * This file was originally written as part of the PTAM Wrapper by J. Engel, J. Sturm, 
 * D. Cremers and has been modified by the authors to serve several tasks 
 * needed for the ardrone to properly use the ORB SLAM 2 algorithm for state 
 * estimation.
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
#ifndef ScaleStruct_H
#define ScaleStruct_H

#include <math.h>
#include <iostream>

using namespace std;

class ScaleStruct
{
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
        bool is_nav_inlier = fabs ( nav_z_ ) > nav_tol && fabs ( nav_z_ ) < nav_max_tol;
        bool is_orb_inlier = fabs ( orb_z_ ) > orb_tol && fabs ( orb_z_ ) < orb_max_tol;
        return   is_nav_inlier && is_orb_inlier;
    }

    inline bool operator < ( const ScaleStruct& comp ) const {
        return scale_ < comp.scale_;
    }
};

#endif
