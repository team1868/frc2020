/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"
#include <stdio.h>
#include <math.h>

static const double RADIANS_TO_DEGREES_CONVERTER = 180 / PI;

class AutoMeasures{
    public: 
        AutoMeasures();
        ~AutoMeasures();
    private:
        // if smth is closer to the side of the opposing player station then subtract that # from the variable 
        double initLineError_ ;
        double trenchDistError_;
        double trenchWidthError_;
        double trenchLengthError_;
        double targetZDistError_;
        double loadingDDistError_;
        double playerSt2MidError_; // add positive number if it's more to the left than expected 

        // Distance of Initiation Line To...
        double distInitLineToPS_; 
        double distInitLinetoTrench_;
        double distInitLinetoTZ_;
        double distSidewaysTZToMidTrench; // InitLineAlignedWithTZToInitLineAlignedWithMidTrench
        double distSidewaysLBToMidTrench_; //InitLineAlignedWithLBToInitLineAlignedWithMidTrench_
        double distInitLinetoCP_; // should = distInitLinetoTrench + trenchLength_
        double distInitLineAlignedWithPSToMidTrench_;
        double distSidewaysPSToMidTrench_; 

        // Trench
        double trenchWidth_;
        double trenchLength_; // not entire trench length just side of trench near opposing alliance stations to control panel

        double distCenterLBtoCenterTZ_;
        double distSidewaysMidPSToMidTrench_; //distInitLineAlignedWithMidPSToInitLineAlignedWithMidTrench_
};

//1 
// shoot 3 t -33.0 d -8.7 t 0.0 d -9.5 d 9.5 t -33.0 d 8.7 t 0.0