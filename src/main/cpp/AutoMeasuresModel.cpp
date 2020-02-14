/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"
#include <stdio.h>
#include <math.h>
#include "auto/AutoMeasures.h"

/* how to do arctan in c++ 
// search up string concatenation in c++ 
// 1) set dists to account for error
// 2) make strings

/* 1
/* 2 switch case check which auto seqence it is & based on that 
cpp ??
*/ 

/* how to do arctan in c++ 
// search up string concatenation in c++ 
// 1) set dists to account for error
// 2) make strings

/* 1
/* 2 switch case check which auto seqence it is & based on that 
cpp ??
*/ 
AutoMeasures::AutoMeasures(){
    distInitLineToPS_ = 85.0; 
    distInitLinetoTrench_ = 83.63;
    distInitLinetoTZ_ = 85.0; 
    distSidewaysTZToMidTrench = 66.91;
    distSidewaysLBToMidTrench_ = 113 + distSidewaysMidPSToMidTrench_; 
    distSidewaysPSToMidTrench_ = 133.875; 
    distInitLineAlignedWithPSToMidTrench_ = sqrt(distInitLinetoTrench_ * distInitLinetoTrench_ + distSidewaysMidPSToMidTrench_ * distSidewaysMidPSToMidTrench_);

    distInitLineToPS_ += initLineError_;
    distInitLinetoTrench_ += trenchDistError_;
    trenchWidth_ += trenchWidthError_;
    trenchLength_ += trenchLengthError_;
    distCenterLBtoCenterTZ_ += playerSt2MidError_; 
    distInitLinetoCP_ = distInitLinetoTrench_ + trenchLength_;
}






//1 
// shoot 3 t -33.0 d -8.7 t 0.0 d -9.5 d 9.5 t -33.0 d 8.7 t 0.0
 





