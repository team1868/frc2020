/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"
#include <stdio.h>
#include <math.h>

//checks whether alliance is blue or red 
void RobotModel::CheckAllianceColor(){
    if(frc::DriverStation::GetInstance().GetAlliance() == frc::DriverStation::kRed){
        /** assumes inputted offset values are of blue alliance's side but if playing for red alliance then
        need to account for errors by subtracting that amount from red's side */
        initLineError_ *= -1; 
        trenchDistError_ *= -1;
        trenchWidthError_ *= -1;
        trenchLengthError_ *= -1;
        targetZDistError_ *= -1;
        loadingDDistError_ *= -1;
        playerSt2MidError_ *= -1;
        printf("alliance color is red");
    } else {
        printf("alliance color is blue");
    }
    
    //setting distances according to frc manual
    trenchLength_ = 120.0; 
    trenchWidth_ = 55.5;
    distInitLineToPS_ = 85.0;
    distInitLinetoTrench_ = 83.63;
    distInitLinetoTZ_ = 85.0;
    distInitLinetoLB_ = 85.0;
    distSidewaysTZToMidTrench_ = 66.91;
    distMidPSToMidTZ_ = 58.5;
    distSidewaysPSToMidTrench_ = 133.875;
    distCenterLBtoCenterTZ_ = 97.75;

    //accounting for differences b/w distances shown on frc manual compared to competition arena
    distInitLineToPS_ += initLineError_; // if it's closer to the opposing player station then subtract that amt from dist
    if(distInitLineToPS_ < 0.0){
        printf("error: distance value for init line to player station is negative");
        distInitLineToPS_ = 85.0;
    }
    distInitLinetoTrench_ += trenchDistError_;
    if(distInitLinetoTrench_ < 0.0){
        printf("error: distance value for init line to trench is negative");
        distInitLineToPS_ = 83.63;
    }
    trenchWidth_ += trenchWidthError_;
    if(trenchWidth_ < 0.0){
        printf("error: distance value for trench width is negative");
        trenchWidth_ = 55.5;
    }
    trenchLength_ += trenchLengthError_;
    if(trenchLength_ < 0.0){
        printf("error: distance value for trench length is negative");
        trenchLength_ = 120.0;
    }
    distInitLinetoTZ_ += initLineError_;
    if(distInitLinetoTZ_ < 0.0){
        printf("error: distance value for init line to target zone is negative");
        distInitLinetoTZ_ = 85.0;
    }
    distInitLinetoLB_ += initLineError_;
    if(distInitLinetoLB_ < 0.0){
        printf("error: distance value for init line to loading bay is negative");
        distInitLinetoLB_ = 85.0;
    }
    distCenterLBtoCenterTZ_ += playerSt2MidError_;
    if(distCenterLBtoCenterTZ_ < 0.0){
        printf("error: distance value for center of loading bay to center of target zone is negative");
        distInitLineToPS_ = 83.63;
    }
    distInitLinetoCP_ = distInitLinetoTrench_ + trenchLength_;
    distMidPSToMidTZ_ += (targetZDistError_ / 2.0);
    if(distMidPSToMidTZ_ < 0.0){
        printf("error: distance value for mid player station to mid target zone is negative");
        distInitLineToPS_ = 83.63;
    }
    distSidewaysPSToMidTrench_ += playerSt2MidError_;
    if(distSidewaysPSToMidTrench_ < 0.0){
        printf("error: distance value from player station to mid-trench sideways is negative");
        distInitLineToPS_ = 83.63;
    }
    distSidewaysTZToMidTrench_ -= (trenchWidthError_ /2.0); // if trench is a bit wider than the dist b/w TZ to mid trench sideways will decrease
    if(distSidewaysTZToMidTrench_ < 0.0){
        printf("error: distance value from target zone to mid-trench sideways is negative");
        distSidewaysTZToMidTrench_ = 66.91;
    }
    distSidewaysLBToMidTrench_ = 113.0 + distSidewaysPSToMidTrench_; 
    double temp = (distInitLinetoTrench_ * distInitLinetoTrench_ + distSidewaysPSToMidTrench_ * distSidewaysPSToMidTrench_);
    if (temp < 0.0){
        printf("error:trying to square root negative value");
        // distInitLineAlignedWithPSToMidTrench_ = 
    } else {
        distInitLineAlignedWithPSToMidTrench_ = sqrt(distInitLinetoTrench_ * distInitLinetoTrench_ + distSidewaysPSToMidTrench_ * distSidewaysPSToMidTrench_);
    }
    strTrenchLength_ = std::to_string(trenchLength_);
}

//returns chosen sequence
std::string RobotModel::GetChosenSequence() {
    return autoSendableChooser_.GetSelected();
}

std::string RobotModel::GetDefaultSequence(){
    return "t 0.0 d 0.0";
}

//sets angles & distances for sequence 1 based off competition arena's measurements
//sequence 1 = start in front of TZ go to trench & come back
std::string RobotModel::GetChosenSequence1() {
    CheckAllianceColor();

    angleA1_ = -((atan(distSidewaysTZToMidTrench_/distInitLinetoTrench_ ) - 5.66231) * (180/PI));
    strAngleA1_ = std::to_string(angleA1_);
    distA1_ = sqrt(distSidewaysTZToMidTrench_ * distSidewaysTZToMidTrench_ + distInitLinetoTrench_ * distInitLinetoTrench_) / 12 - 0.2252003356;
    strDistA1_ = std::to_string(distA1_);
    distB1_ = trenchLength_ /12 - 0.5;
    strDistB1_ = std::to_string(distB1_);

    testSequence1_ = "t " + strAngleA1_ + " d " + strDistA1_ + " t 0.0 d -" + strDistB1_ + " d " + strDistB1_ + " t " + strAngleA1_ + " d " + strDistA1_ + " t 0.0";
    return testSequence1_;
}

//sets angles & distances for sequence 2 based off competition arena's measurements
//sequence 2 = start in front of LB go to trench & got to init line in front of TZ
std::string RobotModel::GetChosenSequence2() {
    CheckAllianceColor();

    angleA2_ = atan(distCenterLBtoCenterTZ_/distInitLinetoLB_) * (180/PI);
    strAngleA2_ = std::to_string(angleA2_);
    angleB2_ = atan(distSidewaysLBToMidTrench_/distInitLinetoTrench_) * (180/PI);
    strAngleB2_ = std::to_string(angleB2_);
    angleC2_ = atan(distSidewaysTZToMidTrench_/distInitLinetoTrench_) * (180/PI);
    strAngleC2_ = std::to_string(angleC2_);
    distA2_ = sqrt(distInitLinetoTrench_ * distInitLinetoTrench_ + distSidewaysLBToMidTrench_ * distSidewaysLBToMidTrench_) / 12;
    strDistA2_ = std::to_string(distA2_);
    distB2_ = sqrt(distSidewaysTZToMidTrench_ * distSidewaysTZToMidTrench_ + distInitLinetoTrench_ * distInitLinetoTrench_) / 12 - 0.2252003356;
    strDistB2_ = std::to_string(distB2_);

    testSequence2_ = "t " + strAngleA2_ + " t " + strAngleB2_ + " d " + strDistA2_ + " t 0.0 " + " d -" + strTrenchLength_ + " d " + strTrenchLength_ + " t -" + strAngleC2_ + " d " + strDistB2_;
    return testSequence2_;
}

//sets angles & distances for sequence 3 based off competition arena's measurements
//sequence 3 = start in front of trench, go to trench & come back
std::string RobotModel::GetChosenSequence3() {
    CheckAllianceColor();

    angleA3_ = atan(distSidewaysTZToMidTrench_/distInitLinetoTZ_) * (180/PI);
    strAngleA3_ = std::to_string(angleA3_);
    distA3_ = (trenchLength_ + distInitLinetoTrench_)/ 12;
    strDistA3_ = std::to_string(distA3_);
    testSequence3_ = "t -" + strAngleA3_ + " t 0.0 d -" + strDistA3_ + " d " + strDistA3_ + " t -" + strAngleA3_;
    return testSequence3_;
}

//sets angles & distances for sequence 4 based off competition arena's measurements
//sequence 1 = start in front of TZ go to trench & come back
    std::string RobotModel::GetChosenSequence4() {
    CheckAllianceColor();

    angleA4_ = atan(distMidPSToMidTZ_/distInitLinetoTZ_) * (180/PI);
    strAngleA4_ = std::to_string(angleA4_);
    angleB4_ = atan(distSidewaysPSToMidTrench_/distInitLinetoTrench_) * (180/PI);
    strAngleB4_ = std::to_string(angleB4_);
    angleC4_ = angleA4_ + angleB4_;
    strAngleC4_ = std::to_string(angleC4_);
    distA4_ = sqrt(distMidPSToMidTZ_ * distMidPSToMidTZ_ + distInitLinetoTrench_ * distInitLinetoTrench_) /12;
    strDistA4_ = std::to_string(distA4_);
    distB4_ = sqrt(distSidewaysTZToMidTrench_ * distSidewaysTZToMidTrench_ + distInitLinetoTrench_ * distInitLinetoTrench_) / 12 - 0.2252003356;
    strDistB4_ = std::to_string(distB4_);

    testSequence4_ = "t " + strAngleA4_ + " t -" + strAngleB4_ + " d -" + strDistA4_ + " t 0.0 d -" + strTrenchLength_ + " d " + strTrenchLength_ + " t -" + strAngleA4_ + " d " + strDistB4_ + " t 0.0";
    return testSequence4_;
}
