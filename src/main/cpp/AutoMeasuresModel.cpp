/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"
#include <stdio.h>
#include <math.h>
//using namespace std;

void RobotModel::CheckAllianceColor(){
if(frc::DriverStation::GetInstance().GetAlliance() == DriverStation::kRed){
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
trenchLength_ = 120.0; // QFK: the blue trench being an inch wider doesn't automatically make the red trench an inch less wide??
trenchWidth_ = 55.5;
distInitLineToPS_ = 85.0;
distInitLinetoTrench_ = 83.63;
distInitLinetoTZ_ = 85.0;
distInitLinetoLB_ = 85.0;
distSidewaysTZToMidTrench_ = 66.91;
distMidPSToMidTZ_ = 58.5;
distSidewaysPSToMidTrench_ = 133.875;

distInitLineToPS_ += initLineError_; // if it's closer to the opposing player station then subtract that amt from dist
distInitLinetoTrench_ += trenchDistError_;
trenchWidth_ += trenchWidthError_;
trenchLength_ += trenchLengthError_;
distInitLinetoTZ_ += initLineError_;
distInitLinetoLB_ += initLineError_;
distCenterLBtoCenterTZ_ += playerSt2MidError_;
distInitLinetoCP_ = distInitLinetoTrench_ + trenchLength_;
distMidPSToMidTZ_ += (targetZDistError_ / 2);
distSidewaysPSToMidTrench_ += playerSt2MidError_;
distSidewaysTZToMidTrench_ -= (trenchWidthError_ /2); // if trench is a bit wider than the dist b/w TZ to mid trench sideways will decrease
distSidewaysLBToMidTrench_ = 113 + distSidewaysMidPSToMidTrench_; //shouldn't the 113 also be a variable?
distInitLineAlignedWithPSToMidTrench_ = sqrt(distInitLinetoTrench_ * distInitLinetoTrench_ + distSidewaysMidPSToMidTrench_ * distSidewaysMidPSToMidTrench_);
strTrenchLength_ = std::to_string(trenchLength_);

}



std::string RobotModel::GetChosenSequence() {
    return autoSendableChooser_.GetSelected();
}
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
std::string RobotModel::GetChosenSequence3() {
CheckAllianceColor();

angleA3_ = atan(distSidewaysTZToMidTrench_/distInitLinetoTZ_) * (180/PI);
strAngleA3_ = std::to_string(angleA3_);
distA3_ = (trenchLength_ + distInitLinetoTrench_)/ 12;
strDistA3_ = std::to_string(distA3_);
testSequence3_ = "t -" + strAngleA3_ + " t 0.0 d -" + strDistA3_ + " d " + strDistA3_ + " t -" + strAngleA3_;
return testSequence3_;
}
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
