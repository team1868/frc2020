/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/PIDSource/PIDInputSource.h"
#include <frc/WPILib.h>
#include "RobotModel.h"

/**
 * Assigns the robot and resets the accumulated yaw
 * initializes accumulatedYawEntry_ to 0
 * @param RobotModel
 */
NavXPIDSource::NavXPIDSource(RobotModel *robot) {
	robot_ = robot;
	lastYaw_ = robot_->GetNavXYaw();
	currYaw_ = robot_->GetNavXYaw();
	accumulatedYaw_ = currYaw_;
	deltaYaw_ = currYaw_ - lastYaw_;
}

/**
 * calculates accumulatedYaw 
 * @return accumulatedYaw_
 */
double NavXPIDSource::PIDGet() {
	accumulatedYaw_ = robot_->GetNavXYaw();
	return accumulatedYaw_;
}
 
/**
 * Updates currYAW, calculates deltaYaw and accumulatedYaw
 * @return accumulatedYaw
 */
double NavXPIDSource::CalculateAccumulatedYaw() {
	lastYaw_ = currYaw_;
	currYaw_ = robot_->GetNavXYaw();
	deltaYaw_ = currYaw_ - lastYaw_;

	if (deltaYaw_ < -180) {
		accumulatedYaw_ += (180 - lastYaw_) + (180 + currYaw_);
	} else if (deltaYaw_ > 180) {
		accumulatedYaw_ -= (180 + lastYaw_) + (180 - currYaw_);
	} else {
		accumulatedYaw_ += deltaYaw_;
	}

	return accumulatedYaw_;
}

/**
 * Sets AccumulatedYaw and deltaYaw to zero
 * Updates currYaw and lastYaw
 */
void NavXPIDSource::ResetAccumulatedYaw() {
	accumulatedYaw_ = 0.0;
	currYaw_ = robot_->GetNavXYaw();
	printf("finished resetting yaw\n");
	lastYaw_ = currYaw_;
	deltaYaw_ = 0.0;
}

/**
 * Destructor
 */
NavXPIDSource::~NavXPIDSource() {
}

/**
 * Assigns robot, sets averageTalonDistance to 0
 * @param RobotModel
 */
TalonEncoderPIDSource::TalonEncoderPIDSource(RobotModel* robot) {
	robot_ = robot;
	averageTalonDistance_ = 0.0;
}

/**
 * Gets distance from left and right encoders and sets averageTalonDistance
 * as average of the two
 * @return averageTalonDistance_
 */
//not used and doesn't work
double TalonEncoderPIDSource::PIDGet() {
	double leftDistance = robot_->GetLeftDistance();
	double rightDistance = robot_->GetRightDistance();

	averageTalonDistance_= (rightDistance + leftDistance) / 2;
	return averageTalonDistance_;

}

/**
 * Destructor
 */
TalonEncoderPIDSource::~TalonEncoderPIDSource() {
}

/** 
 * Assigns the robot
 * Updates lastTime_ and currTime_
 * Sets lastAvgPosition_, currAvgPosition and avgVelocity to 0
 * @param RobotModel
 */
VelocityPIDSource::VelocityPIDSource(RobotModel *robot){
	robot_ = robot;
	lastTime_ = robot_->GetTime();
	currTime_ = lastTime_;
	lastAvgPosition_ = 0.0;
	currAvgPosition_ = 0.0;
	avgVelocity_ = 0.0;
}

/**
 * Updates lastTime_, currTime_, lastAvgPosition_ and currAvgPosition_ 
 * Calculates avgVelocity_
 */	
//unused 
void VelocityPIDSource::UpdateVelocity(){
	lastTime_ = currTime_;
	currTime_ = robot_->GetTime();

	lastAvgPosition_ = currAvgPosition_;
	currAvgPosition_ = (robot_->GetLeftDistance()+robot_->GetRightDistance())/2.0;

	avgVelocity_ = (currAvgPosition_-lastAvgPosition_)/(currTime_-lastTime_);
}

/**
 * @return avgVelocity_
 */
double VelocityPIDSource::PIDGet(){
	return avgVelocity_;
}

/**
 * Destructor
 */
VelocityPIDSource::~VelocityPIDSource(){

}
