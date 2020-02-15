/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>

class RobotModel;
 /*------------------- NAVX PID SOURCE!! -------------------*/

class NavXPIDSource : public frc::PIDSource {
public:
	/**
	 * Assigns the robot and resets the accumulated yaw
	 * @param RobotModel
	 */
	NavXPIDSource(RobotModel *robot);
	/**
	 * Calculates and returns AccumulatedYaw
	 * @return AccumulatedYaw
	 */
	double PIDGet();
	/**
	 * Updates currYAW, calculates deltaYaw and accumulatedYaw
	 */
	double CalculateAccumulatedYaw();
	/**
	 * Sets AccumulatedYaw and deltaYaw to zero
	 * Updates currYaw and lastYaw
	 */
	void ResetAccumulatedYaw();
	/**
	 * Destructor
	 */
	virtual ~NavXPIDSource();

private:
	double currYaw_, lastYaw_, deltaYaw_, accumulatedYaw_;
	RobotModel *robot_;

};

/*------------------- TALON ENCODER PID SOURCE!! -------------------*/

class TalonEncoderPIDSource : public frc::PIDSource {
public:
	/**
	 * Assigns robot and sets averageTalonDistance to 0
	 * @param RobotModel
	 */
	TalonEncoderPIDSource(RobotModel *robot);
	/**
	 *Gets distance from left and right encoders and sets averageTalonDistance
	 *as average of the two
	 *@return averageTalonDistance_
	 */
	double PIDGet();
	/**
	 * Destructor
	 */
	virtual ~TalonEncoderPIDSource();
private:
	RobotModel *robot_;
	/**
	 * Average distance of left and right encoders
	 */
	double averageTalonDistance_;
};

class TalonFXPIDSource : public frc::PIDSource {
public:
	/**
	 * Assigns robot and sets averageTalonDistance to 0
	 * @param RobotModel
	 */
	TalonFXPIDSource(RobotModel *robot);
	/**
	 *Gets distance from left and right encoders and sets averageTalonDistance
	 *as average of the two
	 *@return averageTalonDistance_
	 */
	double PIDGet();
	/**
	 * Destructor
	 */
	virtual ~TalonFXPIDSource();
private:
	RobotModel *robot_;
	/**
	 * Average distance of left and right encoders
	 */
	double averageTalonDistance_;
};

class VelocityPIDSource : public frc::PIDSource {
public:
	
	VelocityPIDSource(RobotModel *robot);
	
	void UpdateVelocity();
	double PIDGet();
	/**
	 * Destructor
	 */
	virtual ~VelocityPIDSource();
private:
	RobotModel *robot_;
	double lastTime_, currTime_;
	double lastAvgPosition_, currAvgPosition_;
	double avgVelocity_;
};
 
