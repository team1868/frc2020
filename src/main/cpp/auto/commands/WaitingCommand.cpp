/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/WaitingCommand.h"

/** 
 * Assigns the waitTimeSec and creates the timer
 * @param robot a RobotModel
 * @param waitTimeSec a double
 */
WaitingCommand::WaitingCommand(RobotModel *robot, double waitTimeSec) : AutoCommand() {
	// initialize variables
	waitTimeSec_ = waitTimeSec;
	robot_ = robot;
	startTime_ = -1.0;
	isDone_ = false;
}

/**
 * Initializes command, starts timer
 */
void WaitingCommand::Init() {
	startTime_ = robot_->GetTime();
}

/** 
 * Periodic update, checks if the timer meets the waitTimeSec
 * @param currTimeSec a double
 * @param deltaTimeSec a double
 */
void WaitingCommand::Update(double currTimeSec, double deltaTimeSec) {
	// stay in auto command as long as not timed out
	if (startTime_ > 0.0){
		isDone_ = currTimeSec - startTime_ >= waitTimeSec_;
		if (isDone_) {
			printf("done waiting %f", currTimeSec);
		}
	} else {
		Init();
 	}

}

/**
 * Returns true if command is done
 * @returns isDone_
 */
bool WaitingCommand::IsDone() {
	return isDone_;
}

/**
 * Resets for standby
 */ 
void WaitingCommand::Reset() {
	isDone_ = true;
}

/**
 * Destructor
 */ 
WaitingCommand::~WaitingCommand() {}