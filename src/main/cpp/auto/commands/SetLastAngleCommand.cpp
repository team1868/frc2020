// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "auto/commands/SetLastAngleCommand.h"

/**
 * Constructor
 * @param robot a RobotModel
 * @param lastAngle a double
 */ 
SetLastAngleCommand::SetLastAngleCommand(RobotModel *robot, double lastAngle) : AutoCommand(){
    // initialize variables
    robot_ = robot;
    lastAngle_ = lastAngle;
    isDone_ = false;
}

/**
 * Initialize command 
 */ 
void SetLastAngleCommand::Init(){}

/**
 * Periodic update
 * @param currTimeSec a double
 * @param deltaTimeSec a double
 */ 
void SetLastAngleCommand::Update(double currTimeSec, double deltaTimeSec){
    robot_->SetLastPivotAngle(lastAngle_);
    isDone_ = true;
}

/**
 * Resets for standby
 */ 
void SetLastAngleCommand::Reset(){
    isDone_ = true;
}

/**
 * Returns true if command is done
 * @return isDone_
 */ 
bool SetLastAngleCommand::IsDone(){
    return isDone_;
}

/**
 * Destructor
 */ 
SetLastAngleCommand::~SetLastAngleCommand(){}