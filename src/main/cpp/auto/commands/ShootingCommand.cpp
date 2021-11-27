/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/ShootingCommand.h"

/**
 * Constructor
 * @param robot a RobotModel
 * @param autoVelocity a double
 */ 
ShootingCommand::ShootingCommand(RobotModel * robot, double autoVelocity) : AutoCommand() {
    printf("shooting command\n");

    // initialize variables
    robot_ = robot;
    isDone_ = false;
    startShootingTime_ = 0.0;

    autoVelocity_ = autoVelocity; // this is flywheel velocity
    setVelocity_ = true;
}

/**
 * Constructor without auto velocity
 * @param robot a RobotModel
 */ 
ShootingCommand::ShootingCommand(RobotModel * robot) : AutoCommand() {
    printf("shooting command\n");

    // initialize variables
    robot_ = robot;
    isDone_ = false;
    startShootingTime_ = 0.0;

    autoVelocity_ = 0.0; // this is flywheel velocity
    setVelocity_ = false;
}

/**
 * Initializes command (gets start time, desired velocity, sets robot to shooting state)
 */ 
void ShootingCommand::Init(){

    isDone_ = false;
    startShootingTime_ = robot_->GetTime();

    // if constructor doesn't have desired velocity parameter, calculate and set desired velocity
    if(!setVelocity_){
        autoVelocity_ = robot_->CalculateFlywheelVelocityDesired();
    }

    // set robot to shooting state
    robot_->SetShooting(autoVelocity_);
    robot_->ShootingAutoInit();

}

/**
 * Periodic update
 * @param currTimeSec a double
 * @param deltaTimeSec a double
 */ 
void ShootingCommand::Update(double currTimeSec, double deltaTimeSec){
    // timeout or check if done (when at desired velocity)
    if(robot_->GetTime()-startShootingTime_ >= 8.0){
        isDone_ = true;
    } else {
        isDone_ = robot_->GetShootingIsDone();
    }

    // when shooting stops, stop flywheel
    if(isDone_){
        robot_->SetControlModeVelocity(0.0);
    }
}

/**
 * Returns true if command is done
 * @return isDone_
 */
bool ShootingCommand::IsDone(){
    return isDone_;
}

/**
 * Reset robot for standby
 */ 
void ShootingCommand::Reset(){
    isDone_ = true;
}

/**
 * Destructor
 */
ShootingCommand::~ShootingCommand(){}
