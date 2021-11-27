/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/PreppingCommand.h"

/**
 * Constructor
 * @param robot a RobotModel
 * @param desiredVelocity a double
 */ 
PreppingCommand::PreppingCommand(RobotModel * robot, double desiredVelocity) : AutoCommand() {
    printf("prepping command\n");
    
    //initialize variables
    robot_ = robot;
    isDone_ = false;
    setVelocity_ = true;
    desiredVelocity_ = desiredVelocity;
}

/**
 * Constructor without desired velocity
 * @param robot a RobotModel
 */ 
PreppingCommand::PreppingCommand(RobotModel * robot) : AutoCommand() {
    printf("prepping command\n");
    //initialize variables
    robot_ = robot;
    isDone_ = false;
    setVelocity_ = false;
    desiredVelocity_ = 0.0;
}

/**
 * Initializes command, sets desired velocity
 */ 
void PreppingCommand::Init(){
    isDone_ = false;
    if(!setVelocity_){
        desiredVelocity_ = robot_->CalculateFlywheelVelocityDesired();
    }
}

/**
 * Periodic update
 * @param currTimeSec a double
 * @param deltaTimeSec a double
 */ 
void PreppingCommand::Update(double currTimeSec, double deltaTimeSec){
    
    printf("STILL PREPPING\n");
    robot_->SetPrepping(desiredVelocity_);
    isDone_ = true;
}

/**
 * Returns true if done
 * @return isDone_
 */
bool PreppingCommand::IsDone(){
    return isDone_;
}

/**
 * Resets
 */ 
void PreppingCommand::Reset(){
    isDone_ = true;
}

/**
 * Destructor
 */ 
PreppingCommand::~PreppingCommand(){}
