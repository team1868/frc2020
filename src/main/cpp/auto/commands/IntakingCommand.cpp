/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/IntakingCommand.h"

/**
 * Constructor
 * @param robot a RobotModel
 */
IntakingCommand::IntakingCommand(RobotModel * robot) : AutoCommand() {
    robot_ = robot;
    isDone_ = false;
    printf("intaking command\n");
}

/**
 *  Initializes 
 */ 
void IntakingCommand::Init(){
    isDone_ = false; // intaking command has just been called so command is not done yet
}

/**
 * Sets the robot's state to intaking, command is done
 */ 
void IntakingCommand::Update(double currTimeSec, double deltaTimeSec){
    robot_->SetIntaking(); // is true when wrist is up
    isDone_ = true;
}

/** 
 * Checks if command is done. Returns true if done
 * @return isDone_
 */ 
bool IntakingCommand::IsDone(){
    return isDone_;
}

/**
 * Sets command as done. 
 */ 
void IntakingCommand::Reset(){
    isDone_ = true;
}

/**
 * Destructor
 */ 
IntakingCommand::~IntakingCommand(){}
