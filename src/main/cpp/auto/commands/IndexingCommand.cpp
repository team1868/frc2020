/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/IndexingCommand.h"

/**
 * Constructor
 * @param robot a RobotModel
 */
IndexingCommand::IndexingCommand(RobotModel * robot) : AutoCommand() {
    robot_ = robot;
    isDone_ = false;
    printf("indexing command\n");
}

/**
 * Initializes class for run 
 */
void IndexingCommand::Init(){
    isDone_ = false;
}

/** 
 * Periodic update
 * @param currTimeSec current time
 * @param deltaTimeSec delta time
 */
void IndexingCommand::Update(double currTimeSec, double deltaTimeSec){
    isDone_ = true;
    robot_->SetIndexing();
}

/**
 * Returns true if indexing is done
 * @return isDone_
 */
bool IndexingCommand::IsDone(){
    return isDone_;
}

/**
 * Resets robot to standby 
 */
void IndexingCommand::Reset(){
    isDone_ = true;
}

/**
 * Destructor 
 */
IndexingCommand::~IndexingCommand(){
    Reset();
}
