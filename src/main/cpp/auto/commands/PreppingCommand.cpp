/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/PreppingCommand.h"

//Constructor
PreppingCommand::PreppingCommand(RobotModel * robot, double desiredVelocity) : AutoCommand() {
    printf("prepping command\n");
    //initialize variables
    robot_ = robot;
    isDone_ = false;
    setVelocity_ = true;
    desiredVelocity_ = desiredVelocity;
}

//Constructor
PreppingCommand::PreppingCommand(RobotModel * robot) : AutoCommand() {
    printf("prepping command\n");
    //initialize variables
    robot_ = robot;
    isDone_ = false;
    setVelocity_ = false;
    desiredVelocity_ = 0.0;
}

//set desired velocity
void PreppingCommand::Init(){
    isDone_ = false;
    if(!setVelocity_){
        desiredVelocity_ = robot_->CalculateFlywheelVelocityDesired();
    }
}

//prep -> done
void PreppingCommand::Update(double currTimeSec, double deltaTimeSec){
    
    printf("STILL PREPPING\n");
    robot_->SetPrepping(desiredVelocity_);
    isDone_ = true;
}

//set done
bool PreppingCommand::IsDone(){
    return isDone_;
}

//done
void PreppingCommand::Reset(){
    isDone_ = true;
}

PreppingCommand::~PreppingCommand(){
    
}
