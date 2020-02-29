/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/PreppingCommand.h"

PreppingCommand::PreppingCommand(RobotModel * robot, double desiredVelocity) : AutoCommand() {
    printf("prepping command\n");
    robot_ = robot;
    isDone_ = false;
    desiredVelocity_ = desiredVelocity;
}

void PreppingCommand::Init(){
    isDone_ = false;

}

void PreppingCommand::Update(double currTimeSec, double deltaTimeSec){
    robot_->SetPrepping(desiredVelocity_);
    //robot_->SetPrepping(desiredVelocity_);
    isDone_ = robot_->IsAutoFlywheelAtSpeed(desiredVelocity_);

    //isDone_ = true;
}

bool PreppingCommand::IsDone(){
    return isDone_;
}

void PreppingCommand::Reset(){
    isDone_ = true;
}

PreppingCommand::~PreppingCommand(){
    
}
