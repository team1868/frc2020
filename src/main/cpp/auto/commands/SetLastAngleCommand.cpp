// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "auto/commands/SetLastAngleCommand.h"

SetLastAngleCommand::SetLastAngleCommand(RobotModel *robot, double lastAngle) : AutoCommand(){
    robot_ = robot;
    lastAngle_ = lastAngle;
    isDone_ = false;
}
void SetLastAngleCommand::Init(){

}
void SetLastAngleCommand::Update(double currTimeSec, double deltaTimeSec){
    robot_->SetLastPivotAngle(lastAngle_);
    isDone_ = true;
}
void SetLastAngleCommand::Reset(){
    isDone_ = true;
}
bool SetLastAngleCommand::IsDone(){
    return isDone_;
}

SetLastAngleCommand::~SetLastAngleCommand(){
    
}