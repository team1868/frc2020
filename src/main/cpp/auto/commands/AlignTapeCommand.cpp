/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/AlignTapeCommand.h"

AlignTapeCommand::AlignTapeCommand(RobotModel *robot, NavXPIDSource *navXSource) : AutoCommand() {
    robot_ = robot;
    lastJetsonAngle_ = 0.0;
    currJetsonAngle_ = 0.0;
    jetsonAngleTolerance_ = 3.0;
    aligning_ = false;
    pivotCommand_ = nullptr;
    //navXSource_ = new NavXPIDSource(robot_);
    navXSource_ = navXSource;
    isDone_ = false;
    maxTime_ = 6.0;
    startTime_ = 0.0;
}

void AlignTapeCommand::Init(){
    printf("init in align tape command\n");
    lastJetsonAngle_ = robot_->GetDeltaAngle();
    currJetsonAngle_ = robot_->GetDeltaAngle();
    startTime_ = robot_->GetTime();
    robot_->SetLight(true);
    robot_->SendZMQ(true);
    aligning_ = false;
    isDone_ = false;
}

void AlignTapeCommand::Update(double currTimeSec, double deltaTimeSec){
    printf("updating align :DD\n");
    robot_->SendZMQ(true);
    if(!aligning_){
        lastJetsonAngle_ = currJetsonAngle_;
        currJetsonAngle_ = robot_->GetDeltaAngle();
        if(robot_->ZMQHasContents() && fabs(lastJetsonAngle_-currJetsonAngle_) <= jetsonAngleTolerance_){
            printf("received last angle %f and curr angle %f, starting\n", lastJetsonAngle_, currJetsonAngle_);
            aligning_ = true;
            printf("turning to angle %f in align tape\n", currJetsonAngle_);
            pivotCommand_ = new PivotCommand(robot_, robot_->GetNavXYaw()+lastJetsonAngle_, true, navXSource_, jetsonAngleTolerance_);
            pivotCommand_->Init();
        }
    } else {
        if(pivotCommand_!=nullptr && !pivotCommand_->IsDone() && currTimeSec-startTime_<=maxTime_){
            pivotCommand_->Update(currTimeSec, deltaTimeSec);
        } else {
            printf("DONE with align tape\n");
            isDone_ = true;
        }
    }
}
bool AlignTapeCommand::IsDone(){
    return isDone_;
}

void AlignTapeCommand::Reset(){
    printf("DONE... resetting\n");
    if(pivotCommand_!=nullptr){
        pivotCommand_->Reset();
        delete pivotCommand_;
        pivotCommand_ = nullptr;
    }
    robot_->SetLight(false);
    //robot_->SendZMQ(false);
    isDone_ = true;
    //printf("here!\n");
}

AlignTapeCommand::~AlignTapeCommand(){
    Reset();
}