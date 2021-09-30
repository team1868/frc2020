/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/AlignTapeCommand.h"

// constructor, called when robot knows will align in some future
AlignTapeCommand::AlignTapeCommand(RobotModel *robot, NavXPIDSource *navXSource, PivotPIDTalonOutput *talonOutput) : AutoCommand() {

    // initialize class variables
    robot_ = robot;
    navXSource_ = navXSource;
    pivotCommand_ = nullptr;
    talonOutput_ = talonOutput;

    lastJetsonAngle_ = 0.0;
    currJetsonAngle_ = 0.0;
    jetsonAngleTolerance_ = 3.0;

    aligning_ = false;
    isDone_ = false;

    maxTime_ = 2.0;
    startTime_ = 0.0;
    
    printf("done with constructor in AlignTapeCommand\n");
}

// constructor, called when robot knows will align in some future
AlignTapeCommand::AlignTapeCommand(RobotModel *robot, NavXPIDSource *navXSource) : AutoCommand() {

    // initialize class variables
    robot_ = robot;
    navXSource_ = navXSource;
    pivotCommand_ = nullptr;
    talonOutput_ = new PivotPIDTalonOutput(robot_);

    lastJetsonAngle_ = 0.0;
    currJetsonAngle_ = 0.0;
    jetsonAngleTolerance_ = 3.0;

    aligning_ = false;
    isDone_ = false;

    maxTime_ = 2.0;
    startTime_ = 0.0;

    printf("done with constructor in AlignTapeCommand\n");
}

// initialize class variables, called when about to start aligning
void AlignTapeCommand::Init(){
    printf("init in align tape command\n");

    // initialize class variables
    lastJetsonAngle_ = robot_->GetDeltaAngle();
    currJetsonAngle_ = robot_->GetDeltaAngle();
    startTime_ = robot_->GetTime();
    aligning_ = false;
    isDone_ = false;

    // set light and exposure
    robot_->SetLight(true);
    robot_->SendZMQ(true);
}

// periodic update while executing command
void AlignTapeCommand::Update(double currTimeSec, double deltaTimeSec){

    // tell jetson to lower exposure
    robot_->SendZMQ(true);

    // timeout, if bad or 0 values received
    if(currTimeSec-startTime_>=5.0){
        printf("WARNING: timeout in AlignTapeCommand, did not receive good values from jetson\n");
        isDone_ = true;
    } else {
        // not timed out

        // check if already has a target angle
        if(!aligning_){
            // does not already have a target angle

            lastJetsonAngle_ = currJetsonAngle_;
            currJetsonAngle_ = robot_->GetDeltaAngle();

            // check if two read values are similar, if so then that is the target angle
            /* Note: read values are from RobotModel, not necessarily two adjacent values from vision,
            * depending on update speed difference
            */
            if(robot_->ZMQHasContents() && fabs(lastJetsonAngle_-currJetsonAngle_) <= jetsonAngleTolerance_ && robot_->GetDistance() > 0.0){
                printf("successfully found similar angles: last angle was %f and currrent angle is %f, starting\n", lastJetsonAngle_, currJetsonAngle_);
                printf("turning to angle %f in align tape\n", currJetsonAngle_);

                aligning_ = true;

                //c reate and initialize pivot command to the most recently read angle

                pivotCommand_ = new PivotCommand(robot_, robot_->GetNavXYaw()+lastJetsonAngle_, true, navXSource_, 1.2, talonOutput_);
                pivotCommand_->Init();
            }
        } else {
            // already has a target angle and a pivot command

            // has pivot command, not done, and not timed out
            if(pivotCommand_ != nullptr && !pivotCommand_->IsDone() && currTimeSec-startTime_<=maxTime_){
                // pivot command is not finished
                pivotCommand_->Update(currTimeSec, deltaTimeSec);
            } else {
                // pivot command is finished, either by completely aligning or timing out
                printf("done with aligning to tape\n");

                // timing out
                if(currTimeSec-startTime_>maxTime_){
                    printf("WARNING: cause is timeout\n");
                }

                isDone_ = true;
            }
        }
    }
}

// check if aligned to tape
bool AlignTapeCommand::IsDone(){
    return isDone_;
}

// check for memory leaks and background pid
void AlignTapeCommand::Reset(){
    printf("done, resetting\n");

    // remove pivot command
    if(pivotCommand_!=nullptr){
        pivotCommand_->Reset();
        delete pivotCommand_;
        pivotCommand_ = nullptr;
    }

    // turn off light
    robot_->SetLight(false);
    isDone_ = true;
}

// destructor, call reset just in case
AlignTapeCommand::~AlignTapeCommand(){
    Reset();
}