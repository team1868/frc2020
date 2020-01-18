/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/profiling/MotionProfileTestCommand.h"

//USING METERS

MotionProfileTestCommand::MotionProfileTestCommand(
    RobotModel *robot,
    VelocityPIDSource *velocitySource, NavXPIDSource *navXSource,
    VelocityPIDOutput *velocityOutput, AnglePIDOutput *angleOutput) {

    robot_ = robot;
    isDone_ = false;

    lastPosition_ = 0.0;
    currPosition_ = 0.0;

    nextVelocity_ = 0.0;
    nextAngle_ = 0.0;

    lastTime_ = 0.0;

    velocitySource_ = velocitySource;
    navXSource_ = navXSource;
    angleOutput_ = angleOutput;
    velocityOutput_ = velocityOutput;

    vP_ = 0.8;
    vI_ = 0.0;
    vD_ = 0.2;

    aP_ = 0.031;
    aI_ = 0.0;
    aD_ = 0.017;

    velocityPID_ = new PIDController(vP_, vI_, vD_, velocitySource_, velocityOutput_);
    anglePID_ = new PIDController(aP_, aI_, aD_, navXSource_, angleOutput_);

    pathIndex_ = 0;
}

void MotionProfileTestCommand::Reset(){
    //TODO
}

void MotionProfileTestCommand::Init(){
    velocityPID_->SetPID(vP_, vI_, vD_);

    CalcNext();

	// set settings for PID
	velocityPID_->SetSetpoint(nextVelocity_);
	velocityPID_->SetContinuous(true);
	velocityPID_->SetInputRange(0, 14.0);
	velocityPID_->SetOutputRange(-1.0, 1.0);
	velocityPID_->SetAbsoluteTolerance(1.0);	 
	velocityPID_->Enable();

    anglePID_->SetPID(aP_, aI_, aD_);

	// set settings for PID
	anglePID_->SetSetpoint(nextAngle_);
	anglePID_->SetContinuous(true);
	anglePID_->SetInputRange(-180.0, 180.0);
	anglePID_->SetOutputRange(-1.0, 1.0);
	anglePID_->SetAbsoluteTolerance(2.0);	 
	anglePID_->Enable();
}

void MotionProfileTestCommand::Update(double currTimeSec, double deltaTimeSec){
    // lastPosition_ = currPosition_;
    // currPosition_ = (robot_->GetLeftDistance()+robot_->GetRightDistance())/2.0;

    // // double deltaTime = currTime - lastTime_;
    // double deltaPosition = currPosition_-lastPosition_;
    // currVelocity_ = deltaTimeSec/deltaPosition;

    double leftOutput, rightOutput;

    velocitySource_->UpdateVelocity();
    CalcNext();

    velocityPID_->SetSetpoint(nextVelocity_);
    anglePID_->SetSetpoint(nextAngle_);

    double dOut = velocityOutput_->GetPIDOutput();
    double aOut = angleOutput_->GetPIDOutput();

    rightOutput = dOut-aOut;
    leftOutput = dOut+aOut;

    robot_->SetDriveValues(RobotModel::kLeftWheels, leftOutput);
    robot_->SetDriveValues(RobotModel::kRightWheels, rightOutput);
}

bool MotionProfileTestCommand::IsDone(){
    return isDone_;
}

void MotionProfileTestCommand::CalcNext(){
    double velocity = points[pathIndex_+4];
    if(!isDone_ && pathIndex_+8<pointsSize){
        pathIndex_ += 8;
        nextVelocity_ = velocity;
        nextAngle_ = points[pathIndex_ + 7]*PI/180.0 + 90.0;
    } else {
        printf("REACHED END OF PATHFINDING ARRAY\n");
        isDone_ = true;
        nextVelocity_ = 0.0; //stop running
    }
}

MotionProfileTestCommand::~MotionProfileTestCommand() {

}
