/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/profiling/MotionProfileTest2Command.h"

//USING METERS

MotionProfileTestCommand::MotionProfileTestCommand(
    RobotModel *robot,
    TalonEncoderPIDSource *distanceSource, NavXPIDSource *navXSource,
    DistancePIDOutput *distanceOutput, AnglePIDOutput *angleOutput) {

    robot_ = robot;
    isDone_ = false;

    lastPosition_ = 0.0;
    currPosition_ = 0.0;

    nextDistance_ = 0.0;
    nextAngle_ = 0.0;

    lastTime_ = 0.0;

    distanceSource_ = distanceSource;
    navXSource_ = navXSource;
    angleOutput_ = angleOutput;
    distanceOutput_ = distanceOutput;

    vP_ = 0.8;
    vI_ = 0.0;
    vD_ = 0.2;

    aP_ = 0.031;
    aI_ = 0.0;
    aD_ = 0.017;

    distancePID_ = new PIDController(vP_, vI_, vD_, distanceSource_, distanceOutput_);
    anglePID_ = new PIDController(aP_, aI_, aD_, navXSource_, angleOutput_);

    pathIndex_ = 0;
}

void MotionProfileTestCommand::Reset(){
    //TODO
}

void MotionProfileTestCommand::Init(){
    distancePID_->SetPID(vP_, vI_, vD_);

    CalcNext();

	// set settings for PID
	distancePID_->SetSetpoint(nextDistance_);
	distancePID_->SetContinuous(true);
	distancePID_->SetInputRange(0, 14.0);
	distancePID_->SetOutputRange(-1.0, 1.0);
	distancePID_->SetAbsoluteTolerance(1.0);	 
	distancePID_->Enable();

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

    CalcNext();

    distancePID_->SetSetpoint(nextDistance_);
    anglePID_->SetSetpoint(nextAngle_);

    double dOut = distanceOutput_->GetPIDOutput();
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
    double distance = points[pathIndex_+3];
    if(!isDone_ && pathIndex_+8<pointsSize){
        pathIndex_ += 8;
        nextDistance_ = distance;
        nextAngle_ = points[pathIndex_ + 7]*PI/180.0 + 90.0;
    } else {
        printf("REACHED END OF PATHFINDING ARRAY\n");
        isDone_ = true;
        nextDistance_ = 0.0; //stop running
    }
}

MotionProfileTestCommand::~MotionProfileTestCommand() {

}
