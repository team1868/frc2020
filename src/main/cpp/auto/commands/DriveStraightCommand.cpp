/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/DriveStraightCommand.h"
#include <frc/WPILib.h>

// constructing
DriveStraightCommand::DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
		double desiredDistance, bool slow) : AutoCommand(),
		driveStraightLayout_(robot->GetFunctionalityTab().GetLayout("DriveStraight", "List Layout"))
		{
	slow_ = slow;
	isAbsoluteAngle_ = false;
	slowSpeed_ = 0.3; //0.3 for practice bot

	// initialize dependencies
	Initializations(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot, desiredDistance);
	
	leftStraightEntry_ = driveStraightLayout_.Add("Left Output", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
	rightStraightEntry_ = driveStraightLayout_.Add("Right Output", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
	desiredAngleEntry_ = driveStraightLayout_.Add("Desired Angle", 0.0).GetEntry();
	desiredTotalFeetEntry_ = driveStraightLayout_.Add("Desired Total Feet", 0.0).GetEntry();
	angleErrorEntry_ = driveStraightLayout_.Add("Angle Error", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
	encoderErrorEntry_ = driveStraightLayout_.Add("Encoder Error", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
	aPIDOutputEntry_ = driveStraightLayout_.Add("Angle PID Output", 0.0).GetEntry();
	dPIDOutputEntry_ = driveStraightLayout_.Add("Distance PID Output", 0.0).GetEntry();
	
	
}

// constructor
DriveStraightCommand::DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
		double desiredDistance, double absoluteAngle) :
		driveStraightLayout_(robot->GetFunctionalityTab().GetLayout("DriveStraight", "List Layout"))
		{
	isAbsoluteAngle_ = true;
	Initializations(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot, desiredDistance);
	desiredAngle_ = absoluteAngle;

	//NOTE: adding repetitive title, this may be an issue later
}

// initialize class for run
void DriveStraightCommand::Init() {
	printf("IN DRIVESTRAIGHT INIT\n");
	isDone_ = false;


	robot_->ResetDriveEncoders();  


	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;

	GetPIDValues();
	// Setting up PID vals
	anglePID_ = new frc::PIDController(rPFac_, rIFac_, rDFac_, navXSource_, anglePIDOutput_);
	distancePID_ = new frc::PIDController(dPFac_, dIFac_, dDFac_, talonEncoderSource_, distancePIDOutput_);

	// absolute angle
	if (!isAbsoluteAngle_) {
		desiredAngle_ = navXSource_->PIDGet();
	}

	// initialize dependencies settings
	initialAvgDistance_ = talonEncoderSource_->PIDGet();
	desiredTotalAvgDistance_ = initialAvgDistance_ + desiredDistance_;

	anglePID_->SetPID(rPFac_, rIFac_, rDFac_);
	distancePID_->SetPID(dPFac_, dIFac_, dDFac_);

	anglePID_->SetSetpoint(desiredAngle_);
	distancePID_->SetSetpoint(desiredTotalAvgDistance_);

	anglePID_->SetContinuous(true);
	anglePID_->SetInputRange(-180.0, 180.0);
	distancePID_->SetContinuous(false); 

	anglePID_->SetOutputRange(-rMaxOutput_, rMaxOutput_);
	distancePID_->SetOutputRange(-initialDMax_, initialDMax_);
	
	
	anglePID_->SetAbsoluteTolerance(rTolerance_);
	distancePID_->SetAbsoluteTolerance(dTolerance_);

	anglePID_->Enable();
	distancePID_->Enable();


	initialDriveTime_ = robot_->GetTime();


	numTimesOnTarget_ = 0;

	lastDistance_ = talonEncoderSource_->PIDGet();
	lastDOutput_ = 0.0;
	printf("Initial Right Distance: %f\n "
			"Initial Left Distance: %f\n"
			"Initial Average Distance: %f\n"
			"Desired Distance: %f\n"
			"Desired Angle: %f\n"
			"Initial getPID(): %f\n"
			"Initial angle: %f \n"
			"Distance error: %f\n"
			"Angle error: %f \n",
			robot_->GetRightDistance(), robot_->GetLeftDistance(),
			initialAvgDistance_, desiredTotalAvgDistance_, desiredAngle_,
			talonEncoderSource_->PIDGet(),  navXSource_->PIDGet(),
			distancePID_->GetError(), anglePID_->GetError());
}



// update current values
void DriveStraightCommand::Update(double currTimeSec, double deltaTimeSec) { 
	// update shuffleboard values
	leftStraightEntry_.SetDouble(leftMotorOutput_);
	rightStraightEntry_.SetDouble(rightMotorOutput_);
	angleErrorEntry_.SetDouble(anglePID_->GetError());
	angleErrorGraphEntry_.SetDouble(anglePID_->GetError());
	desiredAngleEntry_.SetDouble(desiredAngle_);
	encoderErrorEntry_.SetDouble(distancePID_->GetError());
	encoderErrorGraphEntry_.SetDouble(distancePID_->GetError());
	desiredTotalFeetEntry_.SetDouble(desiredTotalAvgDistance_);

	diffDriveTime_ = robot_->GetTime() - initialDriveTime_;

	if (diffDriveTime_ > maxT_){
		dMaxOutput_ = finalDMax_;
	}
	else {
		dMaxOutput_ = initialDMax_ + ((finalDMax_-initialDMax_)/maxT_)*diffDriveTime_;
	}
	distancePID_->SetOutputRange(-dMaxOutput_, dMaxOutput_);
	

// on target
	if (distancePID_->OnTarget() && fabs(talonEncoderSource_->PIDGet() - lastDistance_) < 0.04 ) {
		numTimesOnTarget_++;
		printf("times on target at %d \n", numTimesOnTarget_);
		printf("%f Drivestraight error: %f\n", robot_->GetTime(), distancePID_->GetError());
	} else {
		numTimesOnTarget_ = 0;
	}

    //  error check
	if ((fabs(distancePID_->GetError()) < 1.0) && (robot_->CollisionDetected())) { // not working
		numTimesStopped_++;
		printf("%f Collision Detected \n", robot_->GetTime());
	} else {
		numTimesStopped_ = 0;
	}

	lastDistance_ = talonEncoderSource_->PIDGet();
	if((numTimesOnTarget_ > 5) /*|| (numTimesStopped_ > 0)*/) { //LEAVING AS 10.0 FOR NOW BC WE DON'T KNOW ACTUAL VALUES
		printf("diff time: %fs Final Left Distance: %fft\n" //encoder values not distances
				"Final Right Distance: %fft\n"
				"Final Average Distance: %fft\n"
				"Final Drivestraight error: %fft\n",
				diffDriveTime_, robot_->GetLeftDistance(), robot_->GetRightDistance(),
				talonEncoderSource_->PIDGet(), distancePID_->GetError());
		printf("on target: %d\n", numTimesOnTarget_);
		Reset();

		leftMotorOutput_ = 0.0;
		rightMotorOutput_ = 0.0;

		isDone_ = true;
	} else { // else run motor
		// receive PID outputs
		double dOutput = distancePIDOutput_->GetPIDOutput();
		double rOutput = anglePIDOutput_->GetPIDOutput();

		// shuffleboard update
		aPIDOutputEntry_.SetDouble(rOutput);
		dPIDOutputEntry_.SetDouble(dOutput);

		if (dOutput - lastDOutput_ > 0.5) { // only when accelerating forward
			dOutput = lastDOutput_ + 0.5; //0.4 for KOP

		}
		// set drive outputs
		rightMotorOutput_ = dOutput - rOutput; //sketch, check!
		leftMotorOutput_ = dOutput + rOutput; //TODO sketch check! TODODODODODO SKETCH MAKE SURE NOT OVER 1.0
		lastDOutput_ = dOutput;

//		double maxOutput = fmax(fabs(rightMotorOutput_), fabs(leftMotorOutput_));
	}
    
	if(slow_){
		//TODO fix to ratio rather than hard sets
		if(leftMotorOutput_ > slowSpeed_){
			leftMotorOutput_ = slowSpeed_;
		} else if(leftMotorOutput_ < -slowSpeed_){
			leftMotorOutput_ = -slowSpeed_;
		}
		if(rightMotorOutput_ > slowSpeed_){
			rightMotorOutput_ = slowSpeed_;
		} else if(rightMotorOutput_ < -slowSpeed_){
			rightMotorOutput_ = -slowSpeed_;
		}
	}
	// drive motors
	robot_->SetDriveValues(RobotModel::Wheels::kLeftWheels, leftMotorOutput_);
	robot_->SetDriveValues(RobotModel::Wheels::kRightWheels, rightMotorOutput_);
	//printf("times on target at %f \n\n", numTimesOnTarget_);
}

// repeatedly on target
bool DriveStraightCommand::IsDone() {
	return isDone_;
}

// reset robot to standby
void DriveStraightCommand::Reset() {
	// turn off motors
	robot_->SetDriveValues(0.0, 0.0);

	// destroy angle PID
	if (anglePID_ != NULL) {
		anglePID_->Disable();

		delete anglePID_;

		anglePID_ = NULL;

		printf("Reset Angle PID %f \n", robot_->GetNavXYaw());
	}

	// destroy distance PID
	if (distancePID_ != NULL) {
		distancePID_->Disable();

		delete distancePID_;

		distancePID_ = NULL;
//		printf("Reset Distance PID");

	}
	isDone_ = true;
}
 
//Get pid values from shuffleboard
void DriveStraightCommand::GetPIDValues() { // Ini values are refreshed at the start of auto

	dPFac_ = robot_-> GetDriveStraightDistanceP();
	dIFac_ = robot_-> GetDriveStraightDistanceI();
	dDFac_ = robot_-> GetDriveStraightDistanceD();

	rPFac_ = robot_-> GetDriveStraightAngleP();
	rIFac_ = robot_-> GetDriveStraightAngleI();
	rDFac_ = robot_-> GetDriveStraightAngleD();

	printf("DRIVESTRAIGHT COMMAND DRIVE p: %f, i: %f, d: %f\n", dPFac_, dIFac_, dDFac_);
	printf("DRIVESTRAIGHT COMMAND ANGLE p: %f, i: %f, d: %f\n", rPFac_, rIFac_, rDFac_);
}

// initialize dependencies
void DriveStraightCommand::Initializations(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
			AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
			double desiredDistance) {
	robot_ = robot;

	navXSource_ = navXSource;
	talonEncoderSource_ = talonEncoderSource;

	anglePIDOutput_ = anglePIDOutput;
	distancePIDOutput_ = distancePIDOutput;

	desiredAngle_ = navXSource_->PIDGet();
	initialAvgDistance_ = talonEncoderSource_->PIDGet();

	desiredDistance_ = desiredDistance;
	//desiredTotalAvgDistance_ = 2.0; //TODO CHANGE //initialAvgDistance_ + desiredDistance_;
	//printf("Total desired distance is: %f", desiredTotalAvgDistance_);

	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;
	isDone_ = false;
	initialDriveTime_ = robot_->GetTime();
	diffDriveTime_ = robot_->GetTime() - initialDriveTime_;

	// Setting up the PID controllers to NULL
	GetPIDValues();
	anglePID_ = NULL;
	distancePID_ = NULL;

	rTolerance_ = 0.5;
	dTolerance_ = 2.0 / 12.0;

	rMaxOutput_ = 0.15;
	dMaxOutput_ = 0.1;
	initialDMax_ = 0.1;
	finalDMax_ = 0.85;
	maxT_ = 1.0;

	// initializing number of times robot is on target
	numTimesOnTarget_ = 0;
	numTimesStopped_ = 0;

	lastDistance_ = talonEncoderSource_->PIDGet();
	lastDOutput_ = 0.0;
}


DriveStraightCommand::~DriveStraightCommand() {
	//leftStraightEntry_->Remove();
	/*
	anglePID_->Disable();
	distancePID_->Disable();
	anglePID_->~PIDController();
	distancePID_->~PIDController();
	*/
	Reset();

	leftStraightEntry_.Delete();
	rightStraightEntry_.Delete();
	angleErrorEntry_.Delete();
	angleErrorGraphEntry_.Delete();
	desiredAngleEntry_.Delete();
	encoderErrorEntry_.Delete();
	encoderErrorGraphEntry_.Delete();
	desiredTotalFeetEntry_.Delete();
	dPIDOutputEntry_.Delete();
	aPIDOutputEntry_.Delete();
}