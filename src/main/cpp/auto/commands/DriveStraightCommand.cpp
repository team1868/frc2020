/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/DriveStraightCommand.h"
#include <frc/WPILib.h>

/** 
  * Constuctor with slow option
  * @param navXSource a NavXPIDSource
  * @param talonEncoderSource a TalonEncoderPIDSource
  * @param distancePIDOutput an AnglePIDOutput
  * @param distancePIDOutput a DistancePIDOutput
  * @param robot a RobotModel
  * @param desiredDistance a double that refers to the desired distance to travel
  * @param slow is true if robot should move slowly
  */
DriveStraightCommand::DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
		double desiredDistance, bool slow) : AutoCommand(),
		driveStraightLayout_(robot->GetFunctionalityTab().GetLayout("DriveStraight", "List Layout"))
		{

	// initializing variables		
	slow_ = slow;
	isAbsoluteAngle_ = false;
	slowSpeed_ = 0.22; //0.3 for practice bot

	// initialize dependencies
	Initializations(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot, desiredDistance);
	
	leftStraightEntry_ = driveStraightLayout_.Add("Left Output", 0.0).GetEntry();
	rightStraightEntry_ = driveStraightLayout_.Add("Right Output", 0.0).GetEntry();
	desiredAngleEntry_ = driveStraightLayout_.Add("Desired Angle", 0.0).GetEntry();
	desiredTotalFeetEntry_ = driveStraightLayout_.Add("Desired Total Feet", 0.0).GetEntry();
	angleErrorEntry_ = driveStraightLayout_.Add("Angle Error", 0.0).GetEntry();
	encoderErrorEntry_ = driveStraightLayout_.Add("Encoder Error", 0.0).GetEntry();
	aPIDOutputEntry_ = driveStraightLayout_.Add("Angle PID Output", 0.0).GetEntry();
	dPIDOutputEntry_ = driveStraightLayout_.Add("Distance PID Output", 0.0).GetEntry();
	
}

/** 
  * Constructor with absolute angle option
  * @param navXSource a NavXPIDSource
  * @param talonEncoderSource a TalonEncoderPIDSource
  * @param anglePIDOutput an AnglePIDOutput
  * @param distancePIDOutput a DistancePIDOutput
  * @param robot a RobotModel
  * @param desiredDistance a double that refers to the desired distance to travel
  * @param slow is true if robot should move slowly
  * @param absoluteAngle a double that refers to the current absolute angle (keep the same angle while driving straight)
  */
DriveStraightCommand::DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
		double desiredDistance, bool slow, double absoluteAngle) :
		driveStraightLayout_(robot->GetFunctionalityTab().GetLayout("DriveStraight", "List Layout"))
		{
		
	// initializing variablese
	slow_ = slow;
	isAbsoluteAngle_ = true;
	slowSpeed_ = 0.22; // 0.3 for the practice bot
	Initializations(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot, desiredDistance);
	desiredAngle_ = absoluteAngle;

	// initialize dependencies
	Initializations(navXSource, talonEncoderSource, anglePIDOutput, distancePIDOutput, robot, desiredDistance);
	
	leftStraightEntry_ = driveStraightLayout_.Add("Left Output", 0.0).GetEntry();
	rightStraightEntry_ = driveStraightLayout_.Add("Right Output", 0.0).GetEntry();
	desiredAngleEntry_ = driveStraightLayout_.Add("Desired Angle", 0.0).GetEntry();
	desiredTotalFeetEntry_ = driveStraightLayout_.Add("Desired Total Feet", 0.0).GetEntry();
	angleErrorEntry_ = driveStraightLayout_.Add("Angle Error", 0.0).GetEntry();
	encoderErrorEntry_ = driveStraightLayout_.Add("Encoder Error", 0.0).GetEntry();
	aPIDOutputEntry_ = driveStraightLayout_.Add("Angle PID Output", 0.0).GetEntry();
	dPIDOutputEntry_ = driveStraightLayout_.Add("Distance PID Output", 0.0).GetEntry();
}

/**
 * Initializes class for run
 */
void DriveStraightCommand::Init() {
	printf("In drivestraight init\n");

	isDone_ = false;
	robot_->ResetDriveEncoders();  

	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;

	// Setting up PID
	GetPIDValues();

	anglePID_ = new frc::PIDController(rPFac_, rIFac_, rDFac_, navXSource_, anglePIDOutput_);
	distancePID_ = new frc::PIDController(dPFac_, dIFac_, dDFac_, talonEncoderSource_, distancePIDOutput_);

	desiredAngle_ = robot_->GetLastPivotAngle(); // keep the same desired angle

	// initialize dependencies settings
	initialAvgDistance_ = talonEncoderSource_->PIDGet();
	desiredTotalAvgDistance_ = initialAvgDistance_ + desiredDistance_;

	// configure PID
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

	initialDriveTime_ = robot_->GetTime(); // for timeout

	numTimesOnTarget_ = 0; // for double checking if is finished

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

/**
 * Updates current variable values and shuffleboard
 */ 
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

	//set max pid power output for distance
	diffDriveTime_ = robot_->GetTime() - initialDriveTime_;
	if (diffDriveTime_ > maxT_){
		dMaxOutput_ = finalDMax_;
	} else {
		dMaxOutput_ = initialDMax_ + ((finalDMax_-initialDMax_)/maxT_)*diffDriveTime_; // average
	}
	distancePID_->SetOutputRange(-dMaxOutput_, dMaxOutput_);
	

	// check on target
	if (distancePID_->OnTarget() && fabs(talonEncoderSource_->PIDGet() - lastDistance_) < 0.04 ) {
		numTimesOnTarget_++;
		printf("times on target at %d \n", numTimesOnTarget_);
		printf("%f Drivestraight error: %f\n", robot_->GetTime(), distancePID_->GetError());
	} else {
		numTimesOnTarget_ = 0;
	}

    // check error from collision
	if ((fabs(distancePID_->GetError()) < 1.0) && (robot_->CollisionDetected())) { // not working
		numTimesStopped_++;
		printf("%f Collision Detected \n", robot_->GetTime());
	} else {
		numTimesStopped_ = 0;
	}

	lastDistance_ = talonEncoderSource_->PIDGet();

	if(numTimesOnTarget_ > 5) { // on target for 5 iterations, done
		printf("diff time: %fs Final Left Distance: %fft\n" // encoder values not distances
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
	} else { // not done, keep going
		// receive PID outputs
		double dOutput = distancePIDOutput_->GetPIDOutput();
		double rOutput = anglePIDOutput_->GetPIDOutput();

		// shuffleboard update
		aPIDOutputEntry_.SetDouble(rOutput);
		dPIDOutputEntry_.SetDouble(dOutput);

		if (dOutput - lastDOutput_ > 0.5) { // cannot accelerate by more than 0.5 power in one iteration
			dOutput = lastDOutput_ + 0.5;

		}

		// set drive outputs for arcade drive
		rightMotorOutput_ = dOutput - rOutput;
		leftMotorOutput_ = dOutput + rOutput;
		lastDOutput_ = dOutput;

	}
    
	//slow down driving for tippy robots
	if(slow_){
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

	// set drive motors
	robot_->SetDriveValues(RobotModel::Wheels::kLeftWheels, leftMotorOutput_);
	robot_->SetDriveValues(RobotModel::Wheels::kRightWheels, rightMotorOutput_);
}

/**
 * Returns true if done (repeatedly on target)
 * @returns isDone_
 */ 
bool DriveStraightCommand::IsDone() {
	return isDone_;
}

/**
 *  Resets robot to standby
 */ 
void DriveStraightCommand::Reset() {
	// turn off motors
	robot_->SetDriveValues(0.0, 0.0);

	// destroy angle PID
	if (anglePID_ != nullptr) {
		anglePID_->Disable();
		delete anglePID_;
		anglePID_ = nullptr;
		printf("Reset Angle PID %f \n", robot_->GetNavXYaw());
	}

	// destroy distance PID
	if (distancePID_ != nullptr) {
		distancePID_->Disable();
		delete distancePID_;
		distancePID_ = nullptr;
	}

	isDone_ = true;
}
 
/**
 * Gets pid values from shuffleboard
 */
void DriveStraightCommand::GetPIDValues() { // Init values are refreshed at the start of auto

	dPFac_ = robot_-> GetDriveStraightDistanceP();
	dIFac_ = robot_-> GetDriveStraightDistanceI();
	dDFac_ = robot_-> GetDriveStraightDistanceD();

	rPFac_ = robot_-> GetDriveStraightAngleP();
	rIFac_ = robot_-> GetDriveStraightAngleI();
	rDFac_ = robot_-> GetDriveStraightAngleD();

	printf("drivestraight command PID values distance p: %f, i: %f, d: %f\n", dPFac_, dIFac_, dDFac_);
	printf("drivestraight command PID values angle p: %f, i: %f, d: %f\n", rPFac_, rIFac_, rDFac_);
}

/**
 * Initializes dependencies
 * @param navXSource a NavXPIDSource
 * @param talonEncoderSource a TalonEncoderPIDSource
 * @param anglePIDoutput an AnglePIDOutput
 * @param distancePIDOutput a DistancePIDOutput
 * @param robot a RobotModel
 * @param desiredDistance a double that refers to the desired distance to travel
 */
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

	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;
	isDone_ = false;
	initialDriveTime_ = robot_->GetTime();
	diffDriveTime_ = robot_->GetTime() - initialDriveTime_;

	// setting up the PID controllers to NULL
	GetPIDValues();
	anglePID_ = nullptr;
	distancePID_ = nullptr;

	rTolerance_ = 0.1; //was 0.5
	dTolerance_ = 0.375; //was 4.0/12.0

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

/**
 * Destructor
 */
DriveStraightCommand::~DriveStraightCommand() {
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