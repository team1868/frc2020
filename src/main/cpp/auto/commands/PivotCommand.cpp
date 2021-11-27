/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/PivotCommand.h"
#include <frc/WPILib.h>

/** 
 * Constructor without tolerance
 * @param robot a RobotModel
 * @param desiredAngle a double that is the angle of the turn
 * @param isAbsoluteAngle a bool that represents whether the angle is absolute position of delta angle
 * @param navXSource a NavXPIDSource
 * @param talonOutput a PivotPIDTalonOutput
 */
PivotCommand::PivotCommand(RobotModel *robot, double desiredAngle, bool isAbsoluteAngle,
	NavXPIDSource* navXSource, PivotPIDTalonOutput* talonOutput) :
	pivotLayout_(robot->GetFunctionalityTab().GetLayout("Pivot", "List Layout")) 
	{

	// add left/right outputs and errors to the shuffleboard layout
	leftDriveEntry_ = pivotLayout_.Add("Left Drive Output", 0.0).GetEntry();
	rightDriveEntry_ = pivotLayout_.Add("Right Drive Output", 0.0).GetEntry();
	pivotErrorEntry_ = pivotLayout_.Add("Error", 0.0).GetEntry();

    navXSource_ = navXSource;

	initYaw_ = navXSource_->PIDGet();

	// adjust desiredAngle value based on whether the angle is absolute
	if (isAbsoluteAngle){
		desiredAngle_ = desiredAngle;
	} else {
		desiredAngle_ = initYaw_ + desiredAngle;
		if (desiredAngle_ > 180) {
			desiredAngle_ -= 360;
		} else if (desiredAngle_ < -180) {
			desiredAngle_ += 360;
		}
	}

	// initialize variables
	isDone_ = false;
	robot_ = robot;
	
	// initialize PID talon output
	talonOutput_ = talonOutput;

	// initialize time variables
	pivotCommandStartTime_ = robot_->GetTime();
	pivotTimeoutSec_ = 5.0; //note edited from last year

	maxOutput_ = 0.9;
	tolerance_ = 4.0;
	numTimesOnTarget_ = 0;

	// retrieve pid values from user //moved to shuffleboard model
	pFac_ = robot_->GetPivotP();
	iFac_ = robot_->GetPivotI();
	dFac_ = robot_->GetPivotD();

	//set up PID + print
	printf("p: %f i: %f d: %f and going to %f\n", pFac_, iFac_, dFac_, desiredAngle_);
	pivotPID_ = new frc::PIDController(pFac_, iFac_, dFac_, navXSource_, talonOutput_);

}

/** 
 * Constructor with tolerance
 * @param robot a RobotModel
 * @param desiredAngle a double that is the angle of the turn
 * @param isAbsoluteAngle a bool that represents whether the angle is absolute position of deta angle
 * @param navXSource a NavXPIDSource
 * @param tolerance a double
 * @param talonOutput a PivotPIDTalonOutput
 */
PivotCommand::PivotCommand(RobotModel *robot, double desiredAngle, bool isAbsoluteAngle, 
	NavXPIDSource* navXSource, int tolerance, PivotPIDTalonOutput* talonOutput) :
	pivotLayout_(robot->GetFunctionalityTab().GetLayout("Pivot", "List Layout")) 
	{

	leftDriveEntry_ = pivotLayout_.Add("Pivot Left Drive", 0.0).GetEntry();
	rightDriveEntry_ = pivotLayout_.Add("Pivot Right Drive", 0.0).GetEntry();
	pivotErrorEntry_ = pivotLayout_.Add("Pivot Error", 0.0).GetEntry();

	navXSource_ = navXSource;

	initYaw_ = navXSource_->PIDGet();

	// adjust desiredAngle value based on whether the angle is absolute
	if (isAbsoluteAngle){
		desiredAngle_ = desiredAngle;
	} else {
		desiredAngle_ = initYaw_ + desiredAngle;
		if (desiredAngle_ > 180) {
			desiredAngle_ -= -360;
		} else if (desiredAngle_ < -180) {
			desiredAngle_ += 360;
		}
	}

	// initialize variables
	isDone_ = false;
	robot_ = robot;
	
	// initialize PID talon output
	talonOutput_ = talonOutput;

	// initialize time variables
	pivotCommandStartTime_ = robot_->GetTime();
	pivotTimeoutSec_ = 5.0;//0.0; //note edited from last year

	//retrieve pid values from user
	pFac_ = robot_->GetPivotP();
	iFac_ = robot_->GetPivotI();
	dFac_ = robot_->GetPivotD();

	pivotPID_ = new frc::PIDController(pFac_, iFac_, dFac_, navXSource_, talonOutput_);

	maxOutput_ = 0.9;
	tolerance_ = tolerance;//3.0;

	numTimesOnTarget_ = 0;

}

/** 
 * Gets PID values from Shuffleboard, sets to 0 if not present
 */
void PivotCommand::GetPIDValues() {
	pFac_ = robot_-> GetPivotP();
	iFac_ = robot_-> GetPivotI();
	dFac_ = robot_-> GetPivotD();
}

/**
 * Initializes command
 */
void PivotCommand::Init() {
	robot_->SetLastPivotAngle(desiredAngle_);

	GetPIDValues();
	pivotPID_->SetPID(pFac_, iFac_, dFac_);

	// initliaze NavX angle
	initYaw_ = navXSource_->PIDGet();

	// set settings for PID
	pivotPID_->SetSetpoint(desiredAngle_);
	pivotPID_->SetContinuous(true);
	pivotPID_->SetInputRange(-180, 180);
	pivotPID_->SetOutputRange(-maxOutput_, maxOutput_);
	pivotPID_->SetAbsoluteTolerance(tolerance_);
 	pivotPID_->Enable();

	// target variables
	isDone_ = false;
	numTimesOnTarget_ = 0;
	pivotCommandStartTime_ = robot_->GetTime();

	printf("Initial NavX Angle: %f\n"
			"Desired NavX Angle: %f\n"
			"Pivot time starts at %f\n",
 			initYaw_, desiredAngle_, pivotCommandStartTime_);
}

/**
 * Resets for standby
 */
void PivotCommand::Reset() {
	// turn off motors
	robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);

	// reset shuffleboard values
	leftDriveEntry_.SetDouble(0.0);
	rightDriveEntry_.SetDouble(0.0);

	// disable PID
	if (pivotPID_ != NULL) {
		pivotPID_->Disable();
		delete pivotPID_;
		pivotPID_ = NULL;
		printf("Disabling pivotcommand %f \n", robot_->GetNavXYaw());

	}
	isDone_ = true;

	printf("DONE FROM RESET \n");
}

/** 
 * Periodic update
 * @param currTimeSec a double
 * @param deltaTimeSec a double
 */
void PivotCommand::Update(double currTimeSec, double deltaTimeSec) {

	// calculate time difference
	double timeDiff = robot_->GetTime() - pivotCommandStartTime_;
	bool timeOut = (timeDiff > pivotTimeoutSec_);

	// check if on target
	if (pivotPID_->OnTarget()) {
		numTimesOnTarget_++;
	} else {
		numTimesOnTarget_ = 0;
	}

	// check how many times on target and if timed out
	if ((pivotPID_->OnTarget() && numTimesOnTarget_ > 8) || timeOut){
		
		printf("diffTime: %f Final NavX Angle from PID Source: %f\n"
				"Final NavX Angle from robot: %f \n"
				"%f Angle NavX Error %f\n",
				timeDiff, navXSource_->PIDGet(), robot_->GetNavXYaw(), robot_->GetTime(),
				pivotPID_->GetError());
		
		Reset();
		isDone_ = true;
		robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
		
		printf("%f PIVOT IS DONE \n", robot_->GetTime());
		
		if (timeOut) {
			printf("%f FROM PIVOT TIME OUT GO GET CHICKEN TENDERS @ %f\n", robot_->GetTime(), timeDiff);
		}
	
	} else { 
		// not done
		double output = talonOutput_->GetOutput();

		robot_->SetDriveValues(RobotModel::kLeftWheels, output);
		robot_->SetDriveValues(RobotModel::kRightWheels, -output);

		// update shuffleboard
		rightDriveEntry_.SetDouble(output);
		leftDriveEntry_.SetDouble(-output);
		pivotErrorEntry_.SetDouble(pivotPID_->GetError());

	}
}

/** 
 * Returns if command is done
 * @return isDone_
 */
bool PivotCommand::IsDone() {
	return isDone_;
}

/** 
 * Destructor
 */
PivotCommand::~PivotCommand() {
	Reset();
	leftDriveEntry_.Delete();
	rightDriveEntry_.Delete();
	pivotErrorEntry_.Delete();
}