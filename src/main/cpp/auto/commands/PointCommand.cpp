/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "auto/commands/PointCommand.h"
#include <frc/WPILib.h>

/**
 * Constructor
 * @param robot a RobotModel
 * @param desiredAngle a double that is the angle of the turn
 * @param isAbsoluteAngle a bool that represents whether the angle is absolute position of deta angle
 * @param navXSource a NavXPIDSource
 * @param turnLeft a bool that represents what direction the robot should turn in. Is true if turning left
 * @param talonOutput a PivotPIDTalonOutput
 */
PointCommand::PointCommand(RobotModel *robot, double desiredAngle, bool isAbsoluteAngle, 
	NavXPIDSource* navXSource, bool turnLeft, PivotPIDTalonOutput* talonOutput) :
	pointLayout_(robot->GetFunctionalityTab().GetLayout("Point", "List Layout")) 
	{

	leftDriveEntry_ = pointLayout_.Add("Left Drive Output", 0.0).GetEntry();
	rightDriveEntry_ = pointLayout_.Add("Right Drive Output", 0.0).GetEntry();
	pointErrorEntry_ = pointLayout_.Add("Error", 0.0).GetEntry();

    navXSource_ = navXSource;

	initYaw_ = navXSource_->PIDGet();

	turnLeft_ = turnLeft;

	// adjust angle is absolute
	if (isAbsoluteAngle){
		desiredAngle_ = desiredAngle;
	} else {
		desiredAngle_ = initYaw_ + desiredAngle;
		if (desiredAngle_ > 180) {
			desiredAngle_ -= 360; 
		} else if (desiredAngle_ < -179) {
 			desiredAngle_ += 360;
 		}
	}

	// initialize variables
	isDone_ = false;
	robot_ = robot;
	 
	// initialize PID talon output
	talonOutput_ = talonOutput;

	// initialize time variables
	pointCommandStartTime_ =  robot_->GetTime();
	pointTimeoutSec_ = 5.0; //note edited from last year

	// retrieve pid values from user 
	// moved to shuffleboard model
	pFac_ = robot_->GetPointP();
	iFac_ = robot_->GetPointI();
	dFac_ = robot_->GetPointD();

	printf("p: %f i: %f d: %f and going to %f\n", pFac_, iFac_, dFac_, desiredAngle_);
	pointPID_ = new frc::PIDController(pFac_, iFac_, dFac_, navXSource_, talonOutput_);

	maxOutput_ = 0.9;
	tolerance_ = 3.0;

	numTimesOnTarget_ = 0;

}

/**
 * Constructor with tolerance
 * @param robot a RobotModel
 * @param desiredAngle a double that is the angle of the turn
 * @param isAbsoluteAngle a bool that represents whether the angle is absolute position of deta angle
 * @param navXSource a NavXPIDSource
 * @param tolerance a double
 * @param turnLeft a bool that represents what direction the robot should turn in
 * @param talonOutput a PivotPIDTalonOutput
 */
PointCommand::PointCommand(RobotModel *robot, double desiredAngle, bool isAbsoluteAngle, 
	NavXPIDSource* navXSource, double tolerance, bool turnLeft, PivotPIDTalonOutput* talonOutput) :
	pointLayout_(robot->GetFunctionalityTab().GetLayout("Point", "List Layout"))
	{

	leftDriveEntry_ = pointLayout_.Add("Point Left Drive", 0.0).GetEntry();
	rightDriveEntry_ = pointLayout_.Add("Point Right Drive", 0.0).GetEntry();
	pointErrorEntry_ = pointLayout_.Add("Point Error", 0.0).GetEntry();

	navXSource_ = navXSource;

	initYaw_ = navXSource_->PIDGet();

	turnLeft_ = turnLeft;

	// adjust - angle is absolute
	if (isAbsoluteAngle){
		desiredAngle_ = desiredAngle;
	} else {
		desiredAngle_ = initYaw_ + desiredAngle;
		if (desiredAngle_ > 180) {
			desiredAngle_ -= 360; 
		} else if (desiredAngle_ < -179) {
 			desiredAngle_ += 360;
 		}
	}

	// initialize variables
	isDone_ = false;
	robot_ = robot;
	
	// initialize PID talon output
	talonOutput_ = talonOutput;

	// initialize time variables
	pointCommandStartTime_ = robot_->GetTime();
	pointTimeoutSec_ = 5.0; //note edited from last year

	pFac_ = robot_->GetPointP();
	iFac_ = robot_->GetPointI();
	dFac_ = robot_->GetPointD();

	pointPID_ = new frc::PIDController(pFac_, iFac_, dFac_, navXSource_, talonOutput_);

	maxOutput_ = 0.9;
	tolerance_ = tolerance;

	numTimesOnTarget_ = 0;

}

/**
 * Gets PID values from init file, sets to 0 if not present
 */
void PointCommand::GetPIDValues() {
	pFac_ = robot_-> GetPointP();
	iFac_ = robot_-> GetPointI();
	dFac_ = robot_-> GetPointD();
}

/**
 * Initializes command
 */
void PointCommand::Init() {

	// Setting PID values (in case they changed)
	GetPIDValues();
	pointPID_->SetPID(pFac_, iFac_, dFac_);

	// initialize NavX angle
	initYaw_ = navXSource_->PIDGet();

	// set settings for PID
	pointPID_->SetSetpoint(desiredAngle_);
	pointPID_->SetContinuous(true);
	pointPID_->SetInputRange(-180, 180);
	pointPID_->SetOutputRange(-maxOutput_, maxOutput_); 
	pointPID_->SetAbsoluteTolerance(tolerance_);
	pointPID_->Enable();

	// target variables
	isDone_ = false;
	numTimesOnTarget_ = 0;
	pointCommandStartTime_ = robot_->GetTime();
	actualTimeoutSec_ = fabs(pointPID_->GetError()) * pointTimeoutSec_ / 90.0;

	printf("Initial NavX Angle: %f\n"
			"Desired NavX Angle: %f\n"
			"Chicken tenders point time starts at %f\n",
			initYaw_, desiredAngle_, pointCommandStartTime_);
}

/**
 * Resets for standby
 */
void PointCommand::Reset() {
	// turn off motors
	robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);

	// reset shuffleboard values
	leftDriveEntry_.SetDouble(0.0);
	rightDriveEntry_.SetDouble(0.0);

	// disable PID
	if (pointPID_ != nullptr) {
		pointPID_->Disable();
		delete pointPID_;
		pointPID_ = nullptr;
		printf("Disabling pointcommand %f \n", robot_->GetNavXYaw());

	}
	isDone_ = true;

	printf("DONE FROM RESET \n");
}

/**
 * Periodic update
 * @param currTimeSec a double
 * @param deltaTimeSec a double
 */
void PointCommand::Update(double currTimeSec, double deltaTimeSec) {

	// calculate time difference
	double timeDiff = robot_->GetTime() - pointCommandStartTime_;

	// check for timeout
	bool timeOut = (timeDiff > pointTimeoutSec_);

	// on target
	if (pointPID_->OnTarget()) {
		numTimesOnTarget_++;
	} else {
		numTimesOnTarget_ = 0;
	}

	// number of times on target exceeds 8 or timed out
	if ((pointPID_->OnTarget() && numTimesOnTarget_ > 8) || timeOut){
		printf("diffTime: %f Final NavX Angle from PID Source: %f\n"
				"Final NavX Angle from robot: %f \n" 
				"%f Angle NavX Error %f\n",
				timeDiff, navXSource_->PIDGet(), robot_->GetNavXYaw(), robot_->GetTime(),
					pointPID_->GetError());
		
		Reset();
		robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
		
		printf("%f POINT IS DONE \n", robot_->GetTime());
		if (timeOut) {
			printf("%f TIME OUT @ %f\n", robot_->GetTime(), timeDiff);
		}

	} else { 
		// command is not done
		
		double output = talonOutput_->GetOutput();
		output *= 0.5;

		// adjust motor values according to PID
		printf("ERROR IS %f\n", pointPID_->GetError());
        if (turnLeft_) {
            // turning left, set left wheel to stationary
            robot_->SetDriveValues(RobotModel::kLeftWheels, 0.0);
		    robot_->SetDriveValues(RobotModel::kRightWheels, -output);

        } else {
            // turning right, set right wheel to stationary
            robot_->SetDriveValues(RobotModel::kLeftWheels, output);
	    	robot_->SetDriveValues(RobotModel::kRightWheels, 0.0);

        }
		
		//robot_->SetDriveValues(output, output); //NOTE: NO STATIC FRICTION

		// update shuffleboard
        if (turnLeft_){
            rightDriveEntry_.SetDouble(output);
            leftDriveEntry_.SetDouble(0.0);
        } else {
            rightDriveEntry_.SetDouble(0.0);
            leftDriveEntry_.SetDouble(output);
        }
		
		pointErrorEntry_.SetDouble(pointPID_->GetError());

	}
}

/**
 * Returns if command is done
 * @return isDone_
 */
bool PointCommand::IsDone() {
	return isDone_;
}

/**
 * Destructor
 */
PointCommand::~PointCommand() {
	Reset();
	leftDriveEntry_.Delete();
	rightDriveEntry_.Delete();
	pointErrorEntry_.Delete();
}