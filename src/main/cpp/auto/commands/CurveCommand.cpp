/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../../include/auto/commands/CurveCommand.h"
#include <math.h>
#include <cmath>

/** 
  * Constuctor
  * @param robot a RobotModel
  * @param desiredRadius a double that refers to the desired radius
  * @param desiredAngle a double that refers to the desired angle for the robot to turn
  * @param turnLeft is true if the robot is supposed to turn left
  * @param goForward is true if robot should go forward
  * @param navXSource a NavXPIDSource
  * @param talonEncoderPIDSource a TalonEncoderPIDSource
  * @param anglePIDOutput an AnglePIDOutput
  * @param distancePIDOutput a DistancePIDOutput
  */
CurveCommand::CurveCommand(RobotModel *robot, double desiredRadius, double desiredAngle, bool turnLeft,
  bool goForward, NavXPIDSource* navXSource, TalonEncoderPIDSource *talonEncoderPIDSource,
	AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput) : AutoCommand(), 
  curveLayout_(robot->GetFunctionalityTab().GetLayout("Curve", "List Layout")) { 
  
  //using relative angle, radius is middle of robot
  
  // initialize class variables
  robot_ = robot;
  desiredRadius_ = desiredRadius;
  desiredAngle_ = desiredAngle;
  turnLeft_ = turnLeft;
  goForward_ = goForward;
  isDone_ = false;

  // forward is positive, backwards is negative
  if (goForward_) {
    direction_ = 1.0;
  } else{
    direction_ = -1.0;
  }
  
  // PID
  navXPIDSource_ = navXSource;
  talonEncoderPIDSource_ = talonEncoderPIDSource;
  anglePIDOutput_ = anglePIDOutput;
  distancePIDOutput_ = distancePIDOutput;

  dMaxOutput_ = 0.1; // motor output set to output range
  initialDMax_ = 0.1; // motor output
  finalDMax_ = 0.9; // motor output
  maxT_ = 1.0; //secs

  // shuffleboard
  dOutputNet_ = curveLayout_.Add("Curve dO", 0.0).GetEntry();
  lOutputNet_ = curveLayout_.Add("Curve lO", 0.0).GetEntry();
  rOutputNet_ = curveLayout_.Add("Curve rO", 0.0).GetEntry();
  dErrorNet_ = curveLayout_.Add("Curve dErr", 0.0).GetEntry();
  pidSourceNet_ = curveLayout_.Add("Curve PID Get", 0.0).GetEntry();

  printf("initialized curve with constructor 1\n");
}

/** 
  * Constuctor
  * @param robot a RobotModel
  * @param desiredRadius a double that refers to the desired radius
  * @param desiredAngle a double that refers to the desired angle for the robot to turn
  * @param navXSource a NavXPIDSource
  * @param talonEncoderPIDSource a TalonEncoderPIDSource
  * @param anglePIDOutput an AnglePIDOutput
  * @param distancePIDOutput a DistancePIDOutput
  */
CurveCommand::CurveCommand(RobotModel *robot, double desiredRadius, double desiredAngle,
  NavXPIDSource* navXSource, TalonEncoderPIDSource *talonEncoderPIDSource,
	AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput) : AutoCommand(), 
  curveLayout_(robot->GetFunctionalityTab().GetLayout("Curve", "List Layout")) 
  { 
  
  //using absolute angle, radius is middle of robot
  
  // initialize class variables
  robot_ = robot;
  desiredRadius_ = desiredRadius;
  turnLeft_ = desiredAngle_ < 0;
  desiredAngle_ = abs(desiredAngle);
  isDone_ = false;

  navXPIDSource_ = navXSource;

  // PID
  talonEncoderPIDSource_ = talonEncoderPIDSource;
  anglePIDOutput_ = anglePIDOutput;
  distancePIDOutput_ = distancePIDOutput;

  // shuffleboard
  dOutputNet_ = curveLayout_.Add("Curve dO", 0.0).GetEntry();
  lOutputNet_ = curveLayout_.Add("Curve lO", 0.0).GetEntry();
  rOutputNet_ = curveLayout_.Add("Curve rO", 0.0).GetEntry();
  dErrorNet_ = curveLayout_.Add("Curve dErr", 0.0).GetEntry();
  pidSourceNet_ = curveLayout_.Add("Curve PID Get", 0.0).GetEntry();

  printf("initialized curve with constructor 2\n");
}

/**
 * Initializes command
 */
void CurveCommand::Init(){

  // get initial angle
  initAngle_ = robot_->GetLastPivotAngle();

  // final angle to turn to
  double finalAngle = initAngle_ - desiredAngle_; //weird polarity
  
  // get the shortest angle to move
  if (finalAngle > 180.0){
    finalAngle -= 360.0;
  } else if (finalAngle < -180){
    finalAngle += 360.0;
  }
  robot_->SetLastPivotAngle(finalAngle);

  // initializing all variables that will be used in the command
  curAngle_ = initAngle_;
  curPivDistance_ = 0.0;
  curDesiredAngle_ = curAngle_;
  initialCurveTime_ = robot_->GetTime();
  diffCurveTime_ = robot_->GetTime() - initialCurveTime_;
  numTimesOnTarget_ = 0;
  curveTimeoutSec_ = fabs(desiredAngle_*desiredRadius_*PI/180.0/3.0)+2.0;
  timeOut = false;
  curAngleError_ = 0.0;

  robot_->ResetDriveEncoders(); 

  GetPIDValues();

  dPID_ = new frc::PIDController(dPFac_, dIFac_, dDFac_, talonEncoderPIDSource_, distancePIDOutput_);
	dPID_->SetPID(dPFac_, dIFac_, dDFac_);
  dPID_->SetSetpoint(direction_*desiredAngle_*desiredRadius_*PI/180.0);
  dPID_->SetAbsoluteTolerance(2.0/12.0);
	dPID_->SetContinuous(false);
	dPID_->SetOutputRange(-initialDMax_, initialDMax_);
  dPID_->Enable();

  printf("done with init in curve\n");
}

/**
 * Disables distance PID if it's null & sets isDone_ to true
 */
void CurveCommand::Reset(){
  // stop moving
  robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);

	// destroy distance PID
	if (dPID_ != nullptr) {
		dPID_->Disable();
		delete dPID_;
		dPID_ = nullptr;
		printf("Reset Distance PID");
	}

	isDone_ = true;
}


/**
  * Updates diffCurveTime_ and calls Reset() method if dPid_ is on target & numTimesOnTarget > 6
  * @param currTImeSec a double
  * @param deltaTimeSec a double
  */
void CurveCommand::Update(double currTimeSec, double deltaTimeSec){
  // get time elapsed since command started
  diffCurveTime_ = robot_->GetTime() - initialCurveTime_;

  pidSourceNet_.SetDouble(talonEncoderPIDSource_->PIDGet());
  
  if (dPID_->OnTarget()) {
    numTimesOnTarget_++;
  } else{
    numTimesOnTarget_ = 0;
  }
  
  // number of times on target is more than 6, then end command
  if (dPID_->OnTarget() && numTimesOnTarget_ > 6) {

    printf("difftime: %fs Original Desired Distance: %fft\n"
            "Final NavX Angle from PID Source: %f\n"
            "Final NavX Angle from robot: %f \n"
            "Final Distance from PID Source: %fft\n",
            diffCurveTime_, direction_*desiredAngle_*desiredRadius_*PI/180, navXPIDSource_->PIDGet(), 
            robot_->GetNavXYaw(), talonEncoderPIDSource_->PIDGet());
    
    if (turnLeft_){
       printf("Final Distance from robot: %f\n", robot_->GetRightDistance()); //fixed inversion
    } else {
      printf("Final Distance from robot: %f\n", robot_->GetLeftDistance());
    }
    
    Reset();
    printf("%f CurveCommand IS DONE \n", robot_->GetTime());

  } else { // otherwise, continue
    curPivDistance_ = talonEncoderPIDSource_->PIDGet();
    curDesiredAngle_ = CalcCurDesiredAngle(curPivDistance_);
    curAngleError_ = curDesiredAngle_ - curAngle_;

    double dOutput = distancePIDOutput_->GetPIDOutput();
    double frictionConstant = 0.05;
    
    if (dOutput < 0) {
      dOutput -= frictionConstant;
    } else {
      dOutput += frictionConstant;
    }

    double lOutput;
    double rOutput;
    double dError;

    if(turnLeft_){
      // turning left, right wheel goes larger distance
      rOutput = dOutput*(desiredRadius_+ROBOT_WIDTH/2)/(desiredRadius_);
      lOutput = dOutput * (desiredRadius_-ROBOT_WIDTH/2)/(desiredRadius_); //WORKS WHEN RADIUS > ROBOT_WIDTH/2
    } else {
      rOutput = dOutput * (desiredRadius_-ROBOT_WIDTH/2)/(desiredRadius_); //WORKS WHEN RADIUS > ROBOT_WIDTH/2
      lOutput = dOutput * (desiredRadius_+ROBOT_WIDTH/2)/(desiredRadius_);
    }

    // make sure I is 0
    // power output checks
    if (lOutput > 1.0){
      rOutput = rOutput/lOutput;
      lOutput = 1.0;
    } else if (lOutput < -1.0){
      rOutput = rOutput/(-lOutput);
      lOutput = -1.0;
    }
    if (rOutput > 1.0) {
      lOutput = lOutput/rOutput;
      rOutput = 1.0;
    } else if (rOutput < -1.0) {
      lOutput = lOutput/(-rOutput);
      rOutput = -1.0;
    }

    lOutput *= 0.8;
    rOutput *= 0.8;
    
    // ramp up max PID output from initial to final
    if (diffCurveTime_ > maxT_){
      dMaxOutput_ = finalDMax_;
    } else {
      dMaxOutput_ = initialDMax_ + ((finalDMax_-initialDMax_)/maxT_)*diffCurveTime_; 
    }
    dPID_->SetOutputRange(-dMaxOutput_, dMaxOutput_);

    // run wheels
    robot_->SetDriveValues(lOutput, rOutput);

    dOutputNet_.SetDouble(dOutput);
    lOutputNet_.SetDouble(lOutput);
    rOutputNet_.SetDouble(rOutput);

    dError = direction_*desiredRadius_*desiredAngle_*(PI/180.0) - curPivDistance_;
    dErrorNet_.SetDouble(dError);
  }
}

/**
 * Calculates desired angle based off distance
 * @returns desired angle
 */
double CurveCommand::CalcCurDesiredAngle(double curPivDistance){

  double rawAngle = (curPivDistance/desiredRadius_ *180/PI) + initAngle_; 
  rawAngle = remainder((rawAngle+540.0), 360.0)-180.0;
  if(turnLeft_){ 
    return -rawAngle;
  } else {
    return rawAngle;
  }
}

/**
 * Sets dPFac_, dIFac_ and dDFac_
 */
void CurveCommand::GetPIDValues(){
  dPFac_ = robot_-> GetCurveDistanceP();
  dIFac_ = robot_-> GetCurveDistanceI();
  dDFac_ = robot_-> GetCurveDistanceD();
}

/**
 * Returns isDone_
 * @returns isDone_
 */
bool CurveCommand::IsDone(){
  return isDone_;
}

/**
 * Destructor
 */
CurveCommand::~CurveCommand(){
  // reset and delete PID values
  Reset();

  // delete shuffleboard entries
  dOutputNet_.Delete();
  lOutputNet_.Delete();
  rOutputNet_.Delete();
  dErrorNet_.Delete();
  pidSourceNet_.Delete();
}