/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "../../../include/auto/commands/CurveCommand.h"
#include <math.h>
#include <cmath>

CurveCommand::CurveCommand(RobotModel *robot, double desiredRadius, double desiredAngle, bool turnLeft,
  bool goForward, NavXPIDSource* navXSource, TalonEncoderPIDSource *talonEncoderPIDSource,
	AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput) : AutoCommand(), 
  curveLayout_(robot->GetFunctionalityTab().GetLayout("Curve", "List Layout")) { //using relative angle, radius is middle of robot
  
  robot_ = robot;
  desiredRadius_ = desiredRadius;
  desiredAngle_ = desiredAngle;
  turnLeft_ = turnLeft;
  goForward_ = goForward;
  isDone_ = false;

  if (goForward_) {
    direction_ = 1.0;
  } else{
    direction_ = -1.0;
  }
  
  navXPIDSource_ = navXSource;
  talonEncoderPIDSource_ = talonEncoderPIDSource;
  anglePIDOutput_ = anglePIDOutput;
  distancePIDOutput_ = distancePIDOutput;

  dMaxOutput_ = 0.1; // motor output set to output range
  initialDMax_ = 0.1; // motor output
  finalDMax_ = 0.9; // motor output
  maxT_ = 1.0; //secs

  dOutputNet_ = curveLayout_.Add("Curve dO", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
  lOutputNet_ = curveLayout_.Add("Curve lO", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
  rOutputNet_ = curveLayout_.Add("Curve rO", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
  dErrorNet_ = curveLayout_.Add("Curve dErr", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
  pidSourceNet_ = curveLayout_.Add("Curve PID Get", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
  printf("initialized curve with constructor 1\n");
}

CurveCommand::CurveCommand(RobotModel *robot, double desiredRadius, double desiredAngle,
  NavXPIDSource* navXSource, TalonEncoderPIDSource *talonEncoderPIDSource,
	AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput) : AutoCommand(), 
  curveLayout_(robot->GetFunctionalityTab().GetLayout("Curve", "List Layout")) { //using absolute angle, radius is middle of robot
  
  robot_ = robot;
  desiredRadius_ = desiredRadius;
  turnLeft_ = desiredAngle_ < 0;
  desiredAngle_ = abs(desiredAngle);
  isDone_ = false;

  navXPIDSource_ = navXSource;
  talonEncoderPIDSource_ = talonEncoderPIDSource;
  anglePIDOutput_ = anglePIDOutput;
  distancePIDOutput_ = distancePIDOutput;

  dOutputNet_ = curveLayout_.Add("Curve dO", 0.0).GetEntry();
  lOutputNet_ = curveLayout_.Add("Curve lO", 0.0).GetEntry();
  rOutputNet_ = curveLayout_.Add("Curve rO", 0.0).GetEntry();
  dErrorNet_ = curveLayout_.Add("Curve dErr", 0.0).WithWidget(frc::BuiltInWidgets::kGraph).GetEntry();
  //tErrorNet_ = curveLayout_.Add("Curve tErr", 0.0).GetEntry();
  pidSourceNet_ = curveLayout_.Add("Curve PID Get", 0.0).GetEntry();
  printf("initialized curve with constructor 2\n");
}

// creates new PIDController
void CurveCommand::Init(){

  initAngle_ = robot_->GetLastPivotAngle();

  double finalAngle = initAngle_ - desiredAngle_; //weird polarity
  if (finalAngle>180.0){
    finalAngle -= 360.0;
  } else if (finalAngle<-180){
    finalAngle += 360.0;
  }
  //if (initAngle_ + desiredAngle_ > 360){
  //  finalAngle = (initAngle_ + desiredAngle_)-360;
  //}
  //if(finalAngle > 180){
  //  finalAngle = -(360 - finalAngle);
  //}
  robot_->SetLastPivotAngle(finalAngle);

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

  dPID_->SetSetpoint(direction_*desiredAngle_*desiredRadius_*PI/180.0);//2*PI*desiredRadius_/(360/desiredAngle_));

  dPID_->SetAbsoluteTolerance(1.0/12.0); // TODO: tune

	dPID_->SetContinuous(false);

	dPID_->SetOutputRange(-initialDMax_, initialDMax_);

  dPID_->Enable();

  printf("done with init in curve\n");
}

// disables distance PID if it's null & set's isDone_ to true
void CurveCommand::Reset(){
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

//updates diffCurveTime_ and calls Reset() method if dPid_ is on target & numTimesOnTarget > 6
void CurveCommand::Update(double currTimeSec, double deltaTimeSec){

  diffCurveTime_ = robot_->GetTime() - initialCurveTime_;
  pidSourceNet_.SetDouble(talonEncoderPIDSource_->PIDGet());
  //if(dPID_->OnTarget() && tPID_->OnTarget()){ //TODO add timeout here, also TODO possible source of error if one done and one not?
  if(dPID_->OnTarget()) {
    numTimesOnTarget_++;
  }else{
    numTimesOnTarget_ = 0;
  }
  
  if(dPID_->OnTarget() && numTimesOnTarget_ > 6 ) {
    printf("difftime: %fs Original Desired Distance: %fft\n"
            "Final NavX Angle from PID Source: %f\n"
            "Final NavX Angle from robot: %f \n"
            "Final Distance from PID Source: %fft\n",
            diffCurveTime_, direction_*desiredAngle_*desiredRadius_*PI/180, navXPIDSource_->PIDGet(), 
            robot_->GetNavXYaw(), talonEncoderPIDSource_->PIDGet());
    if(turnLeft_){
       printf("Final Distance from robot: %f\n", robot_->GetRightDistance()); //fixed inversion
    } else {
      printf("Final Distance from robot: %f\n", robot_->GetLeftDistance());
    }
    Reset();
    printf("%f CurveCommand IS DONE \n", robot_->GetTime());
  }
  else {
    curPivDistance_ = talonEncoderPIDSource_->PIDGet();
    curDesiredAngle_ = CalcCurDesiredAngle(curPivDistance_);
    curAngleError_ = curDesiredAngle_ - curAngle_;

    double dOutput = distancePIDOutput_->GetPIDOutput();
    double frictionConstant = 0.05;
    if(dOutput < 0) {
      dOutput -= frictionConstant;
    } else{
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
    //power output checks
    if(lOutput > 1.0){
      rOutput = rOutput/lOutput;
      lOutput = 1.0;
    } else if (lOutput < -1.0){
      rOutput = rOutput/(-lOutput);
      lOutput = -1.0;
    }
    if(rOutput > 1.0) {
      lOutput = lOutput/rOutput;
      rOutput = 1.0;
    } else if (rOutput < -1.0) {
      lOutput = lOutput/(-rOutput);
      rOutput = -1.0;
    }

    lOutput *=0.8;
    rOutput *=0.8;
    
    //ramp up max PID output from initial to final
    if (diffCurveTime_ > maxT_){
      dMaxOutput_ = finalDMax_;
    } else {
      dMaxOutput_ = initialDMax_ + ((finalDMax_-initialDMax_)/maxT_)*diffCurveTime_; 
    }
    dPID_->SetOutputRange(-dMaxOutput_, dMaxOutput_);

    robot_->SetDriveValues(lOutput, rOutput);
    printf("dOutput: %f\n""rOutput: %f\n""lOutput: %f\n""curPivDistance: %f\n" "curve time: %f\n",
          dOutput, rOutput, lOutput, curPivDistance_, diffCurveTime_);

    dOutputNet_.SetDouble(dOutput);
    lOutputNet_.SetDouble(lOutput);
    rOutputNet_.SetDouble(rOutput);

    dError = direction_*desiredRadius_*desiredAngle_*(PI/180.0) - curPivDistance_;
    dErrorNet_.SetDouble(dError);
    printf("dError: %f\n", dError);
  }
}

//calculates desired angle based off distance
double CurveCommand::CalcCurDesiredAngle(double curPivDistance){

  double rawAngle = (curPivDistance/desiredRadius_ *180/PI) + initAngle_; //TODO POSSIBLE ERROR WITH INIT ANGLE
  rawAngle = remainder((rawAngle+540.0), 360.0)-180.0;
  if(turnLeft_){ //CHECK LOGIC??? why is right negative makes no sense
    return -rawAngle;
  } else {
    return rawAngle;
  }
}

//sets dPFac_, dIFac_ and dDFac_
void CurveCommand::GetPIDValues(){
  dPFac_ = robot_-> GetCurveDistanceP();
  dIFac_ = robot_-> GetCurveDistanceI();
  dDFac_ = robot_-> GetCurveDistanceD();
}

//returns isDone_
bool CurveCommand::IsDone(){
  return isDone_;
}

//destructor
CurveCommand::~CurveCommand(){
  Reset();

  dOutputNet_.Delete();
  lOutputNet_.Delete();
  rOutputNet_.Delete();
  dErrorNet_.Delete();
  pidSourceNet_.Delete();
}