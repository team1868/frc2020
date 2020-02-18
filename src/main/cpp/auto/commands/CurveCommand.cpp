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
  bool goForward, NavXPIDSource* navXSource, TalonEncoderCurvePIDSource *talonEncoderCurvePIDSource,
	AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput) : AutoCommand(), 
  curveLayout_(robot->GetFunctionalityTab().GetLayout("Curve", "List Layout")) { //using relative angle, radius is middle of robot
  
  robot_ = robot;
  desiredRadius_ = desiredRadius;
  desiredAngle_ = desiredAngle;
  turnLeft_ = turnLeft;
  goForward_ = goForward;
  if (goForward_) {
    direction_ = 1.0;
  } else{
    direction_ = -1.0;
  }

  
  navXPIDSource_ = navXSource;
  talonEncoderCurvePIDSource_ = talonEncoderCurvePIDSource;
  anglePIDOutput_ = anglePIDOutput;
  distancePIDOutput_ = distancePIDOutput;
  isDone_ = false;

  dMaxOutput_ = 0.1; // motor output set to output range
  initialDMax_ = 0.1; // motor output
  finalDMax_ = 0.9; // motor output
  maxT_ = 1.0; //secs

  dOutputNet_ = curveLayout_.Add("Curve dO", 0.0).WithWidget(BuiltInWidgets::kGraph).GetEntry();
  //tOutputNet_ = curveLayout_.Add("Curve tO", 0.0).GetEntry(); 
  lOutputNet_ = curveLayout_.Add("Curve lO", 0.0).WithWidget(BuiltInWidgets::kGraph).GetEntry();
  rOutputNet_ = curveLayout_.Add("Curve rO", 0.0).WithWidget(BuiltInWidgets::kGraph).GetEntry();
  dErrorNet_ = curveLayout_.Add("Curve dErr", 0.0).WithWidget(BuiltInWidgets::kGraph).GetEntry();
  //tErrorNet_ = curveLayout_.Add("Curve tErr", 0.0).GetEntry();
  pidSourceNet_ = curveLayout_.Add("Curve PID Get", 0.0).WithWidget(BuiltInWidgets::kGraph).GetEntry();
}

CurveCommand::CurveCommand(RobotModel *robot, double desiredRadius, double desiredAngle,
  NavXPIDSource* navXSource, TalonEncoderCurvePIDSource *talonEncoderCurvePIDSource,
	AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput) : AutoCommand(), 
  curveLayout_(robot->GetFunctionalityTab().GetLayout("Curve", "List Layout")) { //using absolute angle, radius is middle of robot
  
  robot_ = robot;
  desiredRadius_ = desiredRadius;
  desiredAngle_ = abs(desiredAngle);
  turnLeft_ = desiredAngle_<0;
  isDone_ = false;

  navXPIDSource_ = navXSource;
  talonEncoderCurvePIDSource_ = talonEncoderCurvePIDSource;
  anglePIDOutput_ = anglePIDOutput;
  distancePIDOutput_ = distancePIDOutput;

  dOutputNet_ = curveLayout_.Add("Curve dO", 0.0).GetEntry();
  //tOutputNet_ = curveLayout_.Add("Curve tO", 0.0).GetEntry(); 
  lOutputNet_ = curveLayout_.Add("Curve lO", 0.0).GetEntry();
  rOutputNet_ = curveLayout_.Add("Curve rO", 0.0).GetEntry();
  dErrorNet_ = curveLayout_.Add("Curve dErr", 0.0).WithWidget(BuiltInWidgets::kGraph).GetEntry();
  //tErrorNet_ = curveLayout_.Add("Curve tErr", 0.0).GetEntry();
  pidSourceNet_ = curveLayout_.Add("Curve PID Get", 0.0).GetEntry();
}

void CurveCommand::Init(){

  initAngle_ = robot_->GetNavXYaw();

  curAngle_ = initAngle_;
  curPivDistance_ = 0.0;
  curDesiredAngle_ = curAngle_;
  initialCurveTime_ = robot_->GetTime();
  diffCurveTime_ = robot_->GetTime() - initialCurveTime_;
  numTimesOnTarget_ = 0;
  curveTimeoutSec_ = fabs(desiredAngle_*desiredRadius_*PI/180.0/3.0)+2.0;
  timeOut = false;

  //if(curAngle_>desiredAngle_) turnLeft_ = true;
  //else turnLeft_ = false;

  curAngleError_ = 0.0;

  //robot_->SetTalonCoastMode();
  robot_->ResetDriveEncoders(); 

  GetPIDValues();

  //navXPIDSource_ = new NavXPIDSource(robot_);
  //talonEncoderPIDSource_ = new TalonEncoderPIDSource(robot_);
  //anglePIDOutput_ = new AnglePIDOutput();
  //distancePIDOutput_ = new DistancePIDOutput();

  dPID_ = new PIDController(dPFac_, dIFac_, dDFac_, talonEncoderCurvePIDSource_, distancePIDOutput_);
  //tPID_ = new PIDController(tPFac_, tIFac_, tDFac_, navXPIDSource_, anglePIDOutput_);

  //tPID_->SetPID(tPFac_, tIFac_, tDFac_);
	dPID_->SetPID(dPFac_, dIFac_, dDFac_);

  dPID_->SetSetpoint(direction_*desiredAngle_*desiredRadius_*PI/180.0);//2*PI*desiredRadius_/(360/desiredAngle_));
  //tPID_->SetSetpoint(desiredAngle_);

  dPID_->SetAbsoluteTolerance(1.0/12.0); //this too U DUDE
  //tPID_->SetAbsoluteTolerance(0.5); //HM TUNE TODODODODODOD

  //tPID_->SetContinuous(true);
  //tPID_->SetInputRange(-180, 180);
	dPID_->SetContinuous(false);
	//tPID_->SetInputRange(-180, 180);

  //tPID_->SetOutputRange(-0.9, 0.9);
	dPID_->SetOutputRange(-initialDMax_, initialDMax_);     //adjust for 2019
	//tPID_->SetAbsoluteTolerance(0.3);	 //MAKE VARIABLES TODO TODO   //adjust for 2019

  dPID_->Enable();
  //tPID_->Enable();
  printf("done with init in curve\n");
}


void CurveCommand::Reset(){
  robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
	// destroy angle PID
	// if (tPID_ != NULL) {
	// 	tPID_->Disable();
	// 	delete tPID_;
	// 	tPID_ = NULL;
	// 	printf("Reset Angle PID %f \n", robot_->GetNavXYaw());
	// }

	// destroy distance PID
	if (dPID_ != NULL) {
		dPID_->Disable();
		delete dPID_;
		dPID_ = NULL;
		printf("Reset Distance PID");

	}
	isDone_ = true;
}

void CurveCommand::Update(double currTimeSec, double deltaTimeSec){

  //printf("I AM UPDATING\n");
  diffCurveTime_ = robot_->GetTime() - initialCurveTime_;
  //printf("direction: %f\n", direction_);
  pidSourceNet_.SetDouble(talonEncoderCurvePIDSource_->PIDGet());
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
            diffCurveTime_, direction_*desiredAngle_*desiredRadius_*PI/180, navXPIDSource_->PIDGet(), robot_->GetNavXYaw(), talonEncoderCurvePIDSource_->PIDGet());
    if(turnLeft_){
      printf("Final Distance from robot: %f\n", robot_->GetRightDistance());//robot_->GetLeftDistance()); /fixed inversion
    } else {
      printf("Final Distance from robot: %f\n", robot_->GetLeftDistance());
    }
    Reset();
    printf("%f CurveCommand IS DONE \n", robot_->GetTime());
  }
  else {
    curPivDistance_ = talonEncoderCurvePIDSource_->PIDGet();
    curDesiredAngle_ = CalcCurDesiredAngle(curPivDistance_);
    curAngleError_ = curDesiredAngle_ - curAngle_;

    //tPID_->SetSetpoint(curDesiredAngle_);

    double dOutput = distancePIDOutput_->GetPIDOutput();
    //double tOutput = anglePIDOutput_->GetPIDOutput();
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
      //lOutput = (dOutput)/(ROBOT_WIDTH*desiredAngle_*PI/180); //strange math
      lOutput = dOutput * (desiredRadius_-ROBOT_WIDTH/2)/(desiredRadius_); //WORKS WHEN RADIUS > ROBOT_WIDTH/2
      // rOutput = dOutput + tOutput;
      // lOutput = dOutput - tOutput; 
    } else {
      //rOutput = (dOutput)/(ROBOT_WIDTH*desiredAngle_*PI/180); //strange math
      rOutput = dOutput * (desiredRadius_-ROBOT_WIDTH/2)/(desiredRadius_); //WORKS WHEN RADIUS > ROBOT_WIDTH/2
      lOutput = dOutput * (desiredRadius_+ROBOT_WIDTH/2)/(desiredRadius_);
      // rOutput = dOutput - tOutput;
      // lOutput = dOutput + tOutput; 
    }

    
    //printf("AM I NEGATIVE?? %f %f %f", dOutput, rOutput, lOutput);
    
    //TODODODODO NEEDED OR IS THIS MESSING WITH THE PID????
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

    
    //ramp up max PID output from initial to final
    if (diffCurveTime_ > maxT_){
      dMaxOutput_ = finalDMax_;
    }
    else {
      dMaxOutput_ = initialDMax_ + ((finalDMax_-initialDMax_)/maxT_)*diffCurveTime_; 
    }
    dPID_->SetOutputRange(-dMaxOutput_, dMaxOutput_);

    robot_->SetDriveValues(lOutput, rOutput);
    printf("dOutput: %f\n""rOutput: %f\n""lOutput: %f\n""curPivDistance: %f\n" "curve time: %f\n",
          dOutput, rOutput, lOutput, curPivDistance_, diffCurveTime_);

    dOutputNet_.SetDouble(dOutput);
    //tOutputNet_.SetDouble(tOutput);
    lOutputNet_.SetDouble(lOutput);
    rOutputNet_.SetDouble(rOutput);

    dError = direction_*desiredRadius_*desiredAngle_*(PI/180.0) - curPivDistance_;
    dErrorNet_.SetDouble(dError);
    printf("dError: %f\n", dError);
    /*
    pid vs robot
    if(turnLeft_){
      dErrorNet_.SetDouble(2*PI/(360/desiredAngle_) - robot_->GetRightDistance());
    } else {
      dErrorNet_.SetDouble((2*PI/(360/desiredAngle_) - robot_->GetLeftDistance()));
    }
    */
    //tErrorNet_.SetDouble(curAngleError_);
  }
}


double CurveCommand::CalcCurDesiredAngle(double curPivDistance){

  double rawAngle = (curPivDistance/desiredRadius_ *180/PI) + initAngle_; //TODO POSSIBLE ERROR WITH INIT ANGLE
  rawAngle = remainder((rawAngle+540.0), 360.0)-180.0;
  if(turnLeft_){ //CHECK LOGIC??? why is right negative makes no sense
    return -rawAngle;
  } else {
    return rawAngle;
  }
}


void CurveCommand::GetPIDValues(){
  dPFac_ = robot_-> GetCurveDistanceP();
  dIFac_ = robot_-> GetCurveDistanceI();
  dDFac_ = robot_-> GetCurveDistanceD();

  // tPFac_ = robot_-> GetCurveTurnP();
  // tIFac_ = robot_-> GetCurveTurnI();
  // tDFac_ = robot_-> GetCurveTurnD();
}

bool CurveCommand::IsDone(){
  return isDone_;
}


CurveCommand::~CurveCommand(){
  /*dPID_->Disable();
  tPID_->Disable();
  dPID_->~PIDController();
  tPID_->~PIDController();*/
  Reset();

  dOutputNet_.Delete();
  //tOutputNet_.Delete();
  lOutputNet_.Delete();
  rOutputNet_.Delete();
  dErrorNet_.Delete();
  //tErrorNet_.Delete();
  pidSourceNet_.Delete();
}