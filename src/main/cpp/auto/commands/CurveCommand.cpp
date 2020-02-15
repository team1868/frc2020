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
  bool goForward, NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
	AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput) : AutoCommand(), 
  curveLayout_(robot->GetFunctionalityTab().GetLayout("Curve", "List Layout")) { //using absolute angle, radius is middle of robot
  
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
  talonEncoderPIDSource_ = talonEncoderSource;
  anglePIDOutput_ = anglePIDOutput;
  distancePIDOutput_ = distancePIDOutput;
  isDone_ = false;

  dOutputNet_ = curveLayout_.Add("Curve dO", 0.0).GetEntry();
  //tOutputNet_ = curveLayout_.Add("Curve tO", 0.0).GetEntry(); 
  lOutputNet_ = curveLayout_.Add("Curve lO", 0.0).GetEntry();
  rOutputNet_ = curveLayout_.Add("Curve rO", 0.0).GetEntry();
  dErrorNet_ = curveLayout_.Add("Curve dErr", 0.0).GetEntry();
  //tErrorNet_ = curveLayout_.Add("Curve tErr", 0.0).GetEntry();
  pidSourceNet_ = curveLayout_.Add("Curve PID Get", 0.0).GetEntry();
}

CurveCommand::CurveCommand(RobotModel *robot, double desiredRadius, double desiredAngle,
  NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
	AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput) : AutoCommand(), 
  curveLayout_(robot->GetFunctionalityTab().GetLayout("Curve", "List Layout")) { //using absolute angle, radius is middle of robot
  
  robot_ = robot;
  desiredRadius_ = desiredRadius;
  desiredAngle_ = abs(desiredAngle);
  turnLeft_ = desiredAngle_<0;
  isDone_ = false;

  navXPIDSource_ = navXSource;
  talonEncoderPIDSource_ = talonEncoderSource;
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

  dPID_ = new PIDController(dPFac_, dIFac_, dDFac_, talonEncoderPIDSource_, distancePIDOutput_);
  //tPID_ = new PIDController(tPFac_, tIFac_, tDFac_, navXPIDSource_, anglePIDOutput_);

  //tPID_->SetPID(tPFac_, tIFac_, tDFac_);
	dPID_->SetPID(dPFac_, dIFac_, dDFac_);

  dPID_->SetSetpoint(direction_*desiredAngle_*desiredRadius_*PI/180.0);//2*PI*desiredRadius_/(360/desiredAngle_));
  //tPID_->SetSetpoint(desiredAngle_);

  dPID_->SetAbsoluteTolerance(3.0/12.0); //this too U DUDE
  //tPID_->SetAbsoluteTolerance(0.5); //HM TUNE TODODODODODOD

  //tPID_->SetContinuous(true);
  //tPID_->SetInputRange(-180, 180);
	dPID_->SetContinuous(false);
	//tPID_->SetInputRange(-180, 180);

  //tPID_->SetOutputRange(-0.9, 0.9);
	dPID_->SetOutputRange(-0.9, 0.9); //MAKE VARIABLES TODO TODO     //adjust for 2019
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

void CurveCommand::Update(double currTimeSec, double deltaTimeSec){ //TODO add timeout!

  //printf("I AM UPDATING\n");

  pidSourceNet_.SetDouble(talonEncoderPIDSource_->PIDGet());

  //if(dPID_->OnTarget() && tPID_->OnTarget()){ //TODO add timeout here, also TODO possible source of error if one done and one not?
  if(dPID_->OnTarget()){
    printf("%f Original Desired Distance: %f\n"
        "Final NavX Angle from PID Source: %f\n"
				"Final NavX Angle from robot: %f \n"
        "Final Distance from PID Source: %f\n",
				robot_->GetTime(), direction_*desiredAngle_*desiredRadius_*PI/180, navXPIDSource_->PIDGet(), robot_->GetNavXYaw(), talonEncoderPIDSource_->PIDGet());
    if(turnLeft_){
      printf("Final Distance from robot: %f\n", robot_->GetRightDistance());//robot_->GetLeftDistance()); /fixed inversion
    } else {
      printf("Final Distance from robot: %f\n", robot_->GetLeftDistance());
    }
		Reset();
		//robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
		printf("%f CurveCommand IS DONE \n", robot_->GetTime());
		//if (timeOut) {
		//	printf("%f FROM CURVE TIME OUT GO LEAVEEEEEE %f\n", robot_->GetTime(), timeDiff);
		//}
  } else {
    /* pid vs robot
    if(turnLeft_){
      curPivDistance_ = robot_->GetRightDistance();//robot_->GetLeftDistance();
    } else {
      curPivDistance_ = robot_->GetLeftDistance();
    }*/
    curPivDistance_ = talonEncoderPIDSource_->PIDGet();
    curDesiredAngle_ = CalcCurDesiredAngle(curPivDistance_);
    curAngleError_ = curDesiredAngle_ - curAngle_;

    //tPID_->SetSetpoint(curDesiredAngle_);

    double dOutput = distancePIDOutput_->GetPIDOutput();
    //double tOutput = anglePIDOutput_->GetPIDOutput();


    double lOutput;
    double rOutput;

    if(turnLeft_){
      // turning left, right wheel goes larger distance
      rOutput = direction_*dOutput*(desiredRadius_+ROBOT_WIDTH/2)/(desiredRadius_);
      //lOutput = (dOutput)/(ROBOT_WIDTH*desiredAngle_*PI/180); //strange math
      lOutput = direction_*dOutput * (desiredRadius_-ROBOT_WIDTH/2)/(desiredRadius_); //WORKS WHEN RADIUS > ROBOT_WIDTH/2
      // rOutput = dOutput + tOutput;
      // lOutput = dOutput - tOutput; 
    } else {
      //rOutput = (dOutput)/(ROBOT_WIDTH*desiredAngle_*PI/180); //strange math
      rOutput = direction_*dOutput * (desiredRadius_-ROBOT_WIDTH/2)/(desiredRadius_); //WORKS WHEN RADIUS > ROBOT_WIDTH/2
      lOutput = direction_*dOutput * (desiredRadius_+ROBOT_WIDTH/2)/(desiredRadius_);
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

    lOutput *= 0.5;
    rOutput *= 0.5;

    robot_->SetDriveValues(lOutput, rOutput);
    printf("dOutput: %f\n""rOutput: %f\n""lOutput: %f\n""curPivDistance: %f\n",
          dOutput, rOutput, lOutput, curPivDistance_);

    dOutputNet_.SetDouble(dOutput);
    //tOutputNet_.SetDouble(tOutput);
    lOutputNet_.SetDouble(lOutput);
    rOutputNet_.SetDouble(rOutput);

    dErrorNet_.SetDouble(direction_*desiredRadius_*desiredAngle_*(PI/180.0) - curPivDistance_);
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