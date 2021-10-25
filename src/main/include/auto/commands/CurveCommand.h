/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "../AutoCommand.h"
#include "../../RobotModel.h"
#include "../PIDSource/PIDInputSource.h"
#include "../PIDSource/PIDOutputSource.h"

class CurveCommand : public AutoCommand {
 public:

   /** constuctor
   * @param robot a RobotModel
   * @param desiredRadius a double that refers to the desired radius
   * @param desiredAngle a double that refers to the desired angle for the robot to turn
   * @param turnLeft is true if the robot is supposed to turn left
   * @param goForward is true if robot should go forward
   * @param navXSource a NavXPIDSource
   * @param talonEncoderPIDSource a TalonEncoderPIDSource
   * @param anglePIDOuptut an AnglePIDOutput
   * @param distancePIDOutput a DistancePIDOutput
   */
  CurveCommand(RobotModel *robot, double desiredRadius, double desiredAngle, bool turnLeft, bool goForward,
    NavXPIDSource* navXSource, TalonEncoderPIDSource *talonEncoderPIDSource,
	  AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput);

  /** constuctor
   * @param robot a RobotModel
   * @param desiredRadius a double that refers to the desired radius
   * @param desiredAngle a double that refers to the desired angle for the robot to turn
   * @param turnLeft is true if the robot is supposed to turn left
   * @param goForward is true if robot should go forward
   * @param navXSource a NavXPIDSource
   * @param talonEncoderPIDSource a TalonEncoderPIDSource
   * @param anglePIDOuptut an AnglePIDOutput
   * @param distancePIDOutput a DistancePIDOutput
   */
  CurveCommand(RobotModel *robot, double desiredRadius, double desiredAngle,
    NavXPIDSource* navXSource, TalonEncoderPIDSource *talonEncoderPIDSource,
	  AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput);
  
  /**
  * creates new PIDController
  */
  void Init();
  
  /**
  * updates diffCurveTime_ and calls Reset() method if dPid_ is on target & numTimesOnTarget > 6
  * @param currTImeSec a double
  * @param deltaTimeSec a double
  */
  void Update(double currTimeSec, double deltaTimeSec);
  
  /**
  * returns isDone_
  */
  bool IsDone();

  /**
  * disables distance PID if it's null & set's isDone_ to true
  */
  void Reset();

  /**
  * destructor
  */
  ~CurveCommand();

 private:

  //calculates desired angle based off distance
  double CalcCurDesiredAngle(double curPivDistance);

  //sets dPFac_, dIFac_ and dDFac_
  void GetPIDValues();

  NavXPIDSource *navXPIDSource_;
  TalonEncoderPIDSource *talonEncoderPIDSource_;
  AnglePIDOutput *anglePIDOutput_;
  DistancePIDOutput *distancePIDOutput_;
  frc::PIDController *dPID_;

  double dPFac_, dIFac_, dDFac_;

  RobotModel *robot_;
  double initAngle_;
  double desiredRadius_, desiredAngle_;
  double curPivDistance_, curDesiredAngle_, curAngle_;
  double initialCurveTime_;
  double diffCurveTime_;

  double dMaxOutput_; // motor output
  double initialDMax_; // motor output
  double finalDMax_; // motor output
  double maxT_; //secs

  int numTimesOnTarget_;
  double curveTimeoutSec_;
  bool timeOut;

  double curAngleError_;

  // deals with direction robot is turning
  bool turnLeft_;
  bool goForward_;
  double direction_;

  bool isDone_;

  //shuffleboard
  frc::ShuffleboardLayout &curveLayout_;
  nt::NetworkTableEntry dOutputNet_, lOutputNet_, rOutputNet_,
    dErrorNet_, tErrorNet_, pidSourceNet_;
};