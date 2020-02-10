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
  CurveCommand(RobotModel *robot, double desiredRadius, double desiredAngle, bool turnLeft,
    NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
	  AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput);
  CurveCommand(RobotModel *robot, double desiredRadius, double desiredAngle,
    NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
	  AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput);
  void Init();
  void Update(double currTimeSec, double deltaTimeSec);
  bool IsDone();
  void Reset();

  ~CurveCommand();

 private:
  double CalcCurDesiredAngle(double curPivDistance);
  void GetPIDValues();


  NavXPIDSource *navXPIDSource_;
  TalonEncoderPIDSource *talonEncoderPIDSource_;
  AnglePIDOutput *anglePIDOutput_;
  DistancePIDOutput *distancePIDOutput_;
  PIDController *dPID_, *tPID_;

  double dPFac_, dIFac_, dDFac_;
  double tPFac_, tIFac_, tDFac_;

  RobotModel *robot_;
  double initAngle_;
  double desiredRadius_, desiredAngle_;
  double curPivDistance_, curDesiredAngle_, curAngle_;

  double curAngleError_;

  bool turnLeft_;

  bool isDone_;

  frc::ShuffleboardLayout &curveLayout_;
  nt::NetworkTableEntry dOutputNet_, tOutputNet_, lOutputNet_, rOutputNet_,
    dErrorNet_, tErrorNet_, pidSourceNet_;
};