/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "auto/PIDSource/PIDInputSource.h"
#include "auto/PIDSource/PIDOutputSource.h"
#include "auto/commands/profiling/Setpoints.h"
#include "../pathfinder/pathfinder.h"
#include "auto/AutoCommand.h"
//#include <pair>

static const double MAX_HIGH_GEAR_SPEED = 13.4; //adjusted
static const double MAX_LOW_GEAR_SPEED = 8.22; //adjusted

class MotionProfileTestCommand : AutoCommand{
 public:
  MotionProfileTestCommand(RobotModel *robot, TalonEncoderPIDSource *distanceSource, NavXPIDSource *navXSource, DistancePIDOutput *distanceOutput, AnglePIDOutput *angleOutput);
  void Init();
  void Update(double currTimeSec, double deltaTimeSec);
  bool IsDone();
  void Reset();

  void CalcNext();

  ~MotionProfileTestCommand();
 private:
  RobotModel *robot_;
  TalonEncoderPIDSource *distanceSource_;
  NavXPIDSource *navXSource_;
  DistancePIDOutput *distanceOutput_;
  AnglePIDOutput *angleOutput_;
  double vP_, vI_, vD_;
  double aP_, aI_, aD_;
  PIDController *distancePID_, *anglePID_;
  double lastTime_;
  double currDistance_, nextDistance_, nextAngle_;
  double lastPosition_;
  double currPosition_;
  int pathIndex_;
  bool isDone_;
};
