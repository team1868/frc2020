// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// #pragma once

// #include <frc/WPILib.h>
// #include <TrapezoidProfileSubsystem.h>
// #include <units/units.h>
// #include "auto/PIDSource/PIDInputSource.h"
// #include "auto/PIDSource/PIDOutputSource.h"

// static const Velocity_t K_MAX_VELOC{13.0};
// static const Acceleration_t K_MAX_ACCEL{6.0};
// static const Distance_t K_INIT_POS{0.0};

// class MotionProfileCommand : TrapezoidProfileSubsystem{
//  public:
//   MotionProfileCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
//         AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot);
//   void UseState(State setpoint) override;
//  private:
//   double kAngleP_, kAngleI_, kAngleD_;
//   double kDistanceP_, kDistanceI_, kDistanceD_;
//   PIDController *anglePID_, *distancePID_;
//   NavXPIDSource* navXSource_;
//   TalonEncoderPIDSource* talonEncoderSource_;
//   AnglePIDOutput* anglePIDOutput_;
//   DistancePIDOutput* distancePIDOutput_;
//   RobotModel* robot_;
// };
