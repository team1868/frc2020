/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// #include "auto/commands/profiling/MotionProfileCommand.h"

// MotionProfileCommand::MotionProfileCommand(
//     NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
//     AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput,
//     RobotModel* robot) :
//     TrapezoidProfileSubsystem<units::feet>(
//         {K_MAX_VELOC, K_MAX_ACCEL}, K_INIT_POS 
//     ){

//     kDistanceP_ = 0.8;
//     kDistanceI_ = 0.0;
//     kDistanceD_ = 0.0;

//     kAngleP_ = 0.04;
//     kAngleI_ = 0.0;
//     kAngleD_ = 0.0;
    
//     navXSource_ = navXSource;
//     talonEncoderSource_ = talonEncoderSource;
//     anglePIDOutput_ = anglePIDOutput;
//     distancePIDOutput_ = distancePIDOutput;
//     robot_ = robot;

//     //anglePID_ = new PIDController(kAngleP_, kAngleI_, kAngleD_, navXSource_, anglePIDOutput_);
// 	distancePID_ = new PIDController(kDistanceP_, kDistanceI_, kDistanceD_, talonEncoderSource_, distancePIDOutput_);
// }

// void MotionProfileCommand::UseState(State setpoint){

// }