// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// #pragma once
// #include "../AutoCommand.h"
// #include "../../RobotModel.h"
// #include "../PIDSource/PIDInputSource.h"
// #include "../PIDSource/PIDOutputSource.h"
// #include "PivotCommand.h"
// #include "DriveStraightCommand.h"
// #include "../modes/AlignMode.h"

// #include <zmq.hpp>
// #include <zhelpers.hpp>

// #include <string>
// #include <chrono>
// #include <thread>
// #include <memory>
// #include <fstream>
// #include <iostream>


// class AlignTapeCommand : public AutoCommand {
// public:
//     AlignTapeCommand(RobotModel* robot, ControlBoard* humanControl,
//                     NavXPIDSource* navXSource, TalonEncoderPIDSource* talonSource,
//                     bool driveStraightDesired, double desiredRelativeAngle);
//     virtual ~AlignTapeCommand();

//     void Init();
//     void Update(double currTimeSec, double deltaTimeSec);
//     void Reset();
    
//     bool IsDone();
//     virtual bool Abort();

//     void ReadFromJetson();

// private:
//     // zmq::context_t *context_;
// 	// zmq::socket_t *subscriber_;

//     RobotModel* robot_;
//     ControlBoard* humanControl_;
//     NavXPIDSource* navXSource_;
//     //TalonEncoderPIDSource* talonSource_;
//     AnglePIDOutput* anglePIDOutput_;
//     //DistancePIDOutput* distancePIDOutput_;

//     PivotCommand* pivotCommand_;
//     //DriveStraightCommand* driveStraightCommand_;
//     AlignMode *alignSequence_;
//     string stringSequence_;

//     // TODO the below is for after a pretty good dead reckon, and would work for curve command
//     double desiredDeltaAngle_;
//     //double desiredDistance_;

//     //enum AlignState{ kPivotInit, kPivotUpdate, kDriveInit, kDriveUpdate};
//     enum AlignState{ kInit, kUpdate};

//     uint32_t currState_;
//     uint32_t nextState_;

//     double initTimeVision_, initTimeAlign_;

//     bool driveStraightDesired_;

//     bool isDone_;
//     bool abort_;
// };