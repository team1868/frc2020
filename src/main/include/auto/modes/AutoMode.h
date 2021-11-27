/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "RobotModel.h"
#include "ControlBoard.h"
#include "../commands/CurveCommand.h"
#include "auto/commands/DriveStraightCommand.h"
#include "auto/commands/PivotCommand.h"
#include "auto/commands/WaitingCommand.h"
#include "auto/PIDSource/PIDInputSource.h"
#include "auto/commands/PointCommand.h"
#include "auto/PIDSource/PIDOutputSource.h"
#include "auto/commands/ShootingCommand.h"
#include "auto/commands/IndexingCommand.h"
#include "auto/commands/IntakingCommand.h"
#include "auto/commands/PreppingCommand.h"
#include "auto/commands/AlignTapeCommand.h"
#include "auto/commands/SetLastAngleCommand.h"


class AutoMode {
  public:
    enum AutoPositions {kBlank}; // Change for 2022

    /**
     * Constructor
     * @param robot a RobotModel
     * @param controlBoard a ControlBoard
     */
    AutoMode(RobotModel *robot, ControlBoard *controlBoard);

    /**
     * Destructor 
    */
    virtual ~AutoMode();

    /**
     * CreateQueue
     * @param pos position 
    */
    virtual void CreateQueue(AutoMode::AutoPositions pos) {};

    /**
     * Gets queue of commands from auto sequence string 
     * @param autoSequence a std::string
     */
    void QueueFromString(std::string autoSequence);

    /**
     * Given character command from autoSequence, returns corresponding AutoCommand
     * @return tempCommand, the corresponding AutoCommand
     */ 
    AutoCommand* GetStringCommand(char command);

    /**
     * Error in stream, returns true if something went wrong
     * @return failed, a boolean, true if failed
     */ 
    bool IsFailed(char command);

    virtual void Init() = 0;

    /**
     * Periodic update
     * @param currTimeSec a double
     * @param deltaTimeSec a double
     */ 
    void Update(double currTimeSec, double deltaTimeSec);

    /**
     * Returns if done, false as long as current command is not null
     * @return true if done
     */ 
    bool IsDone();

    /**
     * Aborts current command
     */ 
    bool Abort();

    /**
     * Disables current command and goes back to the first command if not null
     */
	  void Disable();

  protected:
    AutoCommand *firstCommand_;
	  AutoCommand *currentCommand_;
	  RobotModel* robot_;
    ControlBoard *humanControl_;

	  NavXPIDSource* navX_;
	  TalonEncoderPIDSource* talonEncoder_;
    TalonEncoderPIDSource* talonEncoderCurve_;
    PivotPIDTalonOutput* talonOutput_;

	  AnglePIDOutput *angleOutput_;
	  DistancePIDOutput *distanceOutput_;

	  std::istringstream iss;
	  bool breakDesired_;
	  double currAngle_;
};