/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once 

#include "auto/modes/AutoMode.h"

class BlankMode : public AutoMode {
  public:
    /** 
     * Constructor
     * @param robot a RobotModel
     * @param controlBoard a ControlBoard 
     */
    BlankMode(RobotModel *robot, ControlBoard *controlBoard);

    /** 
     * Overrides CreateQueue method (virtual) from AutoMode, gets queue of commands from test sequence string 
     * @param pos an AutoPositions
     */
    void CreateQueue(AutoMode::AutoPositions pos) override;

    /** 
     * Initializes mode
     */
    void Init();

    /** 
     * Destructor
     */
    virtual ~BlankMode();
};