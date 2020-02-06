/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "Ports2020.h"
#include "ButtonReader.h"
#include <frc/WPILib.h>
#include <unordered_map>

class ControlBoard {
 public:
  enum Joysticks{ kLeftJoy, kRightJoy };
	enum Axes{ kX, kY, kZ, kLT};
  enum Buttons{kHighGearShift, kLowGearShift, kFlywheelButton, kClimberButton, kAlignButton, 
    kTrenchAlignButton, kControlPanelStage2Button, kControlPanelStage3Button, kIntakeSeriesButton,
    kIndexSeriesButton};

  ControlBoard();
  void ReadControls();
  double GetJoystickValue(Joysticks j, Axes a); 
  bool GetDesired(Buttons button);
  bool JustPressed(Buttons button);

  bool GetFlywheelDesired();
  bool GetClimberDesired();

  ~ControlBoard();

 private:
  void ReadAllButtons();
  frc::Joystick *leftJoy_, *rightJoy_, *operatorJoy_, *operatorJoyB_;
  ButtonReader *flywheelButton_, *climberButton_, *alignButton_, *trenchAlignButton_;
  double leftJoyX_, leftJoyY_, rightJoyX_, rightJoyY_;
  std::unordered_map<Buttons, ButtonReader*> buttons_;
};
