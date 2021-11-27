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
  //joysticks, axes & buttons
  enum Joysticks{kLeftJoy, kRightJoy};
	enum Axes{kX, kY, kZ, kLT};
  enum Buttons{kAlignButton, kGearShiftButton, 
              kIntakeSeriesButton, kIndexSeriesButton, 
              kShootClosePrepButton, kShootFarPrepButton, kShootingButton, 
              kClimbRightElevatorUpButton, kClimbRightElevatorDownButton, 
              kClimbLeftElevatorUpButton, kClimbLeftElevatorDownButton, 
              kUndoElevatorButton,
              kWristUpButton, kWristDownButton, kRunRollersButton, kReverseRollersButton,
              kControlPanelButton, kTestFarShootButton};

  /**
   * Constructor - 
   * sets each button to specific port & usb port
   */
  ControlBoard();

  /**
   * Gets joystick values given joystick and a specific axis
   * @param j left or right joystick (kLeftJoy, kRightJoy)
   * @param a x or y axis (kX, kY are the acceptable axes)
   * @return desired joystick's position on specified axis(x or y)
   */
  double GetJoystickValue(Joysticks j, Axes a); 

  /** 
   * Updates joystick variables to current joystick x & y coordinates
   */
  void ReadControls();

  /** 
   * Checks whether specific button is down
   * @param button a specific button
   * @return whether button is currently down aka value of button
   */
  bool GetDesired(Buttons button);

  /**
    * Checks if specific button was just pressed
    * @param button a specific button
    * @return only true when button is initially pressed
    */
  bool JustPressed(Buttons button);

  /**
   * Destructor
   */
  ~ControlBoard();

 private:
  /**
   * Reads & updates states(lastState & currState) of all buttons
   */
  void ReadAllButtons();
  
  frc::Joystick *leftJoy_, *rightJoy_, *operatorJoy_, *operatorJoyB_;
  double leftJoyX_, leftJoyY_, rightJoyX_, rightJoyY_;
  std::unordered_map<Buttons, ButtonReader*> buttons_;
};
