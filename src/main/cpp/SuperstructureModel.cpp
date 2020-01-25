/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"

void RobotModel::SetFlywheelOutput(double power){
    flywheelMotor1_ -> Set(power);
    //flywheelMotor2_ -> Set(-power);
}

void RobotModel::SetClimberOutput(double power){
    climberMotor1_ -> Set(power);
    if (climberEncoder1_->  GetPosition() >= SPARK_ENCODER_TICKS) {
        climberMotor2_ -> Set(-power);
    }
}

void RobotModel::SetLight(){
	lightSolenoid_ -> Set(true);
}


rev::CANSparkMax* RobotModel::GetFlywheelMotor1() {
    return flywheelMotor1_;
}

rev::CANSparkMax* RobotModel::GetFlywheelMotor2() {
    return flywheelMotor2_;
}

void RobotModel::GetControlPanelColor() {
    // blue: cyan 100
    // green: cyan 100 yellow 100
    // red: magenta 100 yellow 100
    // yellow: yellow 100

    if(controlPanelGameData_.length() > 0)
    {
        switch (controlPanelGameData_[0])
        {
            case 'B' :
                //Blue case code
                break;
            case 'G' :
                //Green case code
                break;
            case 'R' :
                 //Red case code
                break;
            case 'Y' :
                //Yellow case code
                break;
            default :
                printf("this data is corrupt");
                break;
        }
    } else {
        printf("no data received yet");
        // no data received yet
    }

}

