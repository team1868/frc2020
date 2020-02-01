/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"
using namespace std;


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

void RobotModel::SetLight(bool setLight){
	lightSolenoid_ -> Set(setLight);
}


rev::CANSparkMax* RobotModel::GetFlywheelMotor1() {
    return flywheelMotor1_;
}

rev::CANSparkMax* RobotModel::GetFlywheelMotor2() {
    return flywheelMotor2_;
}


void RobotModel::GetColorFromSensor() {
    detectedColor_ = colorSensor_->GetColor();
    cout<<"red "<<detectedColor_.red<<endl;
    cout<<"green "<<detectedColor_.green<<endl;
    cout<<"blue "<<detectedColor_.blue<<endl;
}

void RobotModel::MatchColor() {
    colorConfidence_ = 0.9;
    matchedColor_ = colorMatcher_.MatchClosestColor(detectedColor_, colorConfidence_);
    
    if (matchedColor_ == kBlueTarget) {
      colorString_ = "Blue";
    } else if (matchedColor_ == kRedTarget) {
      colorString_ = "Red";
    } else if (matchedColor_ == kGreenTarget) {
      colorString_ = "Green";
    } else if (matchedColor_ == kYellowTarget) {
      colorString_ = "Yellow";
    } else {
      colorString_ = "Unknown"; // add some command to move forward if this happens
    }

    cout<<colorString_<<endl;
}

void RobotModel::GetControlPanelColor() {
    // blue: cyan 100 (255, 0, 255)
    // green: cyan 100 yellow 100 (0, 255, 0)
    // red: magenta 100 yellow 100 (255, 0 , 0)
    // yellow: yellow 100 (255, 255, 0)

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

