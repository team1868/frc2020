/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"

void RobotModel::ShootingAutoInit(){
    superstructureController_->AutoInit();
}

void RobotModel::SetSuperstructureController(SuperstructureController *superstructureController){
    superstructureController_ = superstructureController;
}

void RobotModel::SetAutoState(uint32_t state) {
    state_ = state;
}

uint32_t RobotModel::GetAutoState() {
    return state_;
}

// run flywheel
void RobotModel::SetFlywheelOutput(double power){
    flywheelMotor1_->Set(power);
}

// get flywheel 1 motor encoder value
double RobotModel::GetFlywheel1EncoderValue(){
    return flywheelEncoder1_->GetIntegratedSensorPosition();
}

// get flywheel 2 motor encoder value
double RobotModel::GetFlywheel2EncoderValue() {
    return -flywheelEncoder2_->GetIntegratedSensorPosition(); 
}

// ?
void RobotModel::SetControlModeVelocity(double desiredVelocity) {
    flywheelMotor1_->Set(ControlMode::Velocity, desiredVelocity);
}

// get flywheel motor 1 velocity
int RobotModel::GetFlywheelMotor1Velocity() {
    return flywheelMotor1_->GetSelectedSensorVelocity(0); // 0 means primary closed loop
    // raw sensor units per 100 ms
}

void RobotModel::ConfigFlywheelPID(double pFac, double iFac, double dFac){
    flywheelMotor1_->Config_kP(FLYWHEEL_PID_LOOP_ID, pFac);
    flywheelMotor1_->Config_kI(FLYWHEEL_PID_LOOP_ID, iFac);
    flywheelMotor1_->Config_kD(FLYWHEEL_PID_LOOP_ID, dFac);
}

void RobotModel::ConfigFlywheelF(double fFac){
    flywheelMotor1_->Config_kF(FLYWHEEL_PID_LOOP_ID, fFac);
}

// get flywheel motor 1 output
double RobotModel::FlywheelMotor1Output(){
    flywheelMotor1_->GetMotorOutputPercent();
}

// get flywheel motor 2 output
double RobotModel::FlywheelMotor2Output(){
    flywheelMotor2_->GetMotorOutputPercent();
}

// checking if flywheel is at speed in auto 
bool RobotModel::IsAutoFlywheelAtSpeed(double desiredVelocity){
    return superstructureController_->IsFlywheelAtSpeed(desiredVelocity);
}

// get flywheel motor 1 supply current
double RobotModel::GetFlywheelMotor1Current(){
    return flywheelMotor1_->GetSupplyCurrent();
}

// get flywheel motor 2 supply current
double RobotModel::GetFlywheelMotor2Current(){
    return flywheelMotor2_->GetSupplyCurrent();
}

// bring flywheel hood up
void RobotModel::EngageFlywheelHood() {
    flywheelHoodSolenoid_->Set(true);
}

// bring flywheel hood down
void RobotModel::DisengageFlywheelHood() {
    flywheelHoodSolenoid_->Set(false);
}

void RobotModel::EngageClimberRatchet() {
    climberRatchetSolenoid_->Set(true);
}

void RobotModel::DisengageClimberRatchet() {
    climberRatchetSolenoid_->Set(false);
}

// run right climber motor
void RobotModel::SetRightClimberElevatorOutput(double power){
    climberRightElevatorMotor_->Set(power);
}

// run left climber motor
void RobotModel::SetLeftClimberElevatorOutput(double power){
    climberLeftElevatorMotor_->Set(power);
}

// run control panel motor
void RobotModel::SetControlPanelOutput(double power){
    controlPanelMotor_->Set(power);
}

// run intake rollers
void RobotModel::SetIntakeRollersOutput(double power) {
    intakeRollersMotor_->Set(power); // needs to be negative for comp bot
}

// move intake wrist
void RobotModel::SetIntakeWristOutput(double power) {
    // making sure that power isn't over 1.0 or under -1.0
    if(power > 1.0){
        power = 1.0;
    } else if (power < -1.0) {
        power = -1.0;
    }

    intakeWristMotor_->Set(power);
}

// run index funnel
void RobotModel::SetIndexFunnelOutput(double power) {
    indexFunnelMotorA_->Set(-power);
    indexFunnelMotorB_->Set(-power);
}

// run elevator feeder
void RobotModel::SetElevatorFeederOutput(double power) {
    elevatorFeederMotor_->Set(-power);
}

// run elevator
void RobotModel::SetElevatorOutput(double power) {
    elevatorMotor_->Set(power);
}

void RobotModel::SetLight(bool setLight){
	lightSolenoid_ -> Set(setLight);
}


// getting current color from sensor
void RobotModel::GetColorFromSensor() {
    detectedColor_ = colorSensor_->GetColor();
}

// getting detected color
std::string RobotModel::MatchColor() {
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

    return colorString_;
}

std::string RobotModel::GetControlPanelGameData() {
    return controlPanelGameData_;
}


//should return degrees
double RobotModel::GetIntakeWristAngle(){
    return TICKS_TO_WRIST_DEGREES*intakeWristMotor_->GetSelectedSensorPosition();
}

void RobotModel::ResetWristAngle(){
    intakeWristMotor_->SetSelectedSensorPosition(0);
}

#ifdef PRACTICE_BOT
bool RobotModel::GetElevatorFeederLightSensorStatus() {
    return  elevatorFeederLightSensor_->Get();
}
bool RobotModel::GetElevatorLightSensorStatus() {
    return elevatorLightSensor_->Get();
}

#else
bool RobotModel::GetElevatorFeederLightSensorStatus() {
    return  !elevatorFeederLightSensor_->Get();
}
bool RobotModel::GetElevatorLightSensorStatus() {
    return !elevatorLightSensor_->Get();
}
#endif