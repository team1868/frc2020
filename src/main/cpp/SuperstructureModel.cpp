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

void RobotModel::SetFlywheelOutput(double power){
    flywheelMotor1_->Set(power);
    //flywheelMotor2_->Set(-power);
}

double RobotModel::GetFlywheel1EncoderValue(){
    return flywheelEncoder1_->GetIntegratedSensorPosition();
}

double RobotModel::GetFlywheel2EncoderValue() {
    return -flywheelEncoder2_->GetIntegratedSensorPosition(); 
}

void RobotModel::SetControlModeVelocity(double desiredVelocity) {
    flywheelMotor1_->Set(ControlMode::Velocity, desiredVelocity);
}

int RobotModel::GetFlywheelMotor1Velocity() {
    return flywheelMotor1_->GetSelectedSensorVelocity(0); // 0 means primary closed loop
    // raw sensor units per 100 ms
}

// remove later
double RobotModel::RatioFlywheel(double value){
    /*double ratioFlywheelOutput = 12.43/GetVoltage()*value;
    if(ratioFlywheelOutput > 1){
        ratioFlywheelOutput = 1;
    } else if(ratioFlywheelOutput < -1){
        ratioFlywheelOutput = -1;
    }
    return ratioFlywheelOutput;*/
    //return GetVoltage()*0.00001421*desiredVelocity_;
}

// make into one function later whoops
void RobotModel::ConfigFlywheelP(double pFac){
    //printf("pFac %f\n", pFac);
    flywheelMotor1_->Config_kP(FLYWHEEL_PID_LOOP_ID, pFac);
}
void RobotModel::ConfigFlywheelI(double iFac){
    flywheelMotor1_->Config_kI(FLYWHEEL_PID_LOOP_ID, iFac);
}
void RobotModel::ConfigFlywheelD(double dFac){
    flywheelMotor1_->Config_kD(FLYWHEEL_PID_LOOP_ID, dFac);
}
void RobotModel::ConfigFlywheelF(double fFac){
    //printf("pid ff: %f\n", fFac);
    flywheelMotor1_->Config_kF(FLYWHEEL_PID_LOOP_ID, fFac);
}

double RobotModel::FlywheelMotor1Output(){
    flywheelMotor1_->GetMotorOutputPercent();
}

double RobotModel::FlywheelMotor2Output(){
    flywheelMotor2_->GetMotorOutputPercent();
}

bool RobotModel::IsAutoFlywheelAtSpeed(double desiredVelocity){
    // double value = GetFlywheelMotor1Velocity()*FALCON_TO_RPM;
    // //printf("falcon velocity %f\n", value);
    // //printf("desiredVelocity %f\n", desiredVelocity);
    // if(GetFlywheelMotor1Velocity()*FALCON_TO_RPM > desiredVelocity&& 
    // GetFlywheelMotor1Velocity()*FALCON_TO_RPM < desiredVelocity+150.0){
    //     numTimeAtSpeed_++;
    //     if (numTimeAtSpeed_ >= 1){ //3){ 
    //         //printf("FLYWHEEL IS AT SPEED");
    //         return true;
    //     }
    //     //numTimeAtSpeed_ = 0;
    //     return false;
    // }
    // numTimeAtSpeed_ = 0;
    // return false;
    return superstructureController_->IsFlywheelAtSpeed(desiredVelocity);
}

double RobotModel::GetFlywheelMotor1Current(){
    return flywheelMotor1_->GetSupplyCurrent();
}

double RobotModel::GetFlywheelMotor2Current(){
    return flywheelMotor2_->GetSupplyCurrent();
}

void RobotModel::EngageFlywheelHood() {
    flywheelHoodSolenoid_->Set(true);
}

void RobotModel::DisengageFlywheelHood() {
    //std::cout << "disengaging flywheel hood" << std::endl;
    flywheelHoodSolenoid_->Set(false);
}

void RobotModel::SetRightClimberElevatorOutput(double power){
    climberRightElevatorMotor_->Set(power);
}

void RobotModel::SetLeftClimberElevatorOutput(double power){
    climberLeftElevatorMotor_->Set(power);
}

bool RobotModel::GetRightLimitSwitch(){
    return (limitSwitchRight_->Get());
}

bool RobotModel::GetLeftLimitSwitch(){
    return (limitSwitchLeft_->Get());
}

void RobotModel::SetControlPanelOutput(double power){
    controlPanelMotor_->Set(power);
}

void RobotModel::SetIntakeRollersOutput(double power) {
    intakeRollersMotor_->Set(-power); // needs to be negative for comp bot
}

void RobotModel::SetIntakeWristOutput(double power) {
    if(power > 1.0){
        power = 1.0;
    } else if (power < -1.0) {
        power = -1.0;
    }
    intakeWristMotor_->Set(power);
}

void RobotModel::SetIndexFunnelOutput(double power) {
    indexFunnelMotor_->Set(-power);
}

void RobotModel::SetElevatorFeederOutput(double power) {
    elevatorFeederMotor_->Set(-power);
    //elevatorMotor_->Set(power);
}

void RobotModel::SetElevatorOutput(double power) {
    //std::cout << "elevator should be running B)" << std::endl;
    elevatorMotor_->Set(power);
    //elevatorFeederMotor_->Set(power);
}

void RobotModel::SetLight(bool setLight){
	lightSolenoid_ -> Set(setLight);
}


double RobotModel::GetTargetDistance() {
    // vision code to get distance, idk how that works but it needs to work
    return 0.0;
}


void RobotModel::GetColorFromSensor() {
    detectedColor_ = colorSensor_->GetColor();
    // cout<<"red "<<detectedColor_.red<<endl;
    // cout<<"green "<<detectedColor_.green<<endl;
    // cout<<"blue "<<detectedColor_.blue<<endl;
}


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

    //cout<<colorString_<<endl;
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

bool RobotModel::GetElevatorFeederLightSensorStatus() {
    return  elevatorFeederLightSensor_->Get();
}
bool RobotModel::GetElevatorLightSensorStatus() {
    return elevatorLightSensor_->Get();
}

