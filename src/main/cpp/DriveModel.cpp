/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotModel.h"

RobotModel::RobotModel() :
    driverTab_(frc::Shuffleboard::GetTab("Driveteam Control")),
    modeTab_(frc::Shuffleboard::GetTab("Programmer Control")),
    functionalityTab_(frc::Shuffleboard::GetTab("Functionality")),
    pidTab_(frc::Shuffleboard::GetTab("PID Values")),
    autoOffsetTab_(frc::Shuffleboard::GetTab("Auto Offset Values"))
    {
    
    frc::Shuffleboard::SelectTab("Driveteam Display");

    last_world_linear_accel_x_ = 0.0f;
    last_world_linear_accel_y_ = 0.0f;

    leftDriveOutput_ = rightDriveOutput_ = 0.0;

    lastLeftEncoderValue_ = lastRightEncoderValue_ = 0.0;
    currLeftEncoderValue_ = currRightEncoderValue_ = 0.0;
    
      // initializing timer
    timer_ = new frc::Timer();
    timer_->Start();

    // initializing pdp
    pdp_ = new frc::PowerDistributionPanel();

    // Initializing NavX
    navXSpeed_ = 200;
    navX_ = new AHRS(SPI::kMXP, navXSpeed_);
    Wait(1.0); // NavX takes a second to calibrate
    

    // initilize motor controllers
    leftMaster_ = new WPI_TalonFX(LEFT_DRIVE_MASTER_ID);
    rightMaster_ = new WPI_TalonFX(RIGHT_DRIVE_MASTER_ID);
    leftSlaveA_ = new WPI_TalonFX(LEFT_DRIVE_SLAVE_A_ID);
    rightSlaveA_ = new WPI_TalonFX(RIGHT_DRIVE_SLAVE_A_ID);
    // initialize encoders (talonfxsensorcollection)
    leftDriveEncoder_ = &leftMaster_->GetSensorCollection();
    rightDriveEncoder_ = &rightMaster_->GetSensorCollection(); 


    //TODO check for falcons
    // Setting talon control modes and slaves
    leftMaster_->Set(ControlMode::PercentOutput, 0.0);
    rightMaster_->Set(ControlMode::PercentOutput, 0.0);
    leftSlaveA_->Follow(*leftMaster_);
    rightSlaveA_->Follow(*rightMaster_);

    rightMaster_->SetInverted(false);
    rightSlaveA_->SetInverted(false);
    leftSlaveA_->SetInverted(false);
    leftMaster_->SetInverted(false);


    testSequence_ = "";

    //shuffleboard
    maxOutputEntry_ = GetModeTab().Add("Max Drive Output", 1.0).GetEntry();
    minVoltEntry_ = GetModeTab().Add("Min Voltage", MIN_VOLTAGE_BROWNOUT).GetEntry();
    maxCurrentEntry_ = GetModeTab().Add("Max Current", MAX_CURRENT_OUTPUT).GetEntry();
    leftDriveEncoderEntry_ = GetFunctionalityTab().Add("Left Drive Encoder", 0.0).GetEntry();
    rightDriveEncoderEntry_ = GetFunctionalityTab().Add("Right Drive Encoder", 0.0).GetEntry();

    lowGearSFrictionEntry_ = GetModeTab().Add("L SF", LOW_GEAR_STATIC_FRICTION_POWER).GetEntry();
    lowGearTurnSFrictionEntry_ = GetModeTab().Add("LT total SF", LOW_GEAR_QUICKTURN_STATIC_FRICTION_POWER).GetEntry();
    highGearSFrictionEntry_ = GetModeTab().Add("H SF", HIGH_GEAR_STATIC_FRICTION_POWER).GetEntry();
    highGearTurnSFrictionEntry_ = GetModeTab().Add("HT total SF", HIGH_GEAR_QUICKTURN_STATIC_FRICTION_POWER).GetEntry();

    // power distribution panel
    ratioPower_ = 1.0;
    leftDriveACurrent_ = 0.0;
    leftDriveBCurrent_ = 0.0;
    rightDriveACurrent_ = 0.0;
    rightDriveBCurrent_ = 0.0;
}

void RobotModel::SetDriveValues(double left, double right){
    leftMaster_->Set(left);
    rightMaster_->Set(-right);
}

void RobotModel::SetDriveValues(RobotModel::Wheels wheel, double value) {
    //value = ModifyCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN, value); // TODO
    switch (wheel) {
        case (kLeftWheels): // set left
            leftMaster_->Set(value);
            leftDriveOutput_ = value;
            break;
        case (kRightWheels): // set right
            rightMaster_->Set(-value);
            rightDriveOutput_ = value;
            break;
        case (kAllWheels): // set both
            rightMaster_->Set(-value);
            leftMaster_->Set(value);
            leftDriveOutput_ = rightDriveOutput_ = value;
            break;
        default:
        printf("WARNING: Drive value not set in RobotModel::SetDriveValues()");
    }
}

bool RobotModel::CollisionDetected() {
	bool collisionDetected = false;

	double curr_world_linear_accel_x = navX_->GetWorldLinearAccelX();
	double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x_;
	last_world_linear_accel_x_ = curr_world_linear_accel_x;
	double curr_world_linear_accel_y = navX_->GetWorldLinearAccelY();
	double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y_;
	last_world_linear_accel_y_ = curr_world_linear_accel_y;

	if(GetLeftEncoderStopped() && GetRightEncoderStopped()) {
		collisionDetected = true;
		printf("From ENCODER\n");
	}
	return collisionDetected;
}

double RobotModel::GetTime(){
    return timer_->Get();
}

double RobotModel::GetLeftEncoderValue(){
    return -leftDriveEncoder_->GetIntegratedSensorPosition();
}

double RobotModel::GetRightEncoderValue(){
    return rightDriveEncoder_->GetIntegratedSensorPosition();
}

//return feet
double RobotModel::GetLeftDistance() {
    return GetLeftEncoderValue()/ENCODER_TICKS_FOOT;
}

//return feet
double RobotModel::GetRightDistance() {
<<<<<<< HEAD
	return -rightDriveEncoder_->Get()*(LOW_GEAR_ROTATION_DISTANCE) / ENCODER_TICKS;
=======
	return GetRightEncoderValue()/ENCODER_TICKS_FOOT;
>>>>>>> 27156cfdfd7f61d2b676ca2396eb4351f20bc6ef
}

void RobotModel::ResetDriveEncoders() {
	leftDriveEncoder_->SetIntegratedSensorPosition(0.0);
    rightDriveEncoder_->SetIntegratedSensorPosition(0.0);
}


bool RobotModel::GetRightEncoderStopped() {
    if (currRightEncoderValue_ == lastRightEncoderValue_){
        return true;
    }
    return false;
}

bool RobotModel::GetLeftEncoderStopped() {
    if (currLeftEncoderValue_ == lastLeftEncoderValue_){
        return true;
    }
    return false;
}

void RobotModel::UpdateCurrent(int channel) {
    leftDriveACurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN);
	leftDriveBCurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_B_PDP_CHAN);
	rightDriveACurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
	rightDriveBCurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN);
}

double RobotModel::GetCurrent(int channel) {
    UpdateCurrent(channel);
	switch(channel) {
	case RIGHT_DRIVE_MOTOR_A_PDP_CHAN:
		return rightDriveACurrent_;
	case RIGHT_DRIVE_MOTOR_B_PDP_CHAN:
		return rightDriveBCurrent_;
	case LEFT_DRIVE_MOTOR_A_PDP_CHAN:
		return leftDriveACurrent_;
	case LEFT_DRIVE_MOTOR_B_PDP_CHAN:
		return leftDriveBCurrent_;
	default:
    	printf("WARNING: Current not recieved in RobotModel::GetCurrent()\n");
		return -1;
    }
}

void RobotModel::ModifyCurrent(int channel){

}

double RobotModel::GetTotalPower() {
	return pdp_->GetTotalPower();
}

double RobotModel::GetTotalEnergy() {
	return pdp_->GetTotalEnergy();
}

double RobotModel::GetNavXYaw() {
	return navX_->GetYaw();
}

void RobotModel::ZeroNavXYaw() {
	navX_->ZeroYaw();
	printf("Zeroed Yaw\n");
}

double RobotModel::GetNavXPitch() {
	return navX_->GetPitch();
}

double RobotModel::GetNavXRoll() {
	return navX_->GetRoll();
}

frc::ShuffleboardTab& RobotModel::GetDriverTab(){
    return driverTab_;
}

frc::ShuffleboardTab& RobotModel::GetModeTab(){
    return modeTab_;
}

frc::ShuffleboardTab& RobotModel::GetFunctionalityTab(){
    return functionalityTab_;
}

frc::ShuffleboardTab& RobotModel::GetPIDTab(){
    return pidTab_;
}

frc::ShuffleboardTab& RobotModel::GetAutoOffsetTab(){
    return autoOffsetTab_;
}

std::string RobotModel::GetTestSequence() {
	return testSequence_;
}

void RobotModel::SetTestSequence(std::string testSequence) {
	testSequence_ = testSequence;
}

void RobotModel::CreateNavX(){
	navXSource_ = new NavXPIDSource(this);
}

NavXPIDSource* RobotModel::GetNavXSource(){
	return navXSource_;
}

double RobotModel::GetCurrentVoltage() {
    return pdp_-> GetVoltage();
}

double RobotModel::GetTotalCurrent(){
    return pdp_->GetTotalCurrent();
}

void RobotModel::RefreshShuffleboard(){
    lastLeftEncoderValue_ = currLeftEncoderValue_;
    lastRightEncoderValue_ = currRightEncoderValue_;
    currLeftEncoderValue_ = GetLeftEncoderValue();
    currRightEncoderValue_ = GetRightEncoderValue();
    std::cout << "left:" << currLeftEncoderValue_ << " right:" << currRightEncoderValue_ << std::endl;
    leftDriveEncoderEntry_.SetDouble(currLeftEncoderValue_); 
    rightDriveEncoderEntry_.SetDouble(currRightEncoderValue_);
}