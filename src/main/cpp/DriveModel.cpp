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

      // initializing timer
    timer_ = new frc::Timer();
    timer_->Start();

    // initializing pdp
    pdp_ = new frc::PowerDistributionPanel();

    // Initializing NavX
    navXSpeed_ = 200;
    navX_ = new AHRS(SPI::kMXP, navXSpeed_);
    Wait(1.0); // NavX takes a second to calibrate

    //encoders
    leftDriveEncoder_ = new frc::Encoder(LEFT_DRIVE_ENCODER_YELLOW_PWM_PORT, LEFT_DRIVE_ENCODER_RED_PWM_PORT, true, frc::Encoder::EncodingType::k2X);
    leftDriveEncoder_->SetDistancePerPulse((LOW_GEAR_ROTATION_DISTANCE) / ENCODER_TICKS);
    leftDriveEncoder_->SetReverseDirection(false);

    rightDriveEncoder_ = new frc::Encoder(RIGHT_DRIVE_ENCODER_YELLOW_PWM_PORT, RIGHT_DRIVE_ENCODER_RED_PWM_PORT, false, frc::Encoder::EncodingType::k2X);
    rightDriveEncoder_->SetDistancePerPulse((LOW_GEAR_ROTATION_DISTANCE) / ENCODER_TICKS);
    rightDriveEncoder_->SetReverseDirection(false);

    // initilize motor controllers
    leftMaster_ = new WPI_TalonSRX(LEFT_DRIVE_MASTER_ID);
    rightMaster_ = new WPI_TalonSRX(RIGHT_DRIVE_MASTER_ID);
    leftSlaveA_ = new WPI_VictorSPX(LEFT_DRIVE_SLAVE_A_ID);
    rightSlaveA_ = new WPI_VictorSPX(RIGHT_DRIVE_SLAVE_A_ID);
    leftSlaveB_ = new WPI_VictorSPX(LEFT_DRIVE_SLAVE_B_ID);
    rightSlaveB_ = new WPI_VictorSPX(RIGHT_DRIVE_SLAVE_B_ID);

    //TODO check for falcons
    // Setting talon control modes and slaves
    leftMaster_->Set(ControlMode::PercentOutput, 0.0);
    rightMaster_->Set(ControlMode::PercentOutput, 0.0);
    leftSlaveA_->Follow(*leftMaster_);
    rightSlaveA_->Follow(*rightMaster_);
    leftSlaveB_->Follow(*leftMaster_);
    rightSlaveB_->Follow(*rightMaster_);

    rightMaster_->SetInverted(false);
    rightSlaveA_->SetInverted(false);
    rightSlaveB_->SetInverted(false);
    leftMaster_->SetInverted(false);
    leftSlaveA_->SetInverted(false);
    leftSlaveB_->SetInverted(false);

    //shuffleboard
    maxOutputEntry_ = GetModeTab().Add("Max Drive Output", 1.0).GetEntry();
    minVoltEntry_ = GetModeTab().Add("Min Voltage", MIN_VOLTAGE_BROWNOUT).GetEntry();
    maxCurrentEntry_ = GetModeTab().Add("Max Current", MAX_CURRENT_OUTPUT).GetEntry();

    lowGearSFrictionEntry_ = GetModeTab().Add("L SF", LOW_GEAR_STATIC_FRICTION_POWER).GetEntry();
    lowGearTurnSFrictionEntry_ = GetModeTab().Add("LT total SF", LOW_GEAR_QUICKTURN_STATIC_FRICTION_POWER).GetEntry();
    highGearSFrictionEntry_ = GetModeTab().Add("H SF", HIGH_GEAR_STATIC_FRICTION_POWER).GetEntry();
    highGearTurnSFrictionEntry_ = GetModeTab().Add("HT total SF", HIGH_GEAR_QUICKTURN_STATIC_FRICTION_POWER).GetEntry();

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

	if(leftDriveEncoder_->GetStopped() && rightDriveEncoder_->GetStopped()) {
		collisionDetected = true;
		printf("From ENCODER\n");
	}
	return collisionDetected;
}

double RobotModel::GetTime(){
    return timer_->Get();
}

double RobotModel::GetLeftEncoderValue(){
    return leftDriveEncoder_->Get();
}

double RobotModel::GetRightEncoderValue(){
    return rightDriveEncoder_->Get();
}

double RobotModel::GetLeftDistance() {
	return leftDriveEncoder_->Get()*(LOW_GEAR_ROTATION_DISTANCE) / ENCODER_TICKS;
}

double RobotModel::GetRightDistance() {
	return rightDriveEncoder_->Get()*(LOW_GEAR_ROTATION_DISTANCE) / ENCODER_TICKS;
}

void RobotModel::ResetDriveEncoders() {
	leftDriveEncoder_->Reset();
	rightDriveEncoder_->Reset();
}

bool RobotModel::GetLeftEncoderStopped() {
	return leftDriveEncoder_->GetStopped();
}

bool RobotModel::GetRightEncoderStopped() {
	return rightDriveEncoder_->GetStopped();
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

void RobotModel::CreateNavX(){
	navXSource_ = new NavXPIDSource(this);
}

NavXPIDSource* RobotModel::GetNavXSource(){
	return navXSource_;
}