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
	counter = 0;
	currLeftVelocity_ , currRightVelocity_ = 0.0;
	lastLeftVelocity_, lastRightVelocity_ = 0.0;
    lastLeftEncoderValue_ = lastRightEncoderValue_ = 0.0;
    currLeftEncoderValue_ = currRightEncoderValue_ = 0.0;
    
      // initializing timer
    timer_ = new frc::Timer();
    timer_->Start();

    // Initializing NavX
    navXSpeed_ = 200;
    navX_ = new AHRS(SPI::kMXP, navXSpeed_);
    Wait(1.0); // NavX takes a second to calibrate

    // initializing pdp
    pdp_ = new frc::PowerDistributionPanel();

    // power distribution panel
    ratioAll_ = 1.0;
    ratioDrive_ = 1.0;
    ratioSuperstructure_ = 1.0;
    lastOver_ = false;
    compressorOff_ = false;

    leftDriveACurrent_ = 0.0;
    leftDriveBCurrent_ = 0.0;
    rightDriveACurrent_ = 0.0;
    rightDriveBCurrent_ = 0.0;

    // initializing pnuematics
    compressor_ = new frc::Compressor(PNEUMATICS_CONTROL_MODULE_ID);

    // initilizing motor controllers
    leftMaster_ = new WPI_TalonFX(LEFT_DRIVE_MASTER_ID);
    rightMaster_ = new WPI_TalonFX(RIGHT_DRIVE_MASTER_ID);
    leftSlaveA_ = new WPI_TalonFX(LEFT_DRIVE_SLAVE_A_ID);
    rightSlaveA_ = new WPI_TalonFX(RIGHT_DRIVE_SLAVE_A_ID);
    // initializing encoders (talonfxsensorcollection)
    leftDriveEncoder_ = &leftMaster_->GetSensorCollection();
    rightDriveEncoder_ = &rightMaster_->GetSensorCollection(); 


    //TODO check for falcons
    // setting talon control modes and slaves
    leftMaster_->Set(ControlMode::PercentOutput, 0.0);
    rightMaster_->Set(ControlMode::PercentOutput, 0.0);
    leftSlaveA_->Follow(*leftMaster_);
    rightSlaveA_->Follow(*rightMaster_);

    rightMaster_->SetInverted(false);
    rightSlaveA_->SetInverted(false);
    leftSlaveA_->SetInverted(false);
    leftMaster_->SetInverted(false);


    testSequence_ = "";

    // shuffleboard
    maxOutputEntry_ = GetModeTab().Add("Max Drive Output", 1.0).GetEntry();
    minVoltEntry_ = GetModeTab().Add("Min Voltage", MIN_BROWNOUT_VOLTAGE).GetEntry();
    maxCurrentEntry_ = GetModeTab().Add("Max Current", MAX_CURRENT_OUTPUT).GetEntry();
    leftDriveEncoderEntry_ = GetFunctionalityTab().Add("Left Drive Encoder", 0.0).GetEntry();
    rightDriveEncoderEntry_ = GetFunctionalityTab().Add("Right Drive Encoder", 0.0).GetEntry();
	leftVelocityEntry_ = GetFunctionalityTab().Add("Left Velocity", 0.0).GetEntry();
    rightVelocityEntry_ = GetFunctionalityTab().Add("Right Velocity", 0.0).GetEntry();
	navXYawEntry_ = GetFunctionalityTab().Add("NavX Yaw", 0.0).GetEntry();

    lowGearSFrictionEntry_ = GetModeTab().Add("L SF", LOW_GEAR_STATIC_FRICTION_POWER).GetEntry();
    lowGearTurnSFrictionEntry_ = GetModeTab().Add("LT total SF", LOW_GEAR_QUICKTURN_STATIC_FRICTION_POWER).GetEntry();
    highGearSFrictionEntry_ = GetModeTab().Add("H SF", HIGH_GEAR_STATIC_FRICTION_POWER).GetEntry();
    highGearTurnSFrictionEntry_ = GetModeTab().Add("HT total SF", HIGH_GEAR_QUICKTURN_STATIC_FRICTION_POWER).GetEntry();

    ratioAllEntry_ = GetFunctionalityTab().Add("Ratio All", ratioAll_).GetEntry();
    ratioDriveEntry_ = GetFunctionalityTab().Add("Ratio Drive", ratioDrive_).GetEntry();
    ratioSuperstructureEntry_ = GetFunctionalityTab().Add("Ratio Superstructure", ratioSuperstructure_).GetEntry();
}

void RobotModel::SetDriveValues(double left, double right){
    leftMaster_->Set(left);
    rightMaster_->Set(-right);
}

void RobotModel::SetDriveValues(RobotModel::Wheels wheel, double value) {
    //value = ModifyCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN, value); // TODO
	//value = -value;
    switch (wheel) {
        case (kLeftWheels): // set left
            leftMaster_->Set(-value);
            leftDriveOutput_ = value;
            break;
        case (kRightWheels): // set right
            rightMaster_->Set(value);
            rightDriveOutput_ = value;
            break;
        case (kAllWheels): // set both
            rightMaster_->Set(value);
            leftMaster_->Set(-value);
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
	return GetRightEncoderValue()/ENCODER_TICKS_FOOT;
}

double RobotModel::GetLeftVelocity() {
    return -10*leftDriveEncoder_-> GetIntegratedSensorVelocity();   //TICkS PER SEC
}

double RobotModel::GetRightVelocity() {
    return 10*rightDriveEncoder_-> GetIntegratedSensorVelocity();   //TICkS PER SEC
}

void RobotModel::ResetDriveEncoders() {
	leftDriveEncoder_->SetIntegratedSensorPosition(0.0);
    rightDriveEncoder_->SetIntegratedSensorPosition(0.0);
}

bool RobotModel::GetLeftEncoderStopped() {
	if (currLeftVelocity_ < STOP_VELOCITY_THRESHOLD && currLeftVelocity_ > -STOP_VELOCITY_THRESHOLD 
	&& lastLeftVelocity_ < STOP_VELOCITY_THRESHOLD && lastLeftVelocity_ > -STOP_VELOCITY_THRESHOLD) {
		return true;
	}
	return false;
}

bool RobotModel::GetRightEncoderStopped() {
	if (currRightVelocity_ < STOP_VELOCITY_THRESHOLD && currRightVelocity_ > -STOP_VELOCITY_THRESHOLD 
	&& lastRightVelocity_ < STOP_VELOCITY_THRESHOLD && lastRightVelocity_ > -STOP_VELOCITY_THRESHOLD) {
		return true;
	}
	return false;
}

void RobotModel::StartCompressor() {
	compressor_->Start();
}

double RobotModel::GetCurrentVoltage() {
    return pdp_-> GetVoltage();
}

double RobotModel::GetTotalCurrent(){
    return pdp_->GetTotalCurrent();
}

double RobotModel::GetVoltage() {
	return pdp_->GetVoltage();
}

double RobotModel::GetCompressorCurrent() {
	return compressorCurrent_;
}

double RobotModel::GetRIOCurrent() {
	return roboRIOCurrent_;
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

double RobotModel::CheckMotorCurrentOver(int channel, double power){
    double motorCurrent = GetCurrent(channel);
	if( motorCurrent > MAX_DRIVE_MOTOR_CURRENT){ //current to individual motor is over, TODO change for super
		power = power*MAX_DRIVE_MOTOR_CURRENT / motorCurrent; //ratio down by percent over
	}
	return power;
}

void RobotModel::UpdateCurrent(int channel) {
    leftDriveACurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN);
	leftDriveBCurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_B_PDP_CHAN);
	rightDriveACurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
	rightDriveBCurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN);
    compressorCurrent_ = compressor_->GetCompressorCurrent();
    roboRIOCurrent_ = frc::RobotController::GetInputCurrent();

    // TODO fix and check logic
	if((GetTotalCurrent() > /*MAX_CURRENT_OUTPUT*/maxCurrentEntry_.GetDouble(MAX_CURRENT_OUTPUT) || GetVoltage() <= minVoltEntry_.GetDouble(MIN_BROWNOUT_VOLTAGE)) && !lastOver_){
		printf("\nSTOPPING\n\n");
		compressorOff_ = true;
		if(ratioAll_-0.05 > MIN_RATIO_ALL_CURRENT){
			ratioAll_ -= 0.05;
		} else if (ratioSuperstructure_-0.05 > MIN_RATIO_SUPERSTRUCTURE_CURRENT){
			ratioSuperstructure_ -= 0.05;
		} else if (ratioDrive_-0.05 > MIN_RATIO_DRIVE_CURRENT){
			ratioDrive_ -= 0.05;
		}
		lastOver_ = true;
	} else if((GetTotalCurrent() > /*MAX_CURRENT_OUTPUT*/maxCurrentEntry_.GetDouble(MAX_CURRENT_OUTPUT) || GetVoltage() <= minVoltEntry_.GetDouble(MIN_BROWNOUT_VOLTAGE) && lastOver_)){
		// know compressor is off, because lastOver_ is true
		// TODO WARNING THIS MIN IS NOT A MIN
		printf("am stopping too");
		if(ratioAll_ > MIN_RATIO_ALL_CURRENT){ //sketch, sketch, check this
			ratioAll_ *= ratioAll_;//-= 0.1;
		} else if (ratioSuperstructure_ > MIN_RATIO_SUPERSTRUCTURE_CURRENT){
			ratioSuperstructure_ *= ratioSuperstructure_; //-= 0.1;
		} else if (ratioDrive_ > MIN_RATIO_DRIVE_CURRENT){
			ratioDrive_ *= ratioDrive_;//-= 0.1;
		}
		lastOver_ = true;
	} else { 
		if(compressorOff_){
			StartCompressor();
			compressorOff_ = false;
		}
		if(ratioDrive_+0.001 < 1.0){
			ratioDrive_ *= 1.01;
		} else if (ratioDrive_ < 1.0){
			ratioDrive_ = 1.0;
		} else if(ratioSuperstructure_+0.001 < 1.0){
			ratioSuperstructure_ *= 1.01;
		} else if(ratioSuperstructure_ < 1.0){
			ratioSuperstructure_ = 1.0;
		} else if(ratioAll_+0.001 < 1.0){
			ratioAll_ *= 1.01;
		} else if(ratioAll_ < 1.0){
			ratioAll_ /= 1.01;
		} // else don't make it greater than one!
		lastOver_ = false;
	}


	ratioAllEntry_.SetDouble(ratioAll_);
	ratioDriveEntry_.SetDouble(ratioDrive_);
	ratioSuperstructureEntry_.SetDouble(ratioSuperstructure_);

	//printf("current updated\n");
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

double RobotModel::ModifyCurrent(int channel, double value){

    double power = value*ratioAll_;
	double individualPowerRatio = power;
	double tempPowerRatio;

	switch(channel){ // TODO check these constants what want to use? TODO CHANGE CHANGE DANG IT
		case LEFT_DRIVE_MOTOR_A_PDP_CHAN:
		case RIGHT_DRIVE_MOTOR_A_PDP_CHAN:
			power *= ratioDrive_;
			tempPowerRatio = CheckMotorCurrentOver(RIGHT_DRIVE_MOTOR_A_PDP_CHAN, power);
			if(tempPowerRatio < individualPowerRatio){
				individualPowerRatio = tempPowerRatio;
			}
			tempPowerRatio = CheckMotorCurrentOver(RIGHT_DRIVE_MOTOR_B_PDP_CHAN, power);
			if(tempPowerRatio < individualPowerRatio){
				individualPowerRatio = tempPowerRatio;
			}
			tempPowerRatio = CheckMotorCurrentOver(LEFT_DRIVE_MOTOR_A_PDP_CHAN, power);
			if(tempPowerRatio < individualPowerRatio){
				individualPowerRatio = tempPowerRatio;
			}
			tempPowerRatio = CheckMotorCurrentOver(LEFT_DRIVE_MOTOR_B_PDP_CHAN, power);
			if(tempPowerRatio < individualPowerRatio){
				individualPowerRatio = tempPowerRatio;
			}
			power = individualPowerRatio;
			break;
		/*case CARGO_INTAKE_MOTOR_PDP_CHAN:
			power *= ratioSuperstructure_;
			power = CheckMotorCurrentOver(CARGO_INTAKE_MOTOR_PDP_CHAN, power);
			break;
		case CARGO_FLYWHEEL_MOTOR_PDP_CHAN: //unused, dont want to slow flywheel of wont shoot
			power *= ratioSuperstructure_;
			power = CheckMotorCurrentOver(CARGO_FLYWHEEL_MOTOR_PDP_CHAN, power);
			break;*/
		default:
			printf("WARNING: current not found to modify.  In ModifyCurrents() in RobotModel.cpp");
	}
	// printf("ratio current %f, drive ratio current %f, super ratio current %d", ratioAll_, ratioDrive_, ratioSuperstructure_);
	return power;
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

void RobotModel::RefreshShuffleboard(){
    lastLeftEncoderValue_ = currLeftEncoderValue_;
    lastRightEncoderValue_ = currRightEncoderValue_;
    currLeftEncoderValue_ = GetLeftEncoderValue();
    currRightEncoderValue_ = GetRightEncoderValue();

	lastLeftVelocity_ = currLeftVelocity_;
	lastRightVelocity_ = currRightVelocity_;
	currLeftVelocity_ = GetLeftVelocity();
	currRightVelocity_ = GetRightVelocity();

    leftDriveEncoderEntry_.SetDouble(currLeftEncoderValue_); 
    rightDriveEncoderEntry_.SetDouble(currRightEncoderValue_);
	leftVelocityEntry_.SetDouble(currLeftVelocity_);
    rightVelocityEntry_.SetDouble(currRightVelocity_);
	navXYawEntry_.SetDouble(GetNavXYaw());
}