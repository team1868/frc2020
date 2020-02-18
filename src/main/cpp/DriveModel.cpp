/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "controllers/SuperstructureController.h
#include "RobotModel.h"
#include "ControlBoard.h"
#include "controllers/DriveController.h"
#include "auto/AutoMeasures.h"



const double MIN_TURNING_X = 0.5;
const double MIN_TURNING_XY_DIFFERENCE = 1.0;
static const double MAX_LOW_GEAR_VELOCITY = 8.5;

RobotModel::RobotModel() :
    driverTab_(frc::Shuffleboard::GetTab("Driveteam Control")),
    modeTab_(frc::Shuffleboard::GetTab("Programmer Control")),
    functionalityTab_(frc::Shuffleboard::GetTab("Functionality")),
    pidTab_(frc::Shuffleboard::GetTab("PID Values")),
    autoOffsetTab_(frc::Shuffleboard::GetTab("Auto Offset Values")),
	superstructureTab_(frc::Shuffleboard::GetTab("Superstructure")),
	driveStraightPIDLayout_(GetPIDTab().GetLayout("DriveStraight PID", "List Layout")),
	anglePIDLayout_(driveStraightPIDLayout_.GetLayout("Angle", "List Layout")),
	distancePIDLayout_(driveStraightPIDLayout_.GetLayout("Distance", "List Layout")),
	pivotPIDLayout_(GetPIDTab().GetLayout("Pivot", "List Layout")), 
	curvePIDLayout_(GetPIDTab().GetLayout("Curve PID", "List Layout")),
	curveTurnPIDLayout_(curvePIDLayout_.GetLayout("Curve Turn", "List Layout")),
	curveDistancePIDLayout_(curvePIDLayout_.GetLayout("Curve Distance", "List Layout")),
	pointPIDLayout_(GetPIDTab().GetLayout("Point", "List Layout"))
    {

    printf("I am in drive model constructor\n");
    frc::Shuffleboard::SelectTab("Driveteam Display");

	flywheelVelocTimeout_ = 30;

    last_world_linear_accel_x_ = 0.0f;
    last_world_linear_accel_y_ = 0.0f;
	

    leftDriveOutput_ = rightDriveOutput_ = 0.0;
	counter = 0;
	currLeftVelocity_ = currRightVelocity_ = 0.0;
	lastLeftVelocity_ = lastRightVelocity_ = 0.0;
	currLeftDistance_ = currRightDistance_ = 0.0;
	lastLeftDistance_ = lastRightDistance_ = 0.0;
    lastLeftEncoderValue_ = lastRightEncoderValue_ = 0.0;
    currLeftEncoderValue_ = currRightEncoderValue_ = 0.0;
    initialLeftEncoderValue_ = initialRightEncoderValue_ = 0.0;
	isHighGear_ = true;

      // initializing timer
    timer_ = new frc::Timer();
    timer_->Start();
    // Initializing NavX
    navXSpeed_ = 200;
    navX_ = new AHRS(frc::SPI::kMXP, navXSpeed_);
    frc::Wait(1.0); // NavX takes a second to calibrate
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

	// initializing double solenoid for gear
	gearSolenoid_ = new frc::DoubleSolenoid(PNEUMATICS_CONTROL_MODULE_ID, GEAR_SHIFT_FORWARDS_SOLENOID_PORT, GEAR_SHIFT_REVERSE_SOLENOID_PORT);
	
	// initializing solenoid for led light
	lightSolenoid_ = new frc::Solenoid(PNEUMATICS_CONTROL_MODULE_ID, LIGHT_SOLENOID_PORT);

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

	ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration currentLimitConfig;
	currentLimitConfig.enable = true;
	currentLimitConfig.triggerThresholdCurrent = 32.0;
	currentLimitConfig.triggerThresholdTime = 100.0;
	currentLimitConfig.triggerThresholdCurrent = 30.0;

	leftMaster_->ConfigSupplyCurrentLimit(currentLimitConfig);
	rightMaster_->ConfigSupplyCurrentLimit(currentLimitConfig);
	leftSlaveA_->ConfigSupplyCurrentLimit(currentLimitConfig);
    rightSlaveA_->ConfigSupplyCurrentLimit(currentLimitConfig);

	// superstructure robot model
	
	flywheelMotor1_ = new WPI_TalonFX(FLYWHEEL_MOTOR_ONE_ID);
	flywheelMotor2_ = new WPI_TalonFX(FLYWHEEL_MOTOR_TWO_ID);
	flywheelHoodSolenoid_ = new frc::Solenoid(PNEUMATICS_CONTROL_MODULE_ID, FLYWHEEL_HOOD_SOLENOID_PORT);

	flywheelMotor2_->Follow(*flywheelMotor1_); // should work :) - not tested tho
    flywheelMotor1_->SetInverted(false);
    flywheelMotor2_->SetInverted(true);

	std::cout << "start flywheel encoder creation" << std::endl << std::flush;
    flywheelEncoder1_ = &flywheelMotor1_->GetSensorCollection();
    flywheelEncoder2_ = &flywheelMotor2_->GetSensorCollection();
    std::cout << "end flywheel encoder creation" << std::endl << std::flush;

	climberWinchLeftMotor_ = new WPI_VictorSPX(CLIMB_WINCH_LEFT_MOTOR_ID);
	climberWinchRightMotor_ = new WPI_VictorSPX(CLIMB_WINCH_RIGHT_MOTOR_ID);
	climberElevatorMotor_ = new WPI_TalonSRX(CLIMB_ELEVATOR_ID);

	climberWinchRightEncoder_ = new Encoder(CLIMBER_WINCH_RIGHT_ENCODER_A_PWM_PORT, CLIMBER_WINCH_RIGHT_ENCODER_B_PWM_PORT, false);
	climberWinchLeftEncoder_ = new Encoder(CLIMBER_WINCH_LEFT_ENCODER_A_PWM_PORT, CLIMBER_WINCH_LEFT_ENCODER_B_PWM_PORT, true); // verify that it must be inverted

	intakeRollersMotor_ = new WPI_VictorSPX(INTAKE_ROLLERS_MOTOR_ID);
    intakeWristMotor_ = new WPI_TalonSRX(INTAKE_WRIST_MOTOR_ID);
	intakeWristMotor_->ConfigFactoryDefault();
	intakeWristMotor_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0);
	intakeWristPot_ = new frc::AnalogPotentiometer(INTAKE_WRIST_POT_PORT, 340.0, INTAKE_POT_OFFSET);
	leftDriveOutput_ = rightDriveOutput_ = 0;


    elevatorFeederLightSensor_ = new frc::DigitalInput(BOTTOM_ELEVATOR_LIGHT_SENSOR_PORT);
	elevatorLightSensor_ = new frc::DigitalInput(TOP_ELEVATOR_LIGHT_SENSOR_PORT);
	indexFunnelMotor_ = new WPI_TalonSRX(INDEX_FUNNEL_MOTOR_ID);
    elevatorFeederMotor_ = new WPI_TalonSRX(ELEVATOR_FEEDER_MOTOR_ID);
	elevatorMotor_ = new WPI_TalonSRX(ELEVATOR_MOTOR_ID);
	
	controlPanelMotor_ = new WPI_VictorSPX(CONTROL_PANEL_MOTOR_ID);
	controlPanelGameData_ = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	colorSensor_ = new rev::ColorSensorV3{I2CPORT};	
	colorMatcher_.AddColorMatch(kBlueTarget);
	colorMatcher_.AddColorMatch(kGreenTarget);
	colorMatcher_.AddColorMatch(kRedTarget);
	colorMatcher_.AddColorMatch(kYellowTarget); 

	flywheelOneCurrent_ = 0.0;
	flywheelTwoCurrent_ = 0.0;
	climbOneCurrent_ = 0.0;
	climbTwoCurrent_ = 0.0;
	intakeRollersCurrent_ = 0.0;
	intakeWristCurrent_ = 0.0;
	IndexFunnelCurrent_ = 0.0;
	elevatorFeederCurrent_ = 0.0;
	elevatorCurrent_ = 0.0;

	// shuffleboard
    testSequence_ = "";

	// flywheelMotor1_->ConfigFactoryDefault();
	// /* first choose the sensor */
	// flywheelMotor1_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
	// flywheelMotor1_->SetSensorPhase(true);

	// /* set the peak and nominal outputs */
	// flywheelMotor1_->ConfigNominalOutputForward(0, kTimeoutMs);
	// flywheelMotor1_->ConfigNominalOutputReverse(0, kTimeoutMs);
	// flywheelMotor1_->ConfigPeakOutputForward(1, kTimeoutMs);
	// flywheelMotor1_->ConfigPeakOutputReverse(-1, kTimeoutMs);


    maxOutputEntry_ = GetModeTab().Add("Max Drive Output", 1.0).GetEntry();
    minVoltEntry_ = GetModeTab().Add("Min Voltage", MIN_BROWNOUT_VOLTAGE).GetEntry();
    maxCurrentEntry_ = GetModeTab().Add("Max Current", MAX_CURRENT_OUTPUT).GetEntry();
    leftDriveEncoderEntry_ = GetFunctionalityTab().Add("Left Drive Encoder", 0.0).GetEntry();
    rightDriveEncoderEntry_ = GetFunctionalityTab().Add("Right Drive Encoder", 0.0).GetEntry();
	leftVelocityEntry_ = GetFunctionalityTab().Add("Left Velocity", 0.0).GetEntry();
    rightVelocityEntry_ = GetFunctionalityTab().Add("Right Velocity", 0.0).GetEntry();
	navXYawEntry_ = GetFunctionalityTab().Add("NavX Yaw", 0.0).GetEntry();
	voltageEntry_ = GetModeTab().Add("Battery Voltage", 12.5).GetEntry();

	leftCurrentEntry_ = GetFunctionalityTab().Add("Left Master Current", 0.0).GetEntry();
	rightCurrentEntry_ = GetFunctionalityTab().Add("Right Master Current", 0.0).GetEntry();


    lowGearSFrictionEntry_ = GetModeTab().Add("L SF", LOW_GEAR_STATIC_FRICTION_POWER).GetEntry();
    lowGearTurnSFrictionEntry_ = GetModeTab().Add("LT total SF", LOW_GEAR_QUICKTURN_STATIC_FRICTION_POWER).GetEntry();
    highGearSFrictionEntry_ = GetModeTab().Add("H SF", HIGH_GEAR_STATIC_FRICTION_POWER).GetEntry();
    highGearTurnSFrictionEntry_ = GetModeTab().Add("HT total SF", HIGH_GEAR_QUICKTURN_STATIC_FRICTION_POWER).GetEntry();

    ratioAllEntry_ = GetFunctionalityTab().Add("Ratio All", ratioAll_).GetEntry();
    ratioDriveEntry_ = GetFunctionalityTab().Add("Ratio Drive", ratioDrive_).GetEntry();
    ratioSuperstructureEntry_ = GetFunctionalityTab().Add("Ratio Superstructure", ratioSuperstructure_).GetEntry();
	rColorEntry_ = GetFunctionalityTab().Add("red", 0.0).GetEntry();
	gColorEntry_ = GetFunctionalityTab().Add("green", 0.0).GetEntry();
	bColorEntry_ = GetFunctionalityTab().Add("blue", 0.0).GetEntry();

	initLineErrorEntry_ = GetAutoOffsetTab().Add("initiation line distance", 0.0).GetEntry(); 
	trenchDistErrorEntry_ = GetAutoOffsetTab().Add("trench distance", 0.0).GetEntry(); 
	trenchWidthErrorEntry_ = GetAutoOffsetTab().Add("trench width", 0.0).GetEntry();  
	trenchLengthErrorEntry_ = GetAutoOffsetTab().Add("trench length", 0.0).GetEntry(); 
	targetZDistErrorEntry_ = GetAutoOffsetTab().Add("target zone", 0.0).GetEntry(); 
	targetZHeightErrorEntry_ = GetAutoOffsetTab().Add("target zone height", 0.0).GetEntry(); 
	loadingDDistErrorEntry_ = GetAutoOffsetTab().Add("loading dock", 0.0).GetEntry(); 
	playerSt2MidErrorEntry_ = GetAutoOffsetTab().Add("player station midpoint", 0.0).GetEntry(); 
	initLineSlantEntry_ = GetAutoOffsetTab().Add("initiation line slant", 0.0).GetEntry(); 
	std::cout<< "end of drive model constructor" << std::endl;
}

void RobotModel::SetDriveValues(double left, double right){
    leftMaster_->Set(-left);
    rightMaster_->Set(right);
	leftDrivePower_ = -left;
	rightDrivePower_ = right;
}

void RobotModel::SetDriveValues(RobotModel::Wheels wheel, double value) {
    // value = ModifyCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN, value); // TODO
	// value = -value;
    switch (wheel) {
        case (kLeftWheels): // set left
            leftMaster_->Set(-value);
            leftDriveOutput_ = value;
			leftDrivePower_ = -value;
            break;
        case (kRightWheels): // set right
            rightMaster_->Set(value);
            rightDriveOutput_ = value;
			leftDrivePower_ = value;
            break;
        case (kAllWheels): // set both
            rightMaster_->Set(value);
            leftMaster_->Set(-value);
            leftDriveOutput_ = rightDriveOutput_ = value;
			rightDriveOutput_ = value;
			leftDriveOutput_ = -value;
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

	collisionDetected = false; // For testing drive straight

	return collisionDetected;
}

double RobotModel::GetTime(){
    return timer_->Get();
}

double RobotModel::GetLeftEncoderValue(){
	//left needs to be negated //finds difference from stored value
    return -(leftDriveEncoder_->GetIntegratedSensorPosition() - initialLeftEncoderValue_); 
}

double RobotModel::GetRightEncoderValue(){
    return (rightDriveEncoder_->GetIntegratedSensorPosition() - initialRightEncoderValue_);
}

double RobotModel::GetRawLeftEncoderValue() {
	return leftDriveEncoder_->GetIntegratedSensorPosition();
}

double RobotModel::GetRawRightEncoderValue() {
	return rightDriveEncoder_->GetIntegratedSensorPosition();
}

//return feetGetLeftDis
double RobotModel::GetLeftDistance() {
	if (isHighGear_){
		return GetLeftEncoderValue()/HGEAR_ENCODER_TICKS_FOOT;
	} else{
		return GetLeftEncoderValue()/LGEAR_ENCODER_TICKS_FOOT;
	}
}

//return feet
double RobotModel::GetRightDistance() {
	if (isHighGear_){
		return GetRightEncoderValue()/HGEAR_ENCODER_TICKS_FOOT;
	} else{
		return GetRightEncoderValue()/LGEAR_ENCODER_TICKS_FOOT;
	}
}

double RobotModel::GetLeftVelocity() {
	return (currLeftDistance_ - lastLeftDistance_)/(currVelocTime_ - lastVelocTime_);
}
 
double RobotModel::GetRightVelocity() {
   return (currRightDistance_ - lastLeftDistance_)/(currVelocTime_ - lastVelocTime_);
}

void RobotModel::ResetDriveEncoders() {
	//read curr encoder values and store as initial encoder values
	initialLeftEncoderValue_ = GetRawLeftEncoderValue();
	initialRightEncoderValue_ = GetRawRightEncoderValue();
}

bool RobotModel::GetLeftEncoderStopped() {
	if (currLeftVelocity_ < STOP_VELOCITY_THRESHOLD && currLeftVelocity_ > -STOP_VELOCITY_THRESHOLD 
	&& lastLeftVelocity_ < STOP_VELOCITY_THRESHOLD && lastLeftVelocity_ > -STOP_VELOCITY_THRESHOLD) {
		return true;
	}
	// if (GetLeftVelocity() < STOP_VELOCITY_THRESHOLD && GetLeftVelocity() > -STOP_VELOCITY_THRESHOLD){
	// 	return true;
	// }
	return false;
}

bool RobotModel::GetRightEncoderStopped() {
	if (currRightVelocity_ < STOP_VELOCITY_THRESHOLD && currRightVelocity_ > -STOP_VELOCITY_THRESHOLD 
	&& lastRightVelocity_ < STOP_VELOCITY_THRESHOLD && lastRightVelocity_ > -STOP_VELOCITY_THRESHOLD) {
		return true;
	}
	// if (GetRightVelocity() < STOP_VELOCITY_THRESHOLD && GetRightVelocity() > -STOP_VELOCITY_THRESHOLD){
	// 	return true;
	// }
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

double RobotModel::GetPressureSwitchValue() {
	return 0.0; // fix
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
    /* leftDriveACurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN);
	leftDriveBCurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_B_PDP_CHAN);
	rightDriveACurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
	rightDriveBCurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN); */
	flywheelOneCurrent_ = pdp_->GetCurrent(FLYWHEEL_MOTOR_ONE_PDP_CHAN); // test if this works
	flywheelTwoCurrent_ = pdp_->GetCurrent(FLYWHEEL_MOTOR_TWO_PDP_CHAN);
	climbOneCurrent_ = pdp_->GetCurrent(CLIMB_MOTOR_ONE_PDP_CHAN);
	climbTwoCurrent_ = pdp_->GetCurrent(CLIMB_MOTOR_TWO_PDP_CHAN);
	intakeRollersCurrent_ = pdp_->GetCurrent(INTAKE_ROLLERS_MOTOR_PDP_CHAN);
	intakeWristCurrent_ = pdp_->GetCurrent(INTAKE_WRIST_MOTOR_PDP_CHAN);
	IndexFunnelCurrent_ = pdp_->GetCurrent(INDEX_FUNNEL_MOTOR_PDP_CHAN);
	elevatorFeederCurrent_ = pdp_->GetCurrent(ELEVATOR_FEEDER_MOTOR_PDP_CHAN);
	elevatorCurrent_ = pdp_->GetCurrent(ELEVATOR_MOTOR_PDP_CHAN);

	leftDriveACurrent_ = leftMaster_->GetSupplyCurrent(); //works
	leftDriveBCurrent_ = leftMaster_->GetSupplyCurrent();
	rightDriveACurrent_ = rightMaster_->GetSupplyCurrent();
	rightDriveBCurrent_ = rightMaster_->GetSupplyCurrent();

    compressorCurrent_ = compressor_->GetCompressorCurrent();
    roboRIOCurrent_ = frc::RobotController::GetInputCurrent();


    // TODO fix and check logic
	if((GetTotalCurrent() > /*MAX_CURRENT_OUTPUT*/maxCurrentEntry_.GetDouble(MAX_CURRENT_OUTPUT) || GetVoltage() <= minVoltEntry_.GetDouble(MIN_BROWNOUT_VOLTAGE)) && !lastOver_){
		printf("\nSTOPPING\n\n");
		printf("Total Current %f", GetTotalCurrent());
		printf("Voltage %f", GetVoltage());
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
		break;
	case RIGHT_DRIVE_MOTOR_B_PDP_CHAN:
		return rightDriveBCurrent_;
		break;
	case LEFT_DRIVE_MOTOR_A_PDP_CHAN:
		return leftDriveACurrent_;
		break;
	case LEFT_DRIVE_MOTOR_B_PDP_CHAN:
		return leftDriveBCurrent_;
		break;
	case FLYWHEEL_MOTOR_ONE_PDP_CHAN:
		return flywheelOneCurrent_;
		break;
	case FLYWHEEL_MOTOR_TWO_PDP_CHAN:
		return flywheelTwoCurrent_;
		break;
	case CLIMB_MOTOR_ONE_PDP_CHAN:
		return climbOneCurrent_;
		break;
	case CLIMB_MOTOR_TWO_PDP_CHAN:
		return climbTwoCurrent_;
		break;
	case INTAKE_ROLLERS_MOTOR_PDP_CHAN:
		return intakeRollersCurrent_;
		break;
	case INTAKE_WRIST_MOTOR_PDP_CHAN:
		return intakeWristCurrent_;
		break;
	/*case INDEX_FUNNEL_MOTOR_PDP_CHAN:
		return IndexFunnelCurrent_;
		break; TODO FIX THIS BACK*/
	case ELEVATOR_FEEDER_MOTOR_PDP_CHAN:
		return elevatorFeederCurrent_;
		break;
	case ELEVATOR_MOTOR_PDP_CHAN:
		return elevatorCurrent_;
		break;
	default:
    	printf("WARNING: Current not recieved in RobotModel::GetCurrent()\n");
		return -1;
    }
}

void RobotModel::GearShift() {
   if ((currLeftVelocity_ > MAX_LOW_GEAR_VELOCITY ||
        currRightVelocity_ > MAX_LOW_GEAR_VELOCITY) || 
	   (currLeftVelocity_ < -MAX_LOW_GEAR_VELOCITY ||
        currRightVelocity_ < -MAX_LOW_GEAR_VELOCITY)) {
            SetHighGear();
			//printf("High gear: %f ", GetRightVelocity());
			//printf("%f\n", GetLeftVelocity());
    }
	else {
        SetLowGear();
		//printf("Low gear: %f ", GetRightVelocity());
		//printf("%f\n", GetLeftVelocity());
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

void RobotModel::SetHighGear(){
	if (isHighGear_ == false) {
		gearSolenoid_ -> Set(frc::DoubleSolenoid::Value::kForward);
		isHighGear_ = true;
	}
	//ResetDriveEncoders();
}

void RobotModel::SetLowGear(){
	if (isHighGear_ == true) {
		gearSolenoid_ -> Set(frc::DoubleSolenoid::Value::kReverse);
		isHighGear_ = false;
	}
	//ResetDriveEncoders();
}


//align tapes
void RobotModel::SetDeltaAngle(double angle) {
	desiredDeltaAngle_ = angle;
}
void RobotModel::SetDistance(double distance) {
	desiredDistance_ = distance;
}

double RobotModel::GetDeltaAngle() {
	return desiredDeltaAngle_;
}
double RobotModel::GetDistance() {
	return desiredDistance_;
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

frc::ShuffleboardTab& RobotModel::GetSuperstructureTab(){
    return superstructureTab_;
}

std::string RobotModel::GetTestSequence() {
	return testSequence_;
}

void RobotModel::SetTestSequence(std::string testSequence) {
	testSequence_ = testSequence;
}

std::string RobotModel::GetAlignSequence() {
	return alignSequence_;
}

void RobotModel::SetAlignSequence(std::string alignSequence) {
	alignSequence_ = alignSequence;
}


void RobotModel::CreateNavX(){
	navXSource_ = new NavXPIDSource(this);
}

NavXPIDSource* RobotModel::GetNavXSource(){
	return navXSource_;
}


void RobotModel::RefreshShuffleboard(){

	/* set closed loop gains in slot0 */

	// flywheelMotor1_->Config_kF(kPIDLoopIdx, 0.1097, flywheelVelocTimeout_);
	// flywheelMotor1_->Config_kP(kPIDLoopIdx, 0.22, kTimeoutMs);
	// flywheelMotor1_->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
	// flywheelMotor1_->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

	lastVelocTime_ = currVelocTime_;
	currVelocTime_ = GetTime();
	lastLeftDistance_ = currLeftDistance_;
	lastRightDistance_ = currRightDistance_;
	currLeftDistance_ = GetLeftDistance();
	currRightDistance_ = GetRightDistance();
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
	voltageEntry_.SetDouble(GetCurrentVoltage());

	rColorEntry_.SetDouble(detectedColor_.red);
	gColorEntry_.SetDouble(detectedColor_.green);
	bColorEntry_.SetDouble(detectedColor_.blue);
 

	UpdateCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
	leftCurrentEntry_.SetDouble(leftDriveACurrent_);
	rightCurrentEntry_.SetDouble(rightDriveACurrent_);


	initLineError_ = initLineErrorEntry_.GetDouble(0.0);
	trenchDistError_ = trenchDistErrorEntry_.GetDouble(0.0);
	trenchWidthError_ = trenchWidthErrorEntry_.GetDouble(0.0);
	trenchLengthError_ = trenchLengthErrorEntry_.GetDouble(0.0);
	targetZDistError_ = targetZDistErrorEntry_.GetDouble(0.0);
	loadingDDistError_ = loadingDDistErrorEntry_.GetDouble(0.0);
	playerSt2MidError_ = playerSt2MidErrorEntry_.GetDouble(0.0);

	// if (leftDriveACurrent_ != 0.0 || rightDriveACurrent_ != 0.0) {
	// 	std::cout<< "left: " << leftDriveACurrent_ << " right: " << rightDriveACurrent_ <<std::endl;
	// }
	//std::cout<< "left encoder: " << currLeftEncoderValue_ << " right encoder: " << currRightEncoderValue_ <<std::endl;
	//std::cout<< "time: " << GetTime() << std::endl;
}

RobotModel::~RobotModel(){
	//drive straight
	aPEntry_.Delete();
    aIEntry_.Delete(); 
    aDEntry_.Delete();
    dPEntry_.Delete();
    dIEntry_.Delete(); 
    dDEntry_.Delete(); 

	//pivot
	pEntry_.Delete(); 
	iEntry_.Delete(); 
	dEntry_.Delete(); 

	//curve
	dPFacNet_.Delete();
  	dIFacNet_.Delete();
  	dDFacNet_.Delete();
 	// tPFacNet_.Delete();
  	// tIFacNet_.Delete();
  	// tDFacNet_.Delete();
	
	//point
	pEntryP_.Delete(); 
	iEntryP_.Delete(); 
	dEntryP_.Delete();

	//drive stuff
	leftDriveEncoderEntry_.Delete();
	rightDriveEncoderEntry_.Delete();
	leftVelocityEntry_.Delete();
	rightVelocityEntry_.Delete();

	navXYawEntry_.Delete();
	voltageEntry_.Delete();
	leftCurrentEntry_.Delete();
	rightCurrentEntry_.Delete();

	maxOutputEntry_.Delete();
	minVoltEntry_.Delete();
	maxCurrentEntry_.Delete();
    lowGearSFrictionEntry_.Delete();
	lowGearTurnSFrictionEntry_.Delete();
	highGearSFrictionEntry_.Delete();
	highGearTurnSFrictionEntry_.Delete();
    ratioAllEntry_.Delete();
	ratioDriveEntry_.Delete();
	ratioSuperstructureEntry_.Delete();
}
