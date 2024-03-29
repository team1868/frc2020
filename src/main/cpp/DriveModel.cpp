/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "controllers/SuperstructureController.h"
#include "RobotModel.h"
#include "ControlBoard.h"
#include "controllers/DriveController.h"

/**
 * Constructor for RobotModel
 */ 
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
	gearSolenoid_ = new frc::Solenoid(PNEUMATICS_CONTROL_MODULE_ID, GEAR_SHIFT_SOLENOID_PORT);
	
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

    // setting talon control modes and slaves
    leftMaster_->Set(ControlMode::PercentOutput, 0.0);
    rightMaster_->Set(ControlMode::PercentOutput, 0.0);
    leftSlaveA_->Follow(*leftMaster_);
    rightSlaveA_->Follow(*rightMaster_);

    rightMaster_->SetInverted(false);
    rightSlaveA_->SetInverted(false);
    leftSlaveA_->SetInverted(false);
    leftMaster_->SetInverted(false);
	rightMaster_->SetNeutralMode(Coast);
	leftMaster_->SetNeutralMode(Coast);
	leftSlaveA_->SetNeutralMode(Coast);
	rightSlaveA_->SetNeutralMode(Coast);

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
	fortyAmpFXLimit_ = new StatorCurrentLimitConfiguration(true, 32.0, 80.0, 0.5);
	fortyAmpSRXLimit_ = new SupplyCurrentLimitConfiguration(true, 32.0, 40.0, 0.1);
	thirtyAmpSRXLimit_ = new SupplyCurrentLimitConfiguration(true, 24.0, 30.0, 0.1);
	
	flywheelMotor1_ = new WPI_TalonFX(FLYWHEEL_MOTOR_ONE_ID);
	flywheelMotor2_ = new WPI_TalonFX(FLYWHEEL_MOTOR_TWO_ID);
	flywheelHoodSolenoid_ = new frc::Solenoid(PNEUMATICS_CONTROL_MODULE_ID, FLYWHEEL_HOOD_SOLENOID_PORT);

	flywheelMotor2_->Follow(*flywheelMotor1_); 
    flywheelMotor1_->SetInverted(false);
    flywheelMotor2_->SetInverted(true);

	flywheelMotor1_->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
	flywheelMotor1_->ConfigPeakOutputForward(1);
	flywheelMotor1_->ConfigPeakOutputReverse(0);

	numTimeAtSpeed_ = 0;

	std::cout << "start flywheel encoder creation" << std::endl << std::flush;
    flywheelEncoder1_ = &flywheelMotor1_->GetSensorCollection();
    flywheelEncoder2_ = &flywheelMotor2_->GetSensorCollection();
    std::cout << "end flywheel encoder creation" << std::endl << std::flush;

	climberRightElevatorMotor_ = new WPI_TalonSRX(CLIMB_RIGHT_ELEVATOR_ID);
	climberLeftElevatorMotor_ = new WPI_TalonSRX(CLIMB_LEFT_ELEVATOR_ID);
	climberRatchetSolenoid_ = new frc::Solenoid(CLIMB_RATCHET_SOLENOID_PORT);

	intakeRollersMotor_ = new WPI_VictorSPX(INTAKE_ROLLERS_MOTOR_ID);
    intakeWristMotor_ = new WPI_TalonSRX(INTAKE_WRIST_MOTOR_ID);
	intakeWristMotor_->ConfigSupplyCurrentLimit(*thirtyAmpSRXLimit_);
	intakeWristMotor_->ConfigFactoryDefault();
	intakeWristMotor_->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0);
	leftDriveOutput_ = rightDriveOutput_ = 0;
	intakeWristMotor_->SetSelectedSensorPosition(0); //arm up
	resetWristAngle_ = false;

    elevatorFeederLightSensor_ = new frc::DigitalInput(BOTTOM_ELEVATOR_LIGHT_SENSOR_PORT);
	elevatorLightSensor_ = new frc::DigitalInput(TOP_ELEVATOR_LIGHT_SENSOR_PORT);
	funnelLightSensor_ = new frc::DigitalInput(FUNNEL_LIGHT_SENSOR_PORT);
#ifdef PRACTICE_BOT
	//printf("WORKINGINSIDNFLKSDFJLKSF\n");
	indexFunnelMotor_ = new WPI_TalonSRX(INDEX_FUNNEL_MOTOR_ID); // practice bot
    elevatorFeederMotor_ = new WPI_TalonSRX(ELEVATOR_FEEDER_MOTOR_ID); // practice bot
#else
	//printf("NOT WORKINGLKSJDFLKJSDLKFJSLDKJFKLSKDFJLDKJFLKD\n");
	indexFunnelMotorA_ = new WPI_VictorSPX(INDEX_FUNNEL_MOTOR_A_ID); // comp bot
	indexFunnelMotorB_ = new WPI_VictorSPX(INDEX_FUNNEL_MOTOR_B_ID);
    elevatorFeederMotor_ = new WPI_VictorSPX(ELEVATOR_FEEDER_MOTOR_ID); // com76 p bot
#endif
	elevatorMotor_ = new WPI_VictorSPX(ELEVATOR_MOTOR_ID);

	controlPanelMotor_ = new WPI_VictorSPX(CONTROL_PANEL_MOTOR_ID);
	controlPanelGameData_ = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	colorSensor_ = new rev::ColorSensorV3{I2CPORT};	

	limitSwitchRight_ = new frc::DigitalInput(RIGHT_CLIMB_LIMIT_SWITCH_PORT);
	limitSwitchLeft_ = new frc::DigitalInput(LEFT_CLIMB_LIMIT_SWITCH_PORT);
	
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
	indexFunnelACurrent_ = 0.0;
	indexFunnelBCurrent_ = 0.0;
	elevatorFeederCurrent_ = 0.0;
	elevatorCurrent_ = 0.0;

	// shuffleboard
    testSequence_ = "";

	context_ = nullptr; //same context for send + receive zmq
    publisher_ = nullptr;
    subscriber_ = nullptr;
    isSocketBound_ = false;
	hasContents_ = false;
	desiredDistance_ = 0.0;
	desiredDeltaAngle_ = 0.0;

    maxOutputEntry_ = GetModeTab().Add("Max Drive Output", 1.0).GetEntry();
    minVoltEntry_ = GetModeTab().Add("Min Voltage", MIN_BROWNOUT_VOLTAGE).GetEntry();
    maxCurrentEntry_ = GetModeTab().Add("Max Current", MAX_CURRENT_OUTPUT).GetEntry();
    leftDriveEncoderEntry_ = GetFunctionalityTab().Add("Left Drive Encoder", 0.0).GetEntry();
    rightDriveEncoderEntry_ = GetFunctionalityTab().Add("Right Drive Encoder", 0.0).GetEntry();
	leftVelocityEntry_ = GetFunctionalityTab().Add("Left Velocity", 0.0).GetEntry();
    rightVelocityEntry_ = GetFunctionalityTab().Add("Right Velocity", 0.0).GetEntry();
	navXYawEntry_ = GetFunctionalityTab().Add("NavX Yaw", 0.0).GetEntry();
	voltageEntry_ = GetModeTab().Add("Battery Voltage", 12.5).GetEntry();

	climberRightLimitSwitchEntry_ = GetFunctionalityTab().Add("Right Limit Switch", limitSwitchRight_).WithWidget(frc::BuiltInWidgets::kBooleanBox).GetEntry();
	climberLeftLimitSwitchEntry_ = GetFunctionalityTab().Add("Left Limit Switch", limitSwitchLeft_).WithWidget(frc::BuiltInWidgets::kBooleanBox).GetEntry(); 

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
	resetWristAngleEntry_ = GetSuperstructureTab().Add("reset wrist angle", false).WithWidget(frc::BuiltInWidgets::kToggleButton).GetEntry();

	initLineErrorEntry_ = GetAutoOffsetTab().Add("initiation line distance", 0.0).GetEntry(); 
	trenchDistErrorEntry_ = GetAutoOffsetTab().Add("trench distance", 0.0).GetEntry(); 
	trenchWidthErrorEntry_ = GetAutoOffsetTab().Add("trench width", 0.0).GetEntry();  
	trenchLengthErrorEntry_ = GetAutoOffsetTab().Add("trench length", 0.0).GetEntry(); 
	targetZDistErrorEntry_ = GetAutoOffsetTab().Add("target zone", 0.0).GetEntry(); 
	targetZHeightErrorEntry_ = GetAutoOffsetTab().Add("target zone height", 0.0).GetEntry(); 
	loadingDDistErrorEntry_ = GetAutoOffsetTab().Add("loading dock", 0.0).GetEntry(); 
	playerSt2MidErrorEntry_ = GetAutoOffsetTab().Add("player station midpoint", 0.0).GetEntry(); 
	initLineSlantEntry_ = GetAutoOffsetTab().Add("initiation line slant", 0.0).GetEntry(); 
	autoSequenceEntry_ = GetAutoOffsetTab().Add("input auto sequence", "").GetEntry();
	autoInputSequence_ = autoSequenceEntry_.GetString("");

	GetModeTab().Add("ignore for utah", autoSendableChooser_).WithWidget(frc::BuiltInWidgets::kComboBoxChooser);
	autoSendableChooser_.SetDefaultOption("0 blank", "t 0.0 d 0.0");
	autoSendableChooser_.AddOption("1: Target Zone", GetChosenSequence1());
	autoSendableChooser_.AddOption("2: Loading Bay", GetChosenSequence2());
	autoSendableChooser_.AddOption("3: Mid-Trench", GetChosenSequence3());
	autoSendableChooser_.AddOption("4: Mid-Player Station", GetChosenSequence4());
	autoSendableChooser_.AddOption("5: other", autoInputSequence_);

	std::cout<< "end of drive model constructor" << std::endl;
}

/**
 * Sets drive values given outputs
 * @param left a double
 * @param right a double
 */ 
void RobotModel::SetDriveValues(double left, double right){
    leftMaster_->Set(-left);
    rightMaster_->Set(right);
	leftDrivePower_ = -left;
	rightDrivePower_ = right;
}

/**
 * Sets drive values for a specific set of wheels
 * @param wheel a RobotModel::Wheels
 * @param value a double
 */ 
void RobotModel::SetDriveValues(RobotModel::Wheels wheel, double value) {
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

/**
 * Detects if a collision is detected
 * @return true if collision detected
 */ 
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
	}

	collisionDetected = false; // For testing drive straight

	return collisionDetected;
}

/**
 * Get time
 * @return a double, the time
 */ 
double RobotModel::GetTime(){
    return timer_->Get();
}

/**
 * Get left encoder value
 * @return a double, the left encoder value
 */ 
double RobotModel::GetLeftEncoderValue(){
	//left needs to be negated
	// finds difference from stored value
    return -(leftDriveEncoder_->GetIntegratedSensorPosition() - initialLeftEncoderValue_); 
}

/**
 * Get right encoder value
 * @return a double, the right encoder value
 */ 
double RobotModel::GetRightEncoderValue(){
    return (rightDriveEncoder_->GetIntegratedSensorPosition() - initialRightEncoderValue_);
}

/**
 * Get raw left encoder value
 * @return a double, the raw left encoder value
 */ 
double RobotModel::GetRawLeftEncoderValue() {
	return leftDriveEncoder_->GetIntegratedSensorPosition();
}

/**
 * Get raw right encoder value
 * @return a double, the raw right encoder value
 */ 
double RobotModel::GetRawRightEncoderValue() {
	return rightDriveEncoder_->GetIntegratedSensorPosition();
}

/**
 * Get left distance traveled
 * @return a double, distance in feet
 */ 
double RobotModel::GetLeftDistance() {
	if (isHighGear_) {
		return GetLeftEncoderValue()/HGEAR_ENCODER_TICKS_FOOT;
	} else {
		return GetLeftEncoderValue()/LGEAR_ENCODER_TICKS_FOOT;
	}
}

/**
 * Get right distance traveled
 * @return a double, distance in feet
 */ 
double RobotModel::GetRightDistance() {
	if (isHighGear_) {
		return GetRightEncoderValue()/HGEAR_ENCODER_TICKS_FOOT;
	} else {
		return GetRightEncoderValue()/LGEAR_ENCODER_TICKS_FOOT;
	}
}

/**
 * Get velocity of left wheels
 * @return a double, the velocity
 */ 
double RobotModel::GetLeftVelocity() {
	if (isHighGear_){
		return -10.0*(leftDriveEncoder_->GetIntegratedSensorVelocity()/HGEAR_ENCODER_TICKS_FOOT);
	} else{
		return -10.0*(leftDriveEncoder_->GetIntegratedSensorVelocity()/LGEAR_ENCODER_TICKS_FOOT);
	}
}
 
 /**
 * Get velocity of right wheels
 * @return a double, the velocity
 */ 
double RobotModel::GetRightVelocity() {
   	if (isHighGear_){
		return 10.0*(rightDriveEncoder_->GetIntegratedSensorVelocity()/HGEAR_ENCODER_TICKS_FOOT);
	} else {
		return 10.0*(rightDriveEncoder_->GetIntegratedSensorVelocity()/LGEAR_ENCODER_TICKS_FOOT);
	}
}

/**
 * Resets drive encoders
 */ 
void RobotModel::ResetDriveEncoders() {
	// read curr encoder values and store as initial encoder values
	initialLeftEncoderValue_ = GetRawLeftEncoderValue();
	initialRightEncoderValue_ = GetRawRightEncoderValue();
}

/**
 * Returns true if left encoder has stopped
 * @return true if encoder has stopped
 */ 
bool RobotModel::GetLeftEncoderStopped() {
	if (currLeftVelocity_ < STOP_VELOCITY_THRESHOLD && currLeftVelocity_ > -STOP_VELOCITY_THRESHOLD 
	&& lastLeftVelocity_ < STOP_VELOCITY_THRESHOLD && lastLeftVelocity_ > -STOP_VELOCITY_THRESHOLD) {
		return true;
	}
	return false;
}

/**
 * Returns true if right encoder has stopped
 * @return true if encoder has stopped
 */ 
bool RobotModel::GetRightEncoderStopped() {
	if (currRightVelocity_ < STOP_VELOCITY_THRESHOLD && currRightVelocity_ > -STOP_VELOCITY_THRESHOLD 
	&& lastRightVelocity_ < STOP_VELOCITY_THRESHOLD && lastRightVelocity_ > -STOP_VELOCITY_THRESHOLD) {
		return true;
	}
	return false;
}

/**
 * Starts compressor
 */ 
void RobotModel::StartCompressor() {
	compressor_->Start();
}

/**
 * Gets current voltage
 * @return a double, the current voltage
 */
double RobotModel::GetCurrentVoltage() {
    return pdp_-> GetVoltage();
}

/**
 * Gets total current
 * @return a double, the total current
 */
double RobotModel::GetTotalCurrent(){
    return pdp_->GetTotalCurrent();
}

/**
 * Gets current voltage
 * @return a double, the current voltage
 */
double RobotModel::GetVoltage() {
	return pdp_->GetVoltage();
}

/**
 * Gets current compressor current
 * @return a double, the current compressor current
 */
double RobotModel::GetCompressorCurrent() {
	return compressorCurrent_;
}

/**
 * Gets pressure switch value
 * @return a double
 */
double RobotModel::GetPressureSwitchValue() {
	return 0.0; // fix
}

/**
 * Gets RIO current
 * @return a double
 */
double RobotModel::GetRIOCurrent() {
	return roboRIOCurrent_;
}

/**
 * Gets total power
 * @return a double
 */
double RobotModel::GetTotalPower() {
	return pdp_->GetTotalPower();
}

/**
 * Gets total energy
 * @return a double
 */
double RobotModel::GetTotalEnergy() {
	return pdp_->GetTotalEnergy();
}

/**
 * Gets current NavX yaw
 * @return a double
 */
double RobotModel::GetNavXYaw() {
	return navX_->GetYaw();
}

/**
 * get left motor current
 * @return a double
 */
double RobotModel::GetLeftFunnelMotorStatus(){
    return pdp_->GetCurrent(LEFT_FUNNEL_MOTOR_PDP_CHANNEL);
}

/**
 * get right motor current
 * @return a double
 */
double RobotModel::GetRightFunnelMotorStatus(){
    return pdp_->GetCurrent(RIGHT_FUNNEL_MOTOR_PDP_CHANNEL);
}

/**
 * get feeder motor current
 * @return a double
 */
double RobotModel::GetFeederMotorStatus(){
    return pdp_->GetCurrent(ELEVATOR_FEEDER_MOTOR_PDP_CHANNEL);
}

/**
 * Set robot to prepping state
 * @param desiredVelocity a double
 */
void RobotModel::SetPrepping(double desiredVelocity){
	superstructureController_->SetPreppingState(desiredVelocity);
}

/**
 * Set robot to shooting state
 * @param autoVelocity a double
 */
void RobotModel::SetShooting(double autoVelocity){
	superstructureController_->SetShootingState(autoVelocity);
}

/**
 * Set robot to intaking state
 */
void RobotModel::SetIntaking(){
	superstructureController_->SetIntakingState();
}

/**
 * Set robot to indexing state
 */
void RobotModel::SetIndexing(){
	superstructureController_->SetIndexingState();
}

/**
 * Checks if shooting is done
 * @return true if done
 */
bool RobotModel::GetShootingIsDone(){
	return superstructureController_->GetShootingIsDone();
}

/**
 * Checks motor current over power
 * @param channel an integer
 * @param power a double
 * @return a double with the ratio-ed down power by percent
 */
double RobotModel::CheckMotorCurrentOver(int channel, double power){
    double motorCurrent = GetCurrent(channel);
	if( motorCurrent > MAX_DRIVE_MOTOR_CURRENT){ // current to individual motor is over, TODO change for super
		power = power*MAX_DRIVE_MOTOR_CURRENT / motorCurrent; // ratio down by percent over
	}
	return power;
}

/**
 * Updates current for given channel
 * @param channel an integer
 */
void RobotModel::UpdateCurrent(int channel) {
    /* leftDriveACurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN);
	leftDriveBCurrent_ = pdp_->GetCurrent(LEFT_DRIVE_MOTOR_B_PDP_CHAN);
	rightDriveACurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN);
	rightDriveBCurrent_ = pdp_->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN); */
	// flywheelOneCurrent_ = pdp_->GetCurrent(FLYWHEEL_MOTOR_ONE_PDP_CHAN); // test if this works
	// flywheelTwoCurrent_ = pdp_->GetCurrent(FLYWHEEL_MOTOR_TWO_PDP_CHAN);
	// climbOneCurrent_ = pdp_->GetCurrent(CLIMB_MOTOR_ONE_PDP_CHAN);
	// climbTwoCurrent_ = pdp_->GetCurrent(CLIMB_MOTOR_TWO_PDP_CHAN);
	// intakeRollersCurrent_ = pdp_->GetCurrent(INTAKE_ROLLERS_MOTOR_PDP_CHAN);
	// intakeWristCurrent_ = pdp_->GetCurrent(INTAKE_WRIST_MOTOR_PDP_CHAN);
	// IndexFunnelCurrent_ = pdp_->GetCurrent(INDEX_FUNNEL_MOTOR_PDP_CHAN); //note: if add back in make sure to add both motors
	// elevatorFeederCurrent_ = pdp_->GetCurrent(ELEVATOR_FEEDER_MOTOR_PDP_CHAN);
	// elevatorCurrent_ = pdp_->GetCurrent(ELEVATOR_MOTOR_PDP_CHAN);

	leftDriveACurrent_ = leftMaster_->GetSupplyCurrent(); //works
	leftDriveBCurrent_ = leftMaster_->GetSupplyCurrent();
	rightDriveACurrent_ = rightMaster_->GetSupplyCurrent();
	rightDriveBCurrent_ = rightMaster_->GetSupplyCurrent();

    compressorCurrent_ = compressor_->GetCompressorCurrent();
    roboRIOCurrent_ = frc::RobotController::GetInputCurrent();


	if((GetTotalCurrent() > maxCurrentEntry_.GetDouble(MAX_CURRENT_OUTPUT) || GetVoltage() <= minVoltEntry_.GetDouble(MIN_BROWNOUT_VOLTAGE)) && !lastOver_){
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
	} else if ((GetTotalCurrent() > maxCurrentEntry_.GetDouble(MAX_CURRENT_OUTPUT) || 
		GetVoltage() <= minVoltEntry_.GetDouble(MIN_BROWNOUT_VOLTAGE) && lastOver_)){

		// know compressor is off, because lastOver_ is true
		if(ratioAll_ > MIN_RATIO_ALL_CURRENT){ 
			ratioAll_ *= ratioAll_;
		} else if (ratioSuperstructure_ > MIN_RATIO_SUPERSTRUCTURE_CURRENT){
			ratioSuperstructure_ *= ratioSuperstructure_;
		} else if (ratioDrive_ > MIN_RATIO_DRIVE_CURRENT){
			ratioDrive_ *= ratioDrive_;
		}
		lastOver_ = true;

	} else { 
		if(compressorOff_){
			StartCompressor();
			compressorOff_ = false;
		}

		if(ratioDrive_ + 0.001 < 1.0){
			ratioDrive_ *= 1.01;
		} else if (ratioDrive_ < 1.0){
			ratioDrive_ = 1.0;
		} else if(ratioSuperstructure_ + 0.001 < 1.0){
			ratioSuperstructure_ *= 1.01;
		} else if(ratioSuperstructure_ < 1.0){
			ratioSuperstructure_ = 1.0;
		} else if(ratioAll_ + 0.001 < 1.0){
			ratioAll_ *= 1.01;
		} else if(ratioAll_ < 1.0){
			ratioAll_ /= 1.01;
		} // else don't make it greater than one!
		lastOver_ = false;
	}

	ratioAllEntry_.SetDouble(ratioAll_);
	ratioDriveEntry_.SetDouble(ratioDrive_);
	ratioSuperstructureEntry_.SetDouble(ratioSuperstructure_);

}

/**
 * Gets current of a specific motor
 * @param channel an integer
 * @return current of the motor as a double
 */
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

/**
 * Shifts gear
 */
void RobotModel::GearShift() {
	if (fabs(GetLeftVelocity()) > MAX_LOW_GEAR_VELOCITY && fabs(GetRightVelocity()) > MAX_LOW_GEAR_VELOCITY){
		SetHighGear();
	} else if(fabs(GetLeftVelocity()) < MAX_LOW_GEAR_VELOCITY && fabs(GetRightVelocity()) < MAX_LOW_GEAR_VELOCITY) {
		SetLowGear();
	}
}

/**
 * Modifies the current of a particular motor
 * @param channel an integer
 * @param value a double
 * @return a double, the new power
 */
double RobotModel::ModifyCurrent(int channel, double value){

    double power = value*ratioAll_;
	double individualPowerRatio = power;
	double tempPowerRatio;

	switch(channel){
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
		default:
			printf("WARNING: current not found to modify.  In ModifyCurrents() in RobotModel.cpp");
	}
	return power;
}

/**
 * Sets robot to high gear
 */
void RobotModel::SetHighGear(){
	if (isHighGear_ == false) {
		gearSolenoid_ -> Set(false);
		isHighGear_ = true;
	}
}

/**
 * Sets robot to low gear
 */
void RobotModel::SetLowGear(){
	if (isHighGear_ == true) {
		gearSolenoid_ -> Set(true);
		isHighGear_ = false;
	}
}

/**
 * Checks if robot is in high gear
 * @return true if in high gear
 */
bool RobotModel::IsHighGear(){
	return isHighGear_;
}

/**
 * Sets change in angle
 * @param angle a double
 */
void RobotModel::SetDeltaAngle(double angle) {
	desiredDeltaAngle_ = angle;
}

/**
 * Sets distance from target
 * @param distance a double
 */
void RobotModel::SetDistance(double distance) {
	desiredDistance_ = distance;
}

/**
 * Gets change in angle
 * @return angle, a double
 */
double RobotModel::GetDeltaAngle() {
	return desiredDeltaAngle_;
}

/**
 * Gets distance from target
 * @return distance a double
 */
double RobotModel::GetDistance() {
	return desiredDistance_;
}

/**
 * Zeroes the navX Yaw
 */
void RobotModel::ZeroNavXYaw() {
	navX_->ZeroYaw();
}

/**
 * Gets pitch
 * @return double, the pitch
 */
double RobotModel::GetNavXPitch() {
	return navX_->GetPitch();
}

/**
 * Gets roll
 * @return double, the roll
 */
double RobotModel::GetNavXRoll() {
	return navX_->GetRoll();
}

/**
 * Gets driverTab from shuffleboard
 * @return driverTab_
 */
frc::ShuffleboardTab& RobotModel::GetDriverTab(){
    return driverTab_;
}

/**
 * Gets modeTab from shuffleboard
 * @return modeTab_
 */
frc::ShuffleboardTab& RobotModel::GetModeTab(){
    return modeTab_;
}

/**
 * Gets functionalityTab from shuffleboard
 * @return functionalityTab_
 */
frc::ShuffleboardTab& RobotModel::GetFunctionalityTab(){
    return functionalityTab_;
}

/**
 * Gets pidTab from shuffleboard
 * @return pidTab_
 */
frc::ShuffleboardTab& RobotModel::GetPIDTab(){
    return pidTab_;
}

/**
 * Gets autoOffsetTab from shuffleboard
 * @return autoOffsetTab_
 */
frc::ShuffleboardTab& RobotModel::GetAutoOffsetTab(){
    return autoOffsetTab_;
}

/**
 * Gets superstructureTab from shuffleboard
 * @return superstructureTab_
 */
frc::ShuffleboardTab& RobotModel::GetSuperstructureTab(){
    return superstructureTab_;
}

/**
 * Gets the test sequence
 * @return the test sequence as a string
 */
std::string RobotModel::GetTestSequence() {
	return testSequence_;
}

/**
 * Sets the test sequence
 * @param testSequence an std::string
 */
void RobotModel::SetTestSequence(std::string testSequence) {
	testSequence_ = testSequence;
}

/**
 * Sets the last pivot angle
 * @param value a double, the last pivot angle
 */
void RobotModel::SetLastPivotAngle(double value){
	lastPivotAngle_ = value;
}

/**
 * Gets the last pivot angle
 * @return the last pivot angle as a double
 */
double RobotModel::GetLastPivotAngle(){
	return lastPivotAngle_;
}

/**
 * Initializes ZMQ
 */
void RobotModel::ZMQinit(){
	//zmq
    if (context_ == nullptr) {
        context_ = new zmq::context_t(2); // same context for send + receive zmq
        publisher_ = new zmq::socket_t(*context_, ZMQ_PUB);
        subscriber_ = new zmq::socket_t(*context_, ZMQ_SUB);
        ConnectRecvZMQ();
        ConnectSendZMQ();
    }
}

/**
 * Connects to zmq socket to receive from jetson
 */ 
void RobotModel::ConnectRecvZMQ() {
    try {
		printf("in try connect to jetson\n");
        //change to dynamic jetson address
        int confl = 1;
        subscriber_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl));
        subscriber_->setsockopt(ZMQ_RCVTIMEO, 1000);
        subscriber_->connect("tcp://10.18.68.12:5808");
        subscriber_->setsockopt(ZMQ_SUBSCRIBE, "", 0); //filter for nothing
    } catch(const zmq::error_t &exc) {
		printf("ERROR: TRY CATCH FAILED IN ZMQ CONNECT RECEIVE\n");
		std::cerr << exc.what();
	}
    std::cout << "reached end of connect recv zmq\n" << std::flush;
}

/**
 * Updates ZMQ
 */ 
void RobotModel::UpdateZMQ(){ 
	std::string temp = ReadZMQ();
    hasContents_ = !ReadAll(temp);
}

/**
 * Returns true if ZMQ has contents
 * @return true if ZMQ has contents
 */ 
bool RobotModel::ZMQHasContents(){
	return hasContents_;
}

/**
 * Returns the ZMQ contents
 * @return the contents of ZMQ as a string
 */ 
std::string RobotModel::ReadZMQ() {
    std::string contents;
    zmq::message_t m;
    subscriber_->recv(&m, ZMQ_NOBLOCK);
    contents = std::string(static_cast<char*>(m.data()), m.size());
    return contents;
}

/**
 * Split up the ZMQ contents into something readable
 */ 
bool RobotModel::ReadAll(std::string contents) {
    
    std::stringstream ss(contents); // split string contents into a vector
	std::vector<std::string> result;
    bool abort;

	while (ss.good()) {
		std::string substr;
		getline( ss, substr, ' ' );
		if (substr == "") {
			continue;
		}
		result.push_back( substr );
	}

    // jetson string is hasTarget, angle (deg from center), raw distance (ft)
	if(result.size() > 2) {
        double angle = stod(result.at(1));
        double distance = stod(result.at(2));
		SetDeltaAngle(angle);
		SetDistance(distance); // 1.6;
        abort = false;
	} else {
		abort = true;
		SetDeltaAngle(0.0);
	}

    return abort;

}

/**
 * ZMQ socket to send message to jetson
 */ 
void RobotModel::ConnectSendZMQ() {
    try{
        if(!isSocketBound_){
            publisher_->bind("tcp://*:5807");
            isSocketBound_ = true;
        }
        int confl = 1;
        publisher_->setsockopt(ZMQ_CONFLATE, &confl, sizeof(confl));
    } catch (const zmq::error_t &exc) {
		printf("TRY CATCH FAILED IN ZMQ CONNECT SEND\n");
		std::cerr << exc.what();
	}

}

/**
 * Send messages to driverstation
 * @param lowExposure a boolean
 */ 
void RobotModel::SendZMQ(bool lowExposure) {
	std::string message =
		std::to_string(lowExposure) +
		" targetRPM:" +
		std::to_string((int)(superstructureController_->CalculateFlywheelVelocityDesired())) +
		" RPM:" + 
		std::to_string((int)(GetFlywheelMotor1Velocity()*FALCON_TO_RPM));

	if (GetElevatorFeederLightSensorStatus() && GetElevatorLightSensorStatus() && GetFunnelLightSensorStatus()){
		message += " FULL ELEVATOR!!";
	}

    int sent = zmq_send((void *)*publisher_, message.c_str(), message.size(), 0);
}

/**
 * Calculates desired flywheel velocity
 * @return flywheel velocity, a double
 */ 
double RobotModel::CalculateFlywheelVelocityDesired(){
	return superstructureController_->CalculateFlywheelVelocityDesired();
}

/**
 * Create the NavX
 */ 
void RobotModel::CreateNavX(){
	navXSource_ = new NavXPIDSource(this);
}

/**
 * Get the NavX
 * @return navXSource_
 */ 
NavXPIDSource* RobotModel::GetNavXSource(){
	return navXSource_;
}

/**
 * Refreshes shuffleboard values
 */ 
void RobotModel::RefreshShuffleboard(){
	
	lastLeftEncoderValue_ = currLeftEncoderValue_;
    lastRightEncoderValue_ = currRightEncoderValue_;
    currLeftEncoderValue_ = GetLeftEncoderValue();
    currRightEncoderValue_ = GetRightEncoderValue();
	lastVelocTime_ = currVelocTime_;
	currVelocTime_ = GetTime();
	lastLeftDistance_ = currLeftDistance_;
	lastRightDistance_ = currRightDistance_;
	currLeftDistance_ = GetLeftDistance();
	currRightDistance_ = GetRightDistance();
	
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
 
	leftCurrentEntry_.SetDouble(leftDriveACurrent_);
	rightCurrentEntry_.SetDouble(rightDriveACurrent_);
	resetWristAngle_ = resetWristAngleEntry_.GetBoolean(false);
	if (resetWristAngle_) {
		ResetWristAngle();
	}

	initLineError_ = initLineErrorEntry_.GetDouble(0.0);
	trenchDistError_ = trenchDistErrorEntry_.GetDouble(0.0);
	trenchWidthError_ = trenchWidthErrorEntry_.GetDouble(0.0);
	trenchLengthError_ = trenchLengthErrorEntry_.GetDouble(0.0);
	targetZDistError_ = targetZDistErrorEntry_.GetDouble(0.0);
	loadingDDistError_ = loadingDDistErrorEntry_.GetDouble(0.0);
	playerSt2MidError_ = playerSt2MidErrorEntry_.GetDouble(0.0);
}

/**
 * Destructor
 */ 
RobotModel::~RobotModel(){
	
	/*delete climberRatchetSolenoid_;
	delete flywheelHoodSolenoid_;
	delete gearSolenoid_;
	delete lightSolenoid_;
	delete compressor_;
	delete timer_;
	delete navX_;
	delete pdp_;
	
	//motors
	delete leftMaster_;
	delete rightMaster_;
	delete leftSlaveA_;
	delete rightSlaveA_;
	delete flywheelMotor1_;
	delete flywheelMotor2_;
	delete climberLeftElevatorMotor_;
	delete climberRightElevatorMotor_;
	delete intakeRollersMotor_;
	delete intakeWristMotor_;
	delete indexFunnelMotor_;
	delete elevatorFeederMotor_;
	delete elevatorMotor_;
	delete controlPanelMotor_;

	delete elevatorFeederLightSensor_;
	delete elevatorLightSensor_;
	delete colorSensor_;
	delete limitSwitchRight_;
	delete limitSwitchLeft_;

	
	delete fortyAmpFXLimit_;
	delete fortyAmpSRXLimit_;
	delete thirtyAmpSRXLimit_; */

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
	climberRightLimitSwitchEntry_.Delete();
	climberLeftLimitSwitchEntry_.Delete();

	initLineErrorEntry_.Delete();
	trenchDistErrorEntry_.Delete();
	trenchWidthErrorEntry_.Delete();
	trenchLengthErrorEntry_.Delete();
	targetZDistErrorEntry_.Delete();
	targetZHeightErrorEntry_.Delete();
	loadingDDistErrorEntry_.Delete();
	playerSt2MidErrorEntry_.Delete();
	initLineSlantEntry_.Delete();
	

	bColorEntry_.Delete();
	rColorEntry_.Delete();
	gColorEntry_.Delete();
	resetWristAngleEntry_.Delete();
	autoSequenceEntry_.Delete();
}
