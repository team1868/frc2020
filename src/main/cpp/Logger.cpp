/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Logger.h"
#include "Ports2020.h"

std::ofstream Logger::logData;
std::ofstream Logger::logAction;
#define USE_NAVX = true;

void Logger::LogState(RobotModel* myRobot, ControlBoard *myHumanControl) {
	if (!logData.is_open()) {
		 logData.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_datalog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
		    logData << "Time, Left Encoder, Right Encoder, Left Wheel Speed,"
		        << "Right Wheel Speed, Yaw, Roll, Pitch, Voltage, Total Current, "
		        << "Left Drive A Current, Left Drive B Current, Right Drive A Current, Right Drive B Current, "
		        << "Flywheel One Current, Flywheel Two Current, Climber One Current, Climber Two Current, "
				<< "Intake Rollers Current, Intake Wrist Current, "
				<< "Funnel Index Current, Elevator Bottom Current, Elevator Top Current, "
		        << "Compressor Current, "
		        << "RoboRIO Current, Total Power, Total Energy, Pressure, "
		        << "Left Joy X, Left Joy Y, "
		        << "Right Joy X, Right Joy Y, Reverse (currently not in), Arcade, "
		        << "Quick Turn Desired \r\n";
				// fix labels
	}

	logData << myRobot->GetTime() << ", " <<
	    myRobot->GetLeftEncoderValue() << ", " <<
	    myRobot->GetRightEncoderValue() << ", " <<
	    //   myRobot->GetWheelSpeed(RobotModel::kLeftWheels) << ", " <<
	    //   myRobot->GetWheelSpeed(RobotModel::kRightWheels) << ", " <<
	    myRobot->GetNavXYaw() << ", " <<
	    myRobot->GetNavXRoll() << ", " <<
	    myRobot->GetNavXPitch() << ", " <<
	    
		myRobot->GetVoltage() << ", " <<
	    myRobot->GetTotalCurrent() << ", " <<
	    myRobot->GetCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN) << ", " <<
	    myRobot->GetCurrent(LEFT_DRIVE_MOTOR_B_PDP_CHAN) << ", " <<
	    myRobot->GetCurrent(RIGHT_DRIVE_MOTOR_A_PDP_CHAN) << ", " <<
	    myRobot->GetCurrent(RIGHT_DRIVE_MOTOR_B_PDP_CHAN) << ", " <<
		
		myRobot->GetCurrent(FLYWHEEL_MOTOR_ONE_PDP_CHAN) << ", " <<
		myRobot->GetCurrent(FLYWHEEL_MOTOR_TWO_PDP_CHAN) << ", " <<
		myRobot->GetCurrent(CLIMB_MOTOR_ONE_PDP_CHAN) << ", " <<
		myRobot->GetCurrent(CLIMB_MOTOR_TWO_PDP_CHAN) << ", " <<
		myRobot->GetCurrent(INTAKE_ROLLERS_MOTOR_PDP_CHAN) << ", " <<
		myRobot->GetCurrent(INTAKE_WRIST_MOTOR_PDP_CHAN) << ", " <<
		myRobot->GetCurrent(INDEX_FUNNEL_MOTOR_PDP_CHAN) << ", " <<
		myRobot->GetCurrent(ELEVATOR_MOTOR_PDP_CHAN) << ", " <<

	    myRobot->GetCompressorCurrent() << ", " <<
	    myRobot->GetRIOCurrent() << ", " <<
	    myRobot->GetTotalPower() << ", " <<
	    myRobot->GetTotalEnergy() << ", " <<
	    myRobot->GetPressureSwitchValue() << ", " <<
		myHumanControl->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kX) << ", " <<
		myHumanControl->GetJoystickValue(ControlBoard::kLeftJoy, ControlBoard::kY) << ", " <<
	    myHumanControl->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kX) << ", " <<
	    myHumanControl->GetJoystickValue(ControlBoard::kRightJoy, ControlBoard::kY);
	  logData.flush();
}
/* format:
 * robotmodel state / controlboard state
 *
 * ie:
 *
 * timer / left motor / right motor / gear shift / pdp voltage / leftjoy x / leftjoy y
 * 			/ rightjoy x / rightjoy y / reverse desired / gearshift desired /
 */

/* overloaded methods with time stamp */

void Logger::LogAction(RobotModel* myRobot, const std::string& fileName, int line,
				const std::string& stateName, double state) {
	logAction.flush();
	if (!logAction.is_open()) {
			logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << myRobot->GetTime() << ", " << fileName << ", " << line << ", " << stateName
			<< ", " << state << "\r\n";
	logAction.flush();
}


void Logger::LogAction(RobotModel* myRobot, const std::string& fileName, int line,
				const std::string& stateName, const std::string& state) {
	if (!logAction.is_open()) {
			logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << myRobot->GetTime() << ", " << fileName << ", " << line << ", " << stateName
			<< ", " << state << "\r\n";
	logAction.flush();
}


/* overloaded methods without time stamp */
void Logger::LogAction(const std::string& fileName, int line, const std::string& stateName,
			bool state) {
	if (!logAction.is_open()) {
		logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
	logAction.flush();
}


void Logger::LogAction(const std::string& fileName, int line, const std::string& stateName,
			double state) {
	if (!logAction.is_open()) {
		logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
	logAction.flush();
}


void Logger::LogAction(const std::string& fileName, int line, const std::string& stateName,
			const std::string& state) {
	if (!logAction.is_open()) {
		logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
	logAction.flush();
}


void Logger::CloseLogs() {
	logData.close();
	logAction.close();
}


std::string Logger::GetTimeStamp(const char* fileName) {
/*	struct timespec tp;
	clock_gettime(CLOCK_REALTIME,&tp);
	double realTime = (double)tp.tv_sec + (double)((double)tp.tv_nsec*1e-9);
*/
	time_t rawtime = time(0);
	struct tm * timeinfo;	// get current time
	char buffer [80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);	// converts time_t to tm as local time
	strftime (buffer, 80, fileName, timeinfo); // fileName contains %F_%H_%M

	return buffer;
}