/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
/*      Utility functions for logging state of robot
 *
 *      Action log: called by two macros (LOG and DUMP).
 *      - Puts value descriptions and the corresponding value, with a timestamp,
 *        to a file on the roboRIO that can be accessed through psftp.
 *
 *      Data log:
 *      - Continuously records different values such as encoders, button states, joystick inputs,
 *        motor outputs and current draw
 *
 *      Log is in csv format - easily read by programs like excel, google sheets, etc.
 */

/*
 * Downloading logs from roboRIO: (from forummotion post)
 * 1. open command line (on windows) or terminal (on mac)
 * 2. make sure you are connected to the robot, and type:
 * 			psftp lvuser@roborio-1868-frc.local
 * 	  (you need psftp installed on your laptop)
 * 3. some stuff should come up, if it asks you to store something you can either say y or n.
 * 	  i did y and i'm still alive so it should be fine
 * 4. to see all the files in the directory, type
 *    		dir
 * 5. you should see a file with the format
 * 			YEAR-month-day_hour_minute-actionlog.txt
 * 	  or
 * 	  		YEAR-month-day_hour_minute-datalog.txt
 * 6. type CODE:
 *			get FILENAME
 *	  where FILENAME is the thing mentioned above. type it exactly right!!!
 * 7. if you did it right, it should say that the file was downloaded into your directory or something
 * 8. on windows, navigate to C:\Users\blah where blah is the name of your user on your computer.
 *    the file should be in there somewhere.
 *    on mac, i don't know exactly but it should be under something similar
 * 9. yay you did it! you can do whatever you want with the file now, it should follow a format of
 *    TIMESTAMP, FILENAME, LINE, STATENAME, STATE if you used the action log.
 *    for the data log, it's constantly changing so it's no use putting here but we're working on
 *    added headers to the columns so you can easily tell what each column is about.
 *    check Logger::LogState(...) in Logger.cpp if you're curious
 *
 *
 * TO DELETE ACTION AND DATALOGS:
 * in putty:
 * rm *datalog.txt
 * rm *actionlog.txt
 */

#include "RobotModel.h"
#include "ControlBoard.h"
#include <fstream>
#include <string>
#include <ctime>


#define LOG(robot, stateName, state) {MATCH_PERIODIC(1, Logger::LogAction(robot, __FILE__, __LINE__, stateName, state))}
#define DUMP(stateName, state) {MATCH_PERIODIC(1, Logger::LogAction(__FILE__, __LINE__, stateName, state))}

class Logger {
public:

	/**
	 * Log state: records the physical state of the robot and human control
	 * @param robot a RobotModel
	 * @param humanControl a ControlBoard
	 */ 
	static void LogState(RobotModel *robot, ControlBoard *myHumanControl);

	/**
	 * Log action: records higher-level processes
	 * @param robot a RobotModel
	 * @param fileName a reference to an std::string
	 * @param line an integer
	 */ 
	static void LogAction(RobotModel* robot, const std::string& fileName, int line,
				const std::string& stateName, double state);
	
	/**
	 * Log action: records higher-level processes
	 * @param robot a RobotModel
	 * @param fileName a reference to an std::string
	 * @param line an integer
	 * @param stateName a reference to an std::string
	 * @param state a reference to an std::string
	 */ 
	static void LogAction(RobotModel* robot, const std::string& fileName, int line,
				const std::string& stateName, const std::string& state);

	// Without timestamp
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			bool state);
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			double state);
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			const std::string& state);


	/**
	 * Gets time stamp
	 * @return the time as a string
	 */ 
	static std::string GetTimeStamp(const char* fileName);

	/**
	 * Closes the log
	 */ 
	static void CloseLogs();

private:
	static std::ofstream logData, logAction;

};
