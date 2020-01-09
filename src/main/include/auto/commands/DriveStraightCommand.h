/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>
#include <math.h>
#include "RobotModel.h"
#include "../PIDSource/PIDInputSource.h"
#include "../PIDSource/PIDOutputSource.h"
#include "../AutoCommand.h"
#include <networktables/NetworkTableEntry.h>

class DriveStraightCommand : public AutoCommand {
  public:
    /**
     * Constructor that generates the DriveStraight Command
     * @param navXSource angle input for the PID Loop
     * @param talonEncoderSource distance input for distance PID Loop
     * @param anglePIDOutput output for the angle PID Loop
     * @param robot robot model
     * @param distance in feet
     */
    DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
        AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
        double desiredDistance);
      /**
       * Constructor that generates the DriveStraight Command
       * @param navXSource angle input for the PID Loop
       * @param talonEncoderSource distance input for distance PID Loop
       * @param anglePIDOutput output for the angle PID Loop
       * @param robot robot model
       * @param distance in feet
       * @param angle the robot should be
       */
    DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
          AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
          double desiredDistance, double absoluteAngle);

    /**
     * Destructor
     */

    virtual ~DriveStraightCommand();

    /**
     * Checks if the PID is on target or if it times out, then sets isDone_ is true and stops the motors. Otherwise, sets the right and
     * left motors to the PID outputs.
     */

    void Init();

    /**
     * Runs the DriveStraight PIDs. Times out after 3 seconds for now.
     */
    void Update(double currTime, double deltaTime);

    /**
     * @return isDone_
     */

    bool IsDone();

    /**
     * Disables the PIDControllers and make the command done.
     */

    void Reset();

    /**
     * Gets PID values from the ini file
     */
    void GetIniValues();

private:
	void Initializations(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
			AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
			double desiredDistance);

	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonEncoderSource_;
	AnglePIDOutput *anglePIDOutput_;
	DistancePIDOutput *distancePIDOutput_;
	PIDController *anglePID_;
	PIDController *distancePID_;
	RobotModel *robot_;

	bool isAbsoluteAngle_;
	double rPFac_, rIFac_, rDFac_;
	double rMaxOutput_, rTolerance_;
	double dPFac_, dIFac_, dDFac_;
	double dMaxOutput_, dTolerance_;
	double desiredAngle_;
	double initialAvgDistance_;
	double desiredDistance_;
	double desiredTotalAvgDistance_;
	double leftMotorOutput_, rightMotorOutput_;
	double initialDriveTime_, diffDriveTime_, driveTimeoutSec_;
	bool isDone_;
	int numTimesOnTarget_;
	double lastDOutput_;
	double lastDistance_;

	int numTimesStopped_;

  frc::ShuffleboardLayout &driveStraightLayout_, &driveStraightPIDLayout_, &anglePIDLayout_, &distancePIDLayout_;
	nt::NetworkTableEntry leftStraightEntry_, rightStraightEntry_, angleErrorEntry_, angleErrorGraphEntry_, desiredAngleEntry_,
		encoderErrorEntry_, encoderErrorGraphEntry_, desiredTotalFeetEntry_, dPIDOutputEntry_, aPIDOutputEntry_, aPEntry_,
    aIEntry_, aDEntry_, dPEntry_, dIEntry_, dDEntry_;

};
