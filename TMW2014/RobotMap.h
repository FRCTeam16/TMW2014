// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in th future.
#ifndef ROBOTMAP_H
#define ROBOTMAP_H
#include "WPILib.h"
#include "CamPIDOut.h"
#include "BSGyro.h"
#include "CrabSpeed.h"
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
class RobotMap {
public:
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	static SpeedController* driveTrainFrontLeftDrive;
	static SpeedController* driveTrainFrontRightDrive;
	static SpeedController* driveTrainRearLeftDrive;
	static SpeedController* driveTrainRearRightDrive;
	static AnalogChannel* driveTrainFrontLeftPos;
	static CANJaguar* driveTrainFrontLeftSteer;
	static PIDController* driveTrainFrontLeft;
	static AnalogChannel* driveTrainFrontRightPos;
	static CANJaguar* driveTrainFrontRightSteer;
	static PIDController* driveTrainFrontRight;
	static AnalogChannel* driveTrainRearLeftPos;
	static CANJaguar* driveTrainRearLeftSteer;
	static PIDController* driveTrainRearLeft;
	static AnalogChannel* driveTrainRearRightPos;
	static CANJaguar* driveTrainRearRightSteer;
	static PIDController* driveTrainRearRight;
	static Encoder* driveTrainWheelX;
	static Encoder* driveTrainWheelY;
	static Solenoid* driveTrainWheelLock;
	static DigitalInput* driveTrainTargetLeft;
	static DigitalOutput* driveTrainSendTarget;
	static CANJaguar* shooterWindowMotors;
	static CANJaguar* shooterCamLeft;
	static CANJaguar* shooterCamRight;
	static AnalogChannel* shooterCamPos;
	static AnalogChannel* shooterBackupCamPos;
	static Solenoid* shooterFingers;
	static DigitalInput* shooterBallPresent;
	static CANJaguar* pickupBeaterBar;
	static Compressor* pickupComp;
	static Solenoid* pickupBeaterBarOut;
	static Solenoid* pickupWings;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	
	//static CANJaguar* pickupBeaterBar;
	static BSGyro* driveTrainGyro;
	static CamPIDOut* shooterCamOut;
	static PIDController* shooterCamController;
//	static PIDController* shooterBackupCamController;
	
	static CrabSpeed* CrabSpeedX;
	static CrabSpeed* CrabSpeedY;
	static CrabSpeed* CrabSpeedTwist;
	
	static PIDController* driveTrainDriveControlX;
	static PIDController* driveTrainDriveControlY;
	static PIDController* driveTrainDriveControlTwist;
	
	static void init();
};
#endif
