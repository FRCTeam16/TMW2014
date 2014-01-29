// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in th future.
#ifndef SHOOTER_H
#define SHOOTER_H
#include "Commands/Subsystem.h"
#include "WPILib.h"
#include "../CamPIDOut.h"
#include "../BSTimer.h"
/**
 *
 *
 * @author ExampleAuthor
 */
class Shooter: public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
public:
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	CANJaguar* windowMotors;
	CANJaguar* camLeft;
	CANJaguar* camRight;
	AnalogChannel* camPos;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	PIDController* camController;
	Shooter();
	void InitDefaultCommand();
	void CamChecker();
	void Fire();
	bool GetFiring();
	float GetCorrectedCamPos();
	static BSTimer* fireTimer;
	void RunCams(float output);
	
private:
	bool fireFlag;
	float camMotorRatio;
	float stage1Voltage;
	float camPosOffset;
	float CorrectVoltage(float setpoint);
};
#endif
