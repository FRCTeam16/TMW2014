// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in th future.
#include "OI.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "SmartDashboard/SmartDashboard.h"
#include "Commands/AutonomousCommand.h"
#include "Commands/Command1.h"
#include "Commands/InitGyro.h"
#include "Commands/SetCamOffsets.h"
#include "Commands/SetWheelOffsets.h"
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
OI::OI() {
	// Process operator interface input here.
	pi = 3.14159;
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	gamePad = new Joystick(3);
	
	driverJoystickLeft = new Joystick(2);
	
	driverJoystickRight = new Joystick(1);
	
     
        // SmartDashboard Buttons
	SmartDashboard::PutData("Autonomous Command", new AutonomousCommand());
	SmartDashboard::PutData("SetWheelOffsets", new SetWheelOffsets());
	SmartDashboard::PutData("InitGyro", new InitGyro());
	SmartDashboard::PutData("SetCamOffsets", new SetCamOffsets());
	SmartDashboard::PutData("Command 1", new Command1());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
}
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
Joystick* OI::getGamePad() {
	return gamePad;
}
Joystick* OI::getDriverJoystickLeft() {
	return driverJoystickLeft;
}
Joystick* OI::getDriverJoystickRight() {
	return driverJoystickRight;
}
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
double OI::getScaledJoystickRadians() {
	double steerAngle = pi/2;
	steerAngle = driverJoystickRight->GetDirectionRadians();
	if(steerAngle > pi)
		steerAngle = pi;
	if(steerAngle < -pi)
		steerAngle = -pi;
	
	if(steerAngle < -pi/2)
		steerAngle = -pi/2 - steerAngle;
	else if(steerAngle < pi/2)
		steerAngle = pi/2 + steerAngle;
	else
		steerAngle = 3*pi/2 - steerAngle;
//	scalingFactor = driverJoystick->GetTwist()/2+1.5;
	
	return scaledRadians(steerAngle);
}
double OI::getJoystickMagnitude() {
	if(driverJoystickRight->GetMagnitude() < .1)
		return 0;
	else
		if (driverJoystickRight->GetY()<0)
			return -driverJoystickRight->GetMagnitude();
		else
			return driverJoystickRight->GetMagnitude();
}
double OI::getJoystickTwist() {
	if(fabs(driverJoystickLeft->GetX()) < .1)
			return 0;
	else
		return driverJoystickLeft->GetX()/2;
}
double OI::getJoystickX() {
	if (fabs(driverJoystickRight->GetX()) < 0)
		return 0;
	else
		return driverJoystickRight->GetX();
}
double OI::getJoystickY() {
	if (fabs(driverJoystickRight->GetY()) < 0)
		return 0;
	else
		return driverJoystickRight->GetY();
}
double OI::getLeftJoystickXRadians() {
	if(fabs(driverJoystickLeft->GetX())<.00)
		return pi/2;
	else
		return scaledRadians(pi/2 + driverJoystickLeft->GetX()*pi/2);
}
double OI::scaledRadians(double radians) {
	double scaledradians = pi/2;
	double scalingFactor = 1.8;
	if(radians <= pi/2)
		scaledradians = (-(pi/2)/pow(pow(-pi/2,2),scalingFactor/2))*pow(pow(radians-pi/2,2),scalingFactor/2) + pi/2;
	else //if(steerAngle <= pi)
		scaledradians = ((pi/2)/pow((pi/2),scalingFactor))*pow(radians-pi/2,scalingFactor) + pi/2;
	
	return scaledradians;
}
