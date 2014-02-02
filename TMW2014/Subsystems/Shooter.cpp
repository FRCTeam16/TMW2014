// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in th future.
#include "Shooter.h"
#include "../Robotmap.h"
BSTimer* Shooter::fireTimer=NULL;
Shooter::Shooter() : Subsystem("Shooter") {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	windowMotors = RobotMap::shooterWindowMotors;
	camLeft = RobotMap::shooterCamLeft;
	camRight = RobotMap::shooterCamRight;
	camPos = RobotMap::shooterCamPos;
	fingers = RobotMap::shooterFingers;
	ballPresent = RobotMap::shooterballPresent;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	camController = RobotMap::shooterCamController;
	fireFlag = false;
	camMotorRatio = .80;
	CamPIDOut* CamPID;
	CamPID = RobotMap::shooterCamOut;
	CamPID->SetMultiplier(camMotorRatio);
	stage1Voltage = 1.5;
	camPosOffset = 1.75;
	fireTimer = new BSTimer();
	fireTimer->Start();
	fireDelayTimer = new BSTimer();
	fireDelayTimer->Start();
	fireDelayFlag = false;
}
    
void Shooter::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	//SetDefaultCommand(new MySpecialCommand());
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
void Shooter::CamChecker() {
	if(fireDelayTimer->HasPeriodPassed(.3) && fireDelayFlag){
		fireFlag = true;
		fireTimer->Reset();
		fireDelayFlag = false;
		}
	if(fireFlag){
		if(!fireTimer->HasPeriodPassed(.5) || CorrectVoltage(camPos->GetAverageVoltage() - camPosOffset) > stage1Voltage) {
			RunCams(1);
		}
		
		else if(!camController->IsEnabled()){
			camController->Enable();
			camController->SetSetpoint(CorrectVoltage(0 + camPosOffset));
		}
		
		if((camController->OnTarget() && camController->IsEnabled()) || fireTimer->HasPeriodPassed(3)) {
			camController->Disable();
			fireFlag = false;
			RunCams(0);
		}
	}
	else {
		RunCams(0);
	}
}
void Shooter::Fire() {
	if(!fireDelayFlag && !ballPresent->Get()) {
		fireDelayTimer->Reset();
		fireDelayFlag = true;
	}
}

void Shooter::AutoFire() {
	if(!fireFlag) {
		fireFlag = true;
		fireTimer->Reset();
	}
}
void Shooter::RunCams(float output) {
	windowMotors->Set(-output);
	camLeft->Set(-camMotorRatio*output);
	camRight->Set(-camMotorRatio*output);
}
bool Shooter::GetFiring() {
	return fireFlag && fireDelayFlag;
}
float Shooter::CorrectVoltage(float setpoint) {
	
	if (setpoint < 0)
	{
		return setpoint + 5;
	}
	else if (setpoint > 5)
	{
		return setpoint - 5;
	}
	else if (setpoint == 5)
	{
		return 0;
	}
	else
	{
		return setpoint;
	}
}
float Shooter::GetCorrectedCamPos(){
	return (CorrectVoltage(camPos->GetAverageVoltage()-camPosOffset));
}
