// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in th future.
#include "Robot.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INITIALIZATION
DriveTrain* Robot::driveTrain = 0;
Shooter* Robot::shooter = 0;
Pickup* Robot::pickup = 0;
OI* Robot::oi = 0;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INITIALIZATION
void Robot::RobotInit() {
	RobotMap::init();
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	driveTrain = new DriveTrain();
	shooter = new Shooter();
	pickup = new Pickup();
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	// This MUST be here. If the OI creates Commands (which it very likely
	// will), constructing it during the construction of CommandBase (from
	// which commands extend), subsystems are not guaranteed to be
	// yet. Thus, their requires() statements may grab null pointers. Bad
	// news. Don't move it.
	oi = new OI();
	lw = LiveWindow::GetInstance();
	// instantiate the command used for the autonomous period
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
	autonomousCommand = new AutonomousCommand();
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
	prevTrigger = false;
	
	File = RAWCConstants::getInstance();
	Robot::driveTrain->SetWheelbase(20.625/2, 20.625, 20.625/2);
	FLOffset = File->getValueForKey("FLOff");
	FROffset = File->getValueForKey("FROff");
	RLOffset = File->getValueForKey("RLOff");
	RROffset = File->getValueForKey("RROff");
	Robot::driveTrain->SetOffsets(FLOffset, FROffset, RLOffset, RROffset);
	
	Prefs = Preferences::GetInstance();
	if(!Prefs->ContainsKey("CamMotorScaling"))
		Prefs->PutFloat("CamMotorScaling",.53);
	WindowMotorSetTask = new JaguarSetTask("WindowMotors",Robot::shooter->windowMotors);
}
void Robot::DisabledPeriodic() {
		SMDB();
	
	Scheduler::GetInstance()->Run();
}
	
void Robot::AutonomousInit() {
	if (autonomousCommand != NULL)
		autonomousCommand->Start();
}
	
void Robot::AutonomousPeriodic() {
	Scheduler::GetInstance()->Run();
}
	
void Robot::TeleopInit() {
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to 
	// continue until interrupted by another command, remove
	// this line or comment it out.
	autonomousCommand->Cancel();
}
	
void Robot::TeleopPeriodic() {
	SMDB();
		

/******************DRIVETRAIN**************************************/	
	
	//Resets gyro to zero when crab starts
	if (!prevTrigger && Robot::oi->getDriverJoystickRight()->GetRawButton(1))
		Robot::driveTrain->gyro->Reset();
	
	prevTrigger = Robot::oi->getDriverJoystickRight()->GetRawButton(1);
	if(Robot::oi->getDriverJoystickRight()->GetRawButton(1))
	{//Crab	
		Robot::driveTrain->Crab(Robot::oi->getJoystickTwist(),-Robot::oi->getJoystickY(),Robot::oi->getJoystickX(),true);	
	}
	else 
	{//Steering
		//Robot::driveTrain->Steer(Robot::oi->getScaledJoystickRadians(),Robot::oi->getJoystickMagnitude(),0.5);
		Robot::driveTrain->Steer(Robot::oi->getLeftJoystickXRadians(),Robot::oi->getJoystickY(),0.5);
	}

/******************SHOOTER**************************************/	

	//if(Robot::shooter->windowMotors->IsAlive())
		//Robot::shooter->windowMotors->Set(Robot::oi->getGamePad()->GetY(Joystick::kLeftHand));
	
	if(Robot::oi->getGamePad()->GetRawButton(10)){
		Robot::shooter->camController->Disable();
		WindowMotorSetTask->SetOutput(Robot::oi->getGamePad()->GetY(Joystick::kLeftHand));
		Robot::shooter->camLeft->Set(0.53*Robot::oi->getGamePad()->GetY(Joystick::kLeftHand));
		Robot::shooter->camRight->Set(0.53*Robot::oi->getGamePad()->GetY(Joystick::kLeftHand));
	}
	else {
		Robot::shooter->CamChecker(); //Runs every cycle to control cam postion
		if(Robot::oi->getGamePad()->GetRawButton(4) && !Robot::shooter->GetFiring())
			Robot::shooter->Fire();		
	}
	



/******************BEATERBAR**************************************/	
	
	if(Robot::pickup->beaterBar->IsAlive())
		Robot::pickup->beaterBar->Set(Robot::oi->getGamePad()->GetY(Joystick::kRightHand));
}
void Robot::TestPeriodic() {
	lw->Run();
}
void Robot::SMDB() {
	//Joystick Variables
	SmartDashboard::PutNumber("StickMagnitude",Robot::oi->getDriverJoystickRight()->GetMagnitude());
	SmartDashboard::PutNumber("StickDirection",Robot::oi->getDriverJoystickRight()->GetDirectionRadians());
	SmartDashboard::PutNumber("StickTwist",Robot::oi->getDriverJoystickRight()->GetTwist());
	SmartDashboard::PutNumber("GyroAngle", Robot::driveTrain->gyro->GetAngle());
	SmartDashboard::PutNumber("ScaledRadians",Robot::oi->getScaledJoystickRadians());
	SmartDashboard::PutNumber("ScalingFactor",Robot::oi->getJoystickTwist());
	//Wheel Module Voltages
	SmartDashboard::PutNumber("FrontLeftVol",Robot::driveTrain->frontLeftPos->GetAverageVoltage());
	SmartDashboard::PutNumber("FrontRightVol",Robot::driveTrain->frontRightPos->GetAverageVoltage());
	SmartDashboard::PutNumber("RearLeftVol",Robot::driveTrain->rearLeftPos->GetAverageVoltage());
	SmartDashboard::PutNumber("RearRightVol",Robot::driveTrain->rearRightPos->GetAverageVoltage());	
	//Wheel Module Errors
	SmartDashboard::PutNumber("FLError", Robot::driveTrain->frontLeft->GetError());
	SmartDashboard::PutNumber("FRError", Robot::driveTrain->frontRight->GetError());
	SmartDashboard::PutNumber("RLError", Robot::driveTrain->rearLeft->GetError());
	SmartDashboard::PutNumber("RRError", Robot::driveTrain->rearRight->GetError());
	//Wheel Module Setpoints
	SmartDashboard::PutNumber("FLSetPoint", Robot::driveTrain->frontLeft->GetSetpoint());
	SmartDashboard::PutNumber("FRSetPoint", Robot::driveTrain->frontRight->GetSetpoint());
	SmartDashboard::PutNumber("RLSetPoint", Robot::driveTrain->rearLeft->GetSetpoint());
	SmartDashboard::PutNumber("RRSetPoint", Robot::driveTrain->rearRight->GetSetpoint());
	
	//ShooterValues
	SmartDashboard::PutNumber("CorrectedCamPostion",Robot::shooter->GetCorrectedCamPos());
	SmartDashboard::PutNumber("RawCamPostion",Robot::shooter->camPos->GetAverageVoltage());
	SmartDashboard::PutBoolean("CamPIDOnTarget",Robot::shooter->camController->OnTarget());
	SmartDashboard::PutNumber("WindowMotorOutput", Robot::shooter->windowMotors->Get());
	SmartDashboard::PutNumber("CamLeftOutput", Robot::shooter->camLeft->Get());
	SmartDashboard::PutNumber("CamRightOutput", Robot::shooter->camRight->Get());
	SmartDashboard::PutBoolean("CamPIDEnabled", Robot::shooter->camController->IsEnabled());
	SmartDashboard::PutNumber("CamPIDSetPoint", Robot::shooter->camController->GetSetpoint());
	SmartDashboard::PutNumber("CamPIDOutput", Robot::shooter->camController->Get());
	SmartDashboard::PutNumber("FireTimer", Robot::shooter->fireTimer->Get());

	//Jaguar Stauses
	SmartDashboard::PutBoolean("06-FLSteerJagAlive",Robot::driveTrain->frontLeftSteer->IsAlive());
	SmartDashboard::PutBoolean("07-FRSteerJagAlive",Robot::driveTrain->frontRightSteer->IsAlive());
	SmartDashboard::PutBoolean("08-RLSteerJagAlive",Robot::driveTrain->rearLeftSteer->IsAlive());
	SmartDashboard::PutBoolean("09-RRSteerJagAlive",Robot::driveTrain->rearRightSteer->IsAlive());
	SmartDashboard::PutBoolean("10-WindowMotorJagAlive",Robot::shooter->windowMotors->IsAlive());
	SmartDashboard::PutBoolean("11-CamLeftJagAlive",Robot::shooter->camLeft->IsAlive());
	SmartDashboard::PutBoolean("12-CamRightJagAlive",Robot::shooter->camRight->IsAlive());
	SmartDashboard::PutBoolean("13-BeaterBarJagAlive",Robot::pickup->beaterBar->IsAlive());

}
START_ROBOT_CLASS(Robot);
