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
Odroid* Robot::odroid = 0;
OI* Robot::oi = 0;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INITIALIZATION
BSTimer* autoStepTimer = NULL;
void Robot::RobotInit() {
	RobotMap::init();
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	driveTrain = new DriveTrain();
	shooter = new Shooter();
	pickup = new Pickup();
	odroid = new Odroid();
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	// This MUST be here. If the OI creates Commands (which it very likely
	// will), constructing it during the construction of CommandBase (from
	// which commands extend), subsystems are not guaranteed to be
	// yet. Thus, their requires() statements may grab null pointers. Bad
	// news. Don't move it.
	oi = new OI();
	lw = LiveWindow::GetInstance();
	// instantiate the command used for the autonomous period
	prevTrigger = false;
	
	kinect = Kinect::GetInstance();
	
	File = RAWCConstants::getInstance();
	driveTrain->SetWheelbase(20.625/2, 20.625, 20.625/2);
	FLOffset = File->getValueForKey("FLOff");
	FROffset = File->getValueForKey("FROff");
	RLOffset = File->getValueForKey("RLOff");
	RROffset = File->getValueForKey("RROff");
	driveTrain->SetOffsets(FLOffset, FROffset, RLOffset, RROffset);
	
	driveTrain->SetFLTurns((int)File->getValueForKey("FLTurns"));
	driveTrain->SetFRTurns((int)File->getValueForKey("FRTurns"));
	driveTrain->SetRLTurns((int)File->getValueForKey("RLTurns"));
	driveTrain->SetRRTurns((int)File->getValueForKey("RRTurns"));
	
	driveTrain->frontLeft->Enable();
	driveTrain->frontRight->Enable();
	driveTrain->rearLeft->Enable();
	driveTrain->rearRight->Enable();
	
	primaryCamOffset = File->getValueForKey("PCOffset");
	backupCamOffset = File->getValueForKey("BCOffset");
	shooter->SetCamOffsets(primaryCamOffset, backupCamOffset);
		
	pickup->comp->Start();
	
	autoStep = DropPickup;
	autoProgram = fire2DriveForward;
	autoStepComplete = false;
	autoStepIncrementer = 0;
	turnDirection = 0;
	
	autoChooser = new SendableChooser();
	autoChooser->AddObject("01. Shoot 1 From Side", (void*)fire1Side);
	autoChooser->AddObject("02. Shoot 1 From Center", (void*)fire1Center);
	autoChooser->AddDefault("03. Shoot 2 Narrow Center", (void*)fire2DriveForward);
	autoChooser->AddObject("04. Shoot 2 Wide Center", (void*)fire2DriveForwardWide);
	autoChooser->AddObject("05. Shoot 2 From Side", (void*)fire2Side);
	autoChooser->AddObject("06. Shoot 3 From Center", (void*)fire3FromCenter);
	SmartDashboard::PutData("Autonomous Chooser", autoChooser);
	autoStepTimer = new BSTimer();
	autoStepTimer->Start();
	onTargetTimer = new BSTimer();
	onTargetTimer->Start();
	autonomousTimer = new BSTimer();
	autonomousTimer->Start();
	beaterBarTimer = new BSTimer();
	beaterBarTimer->Start();
	driveForwardAngle = 0;
	
	/*LEDChooser = new SendableChooser();
	LEDChooser->AddDefault("1. Solid Blue", (void*)Led0);
	LEDChooser->AddObject("2. Solid Red", (void*)Led1);
	LEDChooser->AddObject("3. Blue Chase", (void*)Led2);
	LEDChooser->AddObject("4. Red Chase", (void*)Led3);
	LEDChooser->AddObject("5. Rainbow Cycle", (void*)Led4);
	LEDChooser->AddObject("6. Seven Color Comet", (void*)Led5);
	LEDChooser->AddObject("7. Patriotic Sweep", (void*)Led6);
	LEDChooser->AddObject("8. Off", (void*)Led7);
	SmartDashboard::PutData("LED Chooser", LEDChooser);*/
	x = 0;
	y = 0;
	twist = 0;
}

void Robot::DisabledInit() {
	File->insertKeyAndValue("FLTurns", driveTrain->GetFLTurns());
	File->insertKeyAndValue("FRTurns", driveTrain->GetFRTurns());
	File->insertKeyAndValue("RLTurns", driveTrain->GetRLTurns());
	File->insertKeyAndValue("RRTurns", driveTrain->GetRRTurns());
	
	File->save();
}

void Robot::DisabledPeriodic() {
	if(oi->getDriverJoystickRight()->GetRawButton(7))
		SMDB();
	driveTrain->CheckForTurns();
	
	//SmartDashboard::PutNumber("AutoChooser", (int)autoProgram);
	
	pickup->beaterBarOut->Set(false);
	pickup->wings->Set(false);
	
	Scheduler::GetInstance()->Run();
	
	LEDSet(4);
	//KinectInfo
	SmartDashboard::PutBoolean("TooManyPlayers!!!!!", kinect->GetNumberOfPlayers() <= 1);
	SmartDashboard::PutBoolean("NoPlayers!!!!!!!", kinect->GetNumberOfPlayers() > 0);
	SmartDashboard::PutBoolean("LeftHand", KinectLeftSelect());
	SmartDashboard::PutBoolean("RightHand", KinectRightSelect());
	SmartDashboard::PutBoolean("RightSelect", turnDirection == 1);
	SmartDashboard::PutBoolean("LeftSelect", turnDirection == -1);
	//Autonomous Selection Feedback
	switch (static_cast<AutoProgram>((int)(autoChooser->GetSelected())))
	{
	case fire1Side:
		SmartDashboard::PutNumber("AutonomousSelection", 1);
		break;
	case fire1Center:
		SmartDashboard::PutNumber("AutonomousSelection", 2);
		break;
	case fire2DriveForward:
		SmartDashboard::PutNumber("AutonomousSelection", 3);
		break;
	case fire2DriveForwardWide:
		SmartDashboard::PutNumber("AutonomousSelection", 4);
		break;
	case fire2Side:
		SmartDashboard::PutNumber("AutonomousSelection", 5);
		break;
	case fire3FromCenter:
		SmartDashboard::PutNumber("AutonomousSelection", 6);
		break;
	}	
}
	
void Robot::AutonomousInit() {
	LEDSet(7);
    autoProgram = static_cast<AutoProgram>((int)(autoChooser->GetSelected()));
	autoStepTimer->Reset();
	autoStepIncrementer = 0;
	turnDirection = 0;
	peakBeaterBarCurrent = 0;
	ballCollectedInc = 0;
	genericAutoProgram.clear();
	autonomousTimer->Reset();
	driveTrain->gyro->Reset();
	x = 0;
	y = 0;
	twist = 0;
	switch(autoProgram) {
	
	case fire1Side:
		genericAutoProgram.push_back(DriveForwardReset);
		genericAutoProgram.push_back(WaitForHot);
		genericAutoProgram.push_back(Fire);	
		genericAutoProgram.push_back(End);				
		break;
		
	case fire1Center:
		genericAutoProgram.push_back(DriveForwardFirstTurn);
		genericAutoProgram.push_back(Fire);
		genericAutoProgram.push_back(End);				
		break;
		
	case fire2Side:
		genericAutoProgram.push_back(DropPickup);
		genericAutoProgram.push_back(DriveForwardReset);
		genericAutoProgram.push_back(WaitForHot);
		genericAutoProgram.push_back(Fire);	
		genericAutoProgram.push_back(ResetShooterAndCollect);
		genericAutoProgram.push_back(LoadBall);
		genericAutoProgram.push_back(Fire);
		genericAutoProgram.push_back(End);		
		break;
		
	case fire2DriveForward: case fire2DriveForwardWide:
		genericAutoProgram.push_back(DropPickup);
		genericAutoProgram.push_back(DriveForwardFirstTurn);
		genericAutoProgram.push_back(Fire);
		genericAutoProgram.push_back(ResetShooterAndCollect);
		genericAutoProgram.push_back(LoadBall);
		genericAutoProgram.push_back(SecondTurn);
		genericAutoProgram.push_back(DropPickup);
		genericAutoProgram.push_back(WaitToFire75);
		genericAutoProgram.push_back(Fire);
		genericAutoProgram.push_back(End);		
		break;
		
	case fire3FromCenter:
		genericAutoProgram.push_back(FirstResetShooter);
		genericAutoProgram.push_back(FirstTurn);
		genericAutoProgram.push_back(DropPickup);
		genericAutoProgram.push_back(Fire);
		genericAutoProgram.push_back(ResetShooterAndCollect);
		genericAutoProgram.push_back(LoadBall);
		genericAutoProgram.push_back(SecondTurn);
		genericAutoProgram.push_back(DropPickup);
		genericAutoProgram.push_back(WaitToFire50);
		genericAutoProgram.push_back(Fire);		
		genericAutoProgram.push_back(ResetShooterAndCollect);
		genericAutoProgram.push_back(LoadBall);
		genericAutoProgram.push_back(DriveForward);
		genericAutoProgram.push_back(Chill);
		genericAutoProgram.push_back(DropPickup);
		genericAutoProgram.push_back(Fire);
		genericAutoProgram.push_back(End);
		break;
		
	}
	autoStep = genericAutoProgram[autoStepIncrementer];
	
	driveTrain->DriveControlTwist->Enable();
	driveForwardAngle = 0;
//	LEDSet(7);
	//odroid->sendProcessImage->Set(true);
}
	
void Robot::AutonomousPeriodic() {
	//KinectInfo
	SmartDashboard::PutBoolean("TooManyPlayers!!!!!", kinect->GetNumberOfPlayers() <= 1);
	SmartDashboard::PutBoolean("NoPlayers!!!!!!!", kinect->GetNumberOfPlayers() > 0);
	SmartDashboard::PutBoolean("LeftHand", KinectLeftSelect());
	SmartDashboard::PutBoolean("RightHand", KinectRightSelect());
	SmartDashboard::PutBoolean("RightSelect", turnDirection == 1);
	SmartDashboard::PutBoolean("LeftSelect", turnDirection == -1);
	
	driveTrain->CheckForTurns();
	
	if(oi->getDriverJoystickRight()->GetRawButton(7))
		SMDB();
//	if (autonomousTimer->HasPeriodPassed(1))
//		LEDSet(4);
	LEDSet(7);
	if(autoStep != FirstResetShooter && autoStep != FirstTurn && autoStep !=DriveForwardFirstTurn && autoStep != DriveForwardReset) {
		shooter->CamChecker();
	}
	
	int turnDegree = 13;
	
	if (autoProgram == fire3FromCenter || autoProgram == fire2DriveForwardWide) {
		turnDegree = 20;
	}
	
	switch(autoStep) {
	
	case ResetShooterAndCollect:
		x = 0;
		y = 0;
		if(autoStepTimer->HasPeriodPassed(.2)){
			if(pickup->beaterBar->GetOutputCurrent() >= peakBeaterBarCurrent) {
				peakBeaterBarCurrent = pickup->beaterBar->GetOutputCurrent();
				ballCollectedInc = 0;
			}
			else
				ballCollectedInc++;
		}
		pickup->beaterBar->Set(1);
		SmartDashboard::PutString("AutoStep", "ResetShooter");
		if(shooter->GetResetCamComplete() && (ballCollectedInc > 10 || autoStepTimer->HasPeriodPassed(1.5))) {
			autoStepComplete = true;
			pickup->beaterBar->Set(0);
			ballCollectedInc = 0;
		}
		break;
		
	case FirstResetShooter:
		x = 0;
		y = 0;
		
		if(!shooter->GetResetCamComplete()) {
			shooter->Reset();
			pickup->beaterBar->Set(-.25);
		}
		else {
			shooter->CamChecker();
		}
		if(turnDirection == 0) {
			if(KinectLeftSelect() || autoStepTimer->HasPeriodPassed(2.0)) {
				turnDirection = -1;
			}
			else if (KinectRightSelect()) {
				turnDirection = 1;
			}
		}
		
		SmartDashboard::PutString("AutoStep", "FirstResetShooter");
		if(turnDirection != 0 && autoStepTimer->HasPeriodPassed(0.25)) {
			autoStepComplete = true;
			pickup->beaterBar->Set(0);
		}
		break;
		
	case WaitToFire50:
		x = 0;
		y = 0;
		SmartDashboard::PutString("AutoStep", "FindTarget");
		if(autonomousTimer->HasPeriodPassed(5.0))
			autoStepComplete = true;
		break;
		
	case WaitToFire75:
		x = 0;
		y = 0;
		SmartDashboard::PutString("AutoStep", "FindTarget");
		if(autonomousTimer->HasPeriodPassed(7.5))
			autoStepComplete = true;
		break;
		
	case WaitForHot:
		if(KinectLeftSelect() || KinectRightSelect())
			autoStepComplete = true;
		SmartDashboard::PutString("AutoStep", "WaitForHot");
		break;
		
	case FirstTurn:
		twist = turnDegree*turnDirection;
		x = 0;
		y = 0;
		if(!shooter->GetResetCamComplete()) {
			shooter->Reset();
			if(autoProgram == fire3FromCenter){
				pickup->beaterBar->Set(-.25);
			}
			else if(!autoStepTimer->HasPeriodPassed(0.25)){
				pickup->beaterBar->Set(.25);
			}
			else
				pickup->beaterBar->Set(0);
			
		}
		else {
			shooter->CamChecker();
		}
		driveTrain->DriveControlTwist->SetPID(.035, 0, .15);
		SmartDashboard::PutString("AutoStep", "FirstTurn");
		if(!driveTrain->DriveControlTwist->OnTarget()) {
			onTargetTimer->Reset();
		}
		if(onTargetTimer->HasPeriodPassed(.2) && shooter->GetResetCamComplete()){
			autoStepComplete = true;
			pickup->beaterBar->Set(0);
		}
		break;
		
	case DriveForwardReset:
		driveTrain->DriveControlTwist->SetPID(.010, 0, .05);		
		x = 0;
		y = -.5;
		twist = 0;
		
		if(!shooter->GetResetCamComplete()) {
			shooter->Reset();
			pickup->beaterBar->Set(-.25);
			}
		else {
			shooter->CamChecker();
		}
		SmartDashboard::PutString("AutoStep", "DriveForwardReset");
		if(autoStepTimer->HasPeriodPassed(1.3) && shooter->GetResetCamComplete()) {
			autoStepComplete = true;
			x = 0;
			y = 0;
			twist = 0;
		}
		break;
		
	case DriveForwardFirstTurn:
		
		if(pickup->ballInPickup->Get() == 1) {
			pickup->beaterBar->Set(.75);
		}
		else {
			pickup->beaterBar->Set(0);
		}
		
		if(turnDirection == 0) {
			if(KinectLeftSelect() || autoStepTimer->HasPeriodPassed(2.5)) {
				turnDirection = -1;
			}
			else if (KinectRightSelect()) {
				turnDirection = 1;
			}
		}
		if(!autoStepTimer->HasPeriodPassed(1.45)) {
			x = 0;
			y = -.5;
			twist = 0;
			driveTrain->DriveControlTwist->SetPID(.010, 0, .05);
		}
		else {
			x = 0;
			y = 0;
			twist = turnDegree*turnDirection;
			driveTrain->DriveControlTwist->SetPID(.040, 0, .15);
		}
		
		if(!shooter->GetResetCamComplete()) {
			shooter->Reset();
			}
		else {
			shooter->CamChecker();
		}
		
		SmartDashboard::PutString("AutoStep", "DriveForwardFirstTurn");
		if(!driveTrain->DriveControlTwist->OnTarget() || turnDirection == 0 || twist == 0) {
			onTargetTimer->Reset();
		}
		if(turnDirection != 0 && onTargetTimer->HasPeriodPassed(1.0) && shooter->GetResetCamComplete()){
			autoStepComplete = true;
			pickup->beaterBar->Set(0);
			x = 0;
			y = 0;
		}
		break;
	
	case SecondTurn:
		twist = turnDegree*-turnDirection;
		x = 0;
		y = 0;
		driveTrain->DriveControlTwist->SetPID(.035, 0, .15);
		pickup->beaterBar->Set(-.25);
		SmartDashboard::PutString("AutoStep", "SecondTurn");
		if(!driveTrain->DriveControlTwist->OnTarget()) {
			onTargetTimer->Reset();
		}
		if(onTargetTimer->HasPeriodPassed(.2) && shooter->GetResetCamComplete()){
			autoStepComplete = true;
			pickup->beaterBar->Set(0);
		}
		break;
		
	case Fire:
		x = 0;
		y = 0;
		shooter->Fire(0, false);
		SmartDashboard::PutString("AutoStep", "Fire");
		if(shooter ->GetCorrectedCamPos() < 4.5 && shooter->GetCorrectedCamPos() > 4.0)
			autoStepComplete = true;
		break;
			
	case Chill:
		x = 0;
		y = 0;
		SmartDashboard::PutString("AutoStep", "Chill");
		if(autoStepTimer->HasPeriodPassed(.5))
			autoStepComplete = true;
		break;
	
	case CollectBall:
		pickup->beaterBar->Set(1);
		x = 0;
		y = 0;
		if(autoStepTimer->HasPeriodPassed(.2) && pickup->beaterBar->GetOutputCurrent() >= peakBeaterBarCurrent) {
			peakBeaterBarCurrent = pickup->beaterBar->GetOutputCurrent();
			ballCollectedInc = 0;
		}
		else
			ballCollectedInc++;
		SmartDashboard::PutString("AutoStep", "CollectBall");
		if(autoStepTimer->HasPeriodPassed(1.5) || ballCollectedInc >= 50) {
			autoStepComplete = true;
			pickup->beaterBar->Set(0);
			peakBeaterBarCurrent = 0;
			ballCollectedInc = 0;
		}
		break;
		
		
	case LoadBall:
		pickup->beaterBarOut->Set(false);
		pickup->beaterBar->Set(-.5);
		x = 0;
		y = 0;
		SmartDashboard::PutString("AutoStep", "LoadBall");
		if(autoStepTimer->HasPeriodPassed(.2)){
			autoStepComplete = true;
		}
		break;
		
	case DropPickup:
		pickup->beaterBarOut->Set(true);
		if(autoStepTimer->HasPeriodPassed(.2)) {
			pickup->beaterBar->Set(.5);
		}
		x = 0;
		y = 0;
		SmartDashboard::PutString("AutoStep", "DropPickup");
		if(autoStepTimer->HasPeriodPassed(0.6)) {
			autoStepComplete = true;
			pickup->beaterBar->Set(0.0);
		}
		break;
	
	case DriveForward:
		x = 0;
		y = -1.0;
		pickup->beaterBar->Set(-.5);
		SmartDashboard::PutString("AutoStep", "DriveForward");
		if(autoStepTimer->HasPeriodPassed(.75)) {
			autoStepComplete = true;
			pickup->beaterBar->Set(0);
		}
		break;
		
	case End:
		x = 0;
		y = 0;
		SmartDashboard::PutString("AutoStep", "End");
		break;
	}
	
	driveTrain->DriveControlTwist->SetSetpoint(twist);
	driveTrain->Crab(driveTrain->CrabSpeedTwist->Get(), y, x, true);
	
/***********************Increment Through Program****************************/	
	if (autoStepComplete) {
		autoStepTimer->Reset();
		autoStepComplete = false;
		autoStepIncrementer ++;
		onTargetTimer->Reset();
		try {
			autoStep = genericAutoProgram.at(autoStepIncrementer);
		}
		catch (const out_of_range& oor) {
			printf ("AutoProgram Vector Out of Range \n");
		}
	}
}
	
void Robot::TeleopInit() {
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to 
	// continue until interrupted by another command, remove
	// this line or comment it out.
	driveTrain->DriveControlTwist->Disable();
}
	
void Robot::TeleopPeriodic() {
	LEDSet(5);
	
	driveTrain->CheckForTurns();
	
	if(oi->getDriverJoystickRight()->GetRawButton(7))
		SMDB();
		
/******************DRIVETRAIN**************************************/	
	
	//Resets gyro to zero when crab starts
	if (!prevTrigger && oi->getDriverJoystickRight()->GetRawButton(1))
		driveTrain->gyro->Reset();
	
	prevTrigger = oi->getDriverJoystickRight()->GetRawButton(1);
	
	if(oi->getDriverJoystickLeft()->GetRawButton(1))
	{
		driveTrain->UndoTurns();
		undoTurnsPressed = true;
	}
	else if(oi->getDriverJoystickRight()->GetRawButton(1))
	{
		driveTrain->Crab(oi->getJoystickTwist(),-oi->getJoystickY(),oi->getJoystickX(),true);	
	}
	else 
	{
		driveTrain->Steer(oi->getLeftJoystickXRadians(),oi->getJoystickY(),0.5);
	}
	
	if (undoTurnsPressed && !oi->getDriverJoystickLeft()->GetRawButton(1))
	{
		driveTrain->frontLeft->Enable();
		driveTrain->frontRight->Enable();
		driveTrain->rearLeft->Enable();
		driveTrain->rearRight->Enable();
		undoTurnsPressed = false;
	}
	
	
/******************SHOOTER**************************************/
	
	if(oi->getGamePad()->GetRawButton(10)){
		if(oi->getGamePad()->GetRawAxis(6) < 0) {
			shooter->Reset();
		}
		else if(oi->getGamePad()->GetRawAxis(6) > 0) {
			shooter->RelieveStress();
		}
		else {
			shooter->RunCams(-oi->getGamePad()->GetRawAxis(2), true);
		}
	}
	else if(oi->getDriverJoystickRight()->GetRawButton(9) || oi->getGamePad()->GetRawButton(9)){
		shooter->RelieveStress();
	}
	else if(oi->getDriverJoystickRight()->GetRawButton(11)){
		shooter->Reset();
	}
	else {
		shooter->CamChecker(); //Runs every cycle to control cam postion
		if((oi->getGamePad()->GetRawButton(4) || oi->getDriverJoystickRight()->GetRawButton(3)) && !shooter->GetFiring()) {
			if(!pickup->beaterBarOut->Get())
				beaterBarTimer->Reset();
			pickup->beaterBarOut->Set(true);
			shooter->Fire(0.6 - beaterBarTimer->Get(), true);
		}
		else if (oi->getDriverJoystickRight()->GetRawButton(2)) {
			shooter->Fire(0,false);
		}
	}
	
	
	shooter->fingers->Set(oi->getGamePad()->GetRawButton(2));
	
/******************WINGS**************************************/
	
	if(oi->getGamePad()->GetRawButton(7))
		pickup->wings->Set(true);
	if(oi->getGamePad()->GetRawButton(5))
		pickup->wings->Set(false);
	
	
/******************BEATERBAR**************************************/	
	
	if(fabs(oi->getGamePad()->GetRawAxis(4)) > fabs(oi->getGamePad()->GetRawAxis(2))) {
		pickup->beaterBar->Set((oi->getGamePad()->GetRawAxis(4)));
	}
	else
		pickup->beaterBar->Set(oi->getGamePad()->GetRawAxis(2));
	
	if(oi->getDriverJoystickLeft()->GetRawButton(4))
		pickup->beaterBar->Set(1);
	
	if(oi->getDriverJoystickLeft()->GetRawButton(5))
		pickup->beaterBar->Set(-1);
	
	if ((oi->getGamePad()->GetRawButton(8) || oi->getDriverJoystickLeft()->GetRawButton(2)) && !pickup->beaterBarOut->Get()) {
		pickup->beaterBarOut->Set(true);
		beaterBarTimer->Reset();
	}
	if (oi->getGamePad()->GetRawButton(6) || oi->getDriverJoystickLeft()->GetRawButton(3)) {
		pickup->beaterBarOut->Set(false);
	}
	
	Scheduler::GetInstance()->Run();
}
void Robot::TestPeriodic() {
	lw->Run();
}
void Robot::SMDB() {
	//Joystick Variables
	SmartDashboard::PutNumber("RightStickY",oi->getDriverJoystickRight()->GetY());
	SmartDashboard::PutNumber("RightStickX",oi->getDriverJoystickRight()->GetX());
	SmartDashboard::PutNumber("LeftStickX", oi->getDriverJoystickLeft()->GetX());
	SmartDashboard::PutNumber("ScaledRadians",oi->getLeftJoystickXRadians());
	SmartDashboard::PutNumber("ScalingFactor",oi->getDriverJoystickRight()->GetTwist()/2+1.5);
	//Gyro Variables
	SmartDashboard::PutNumber("GyroAngle", driveTrain->gyro->GetAngle());
	SmartDashboard::PutNumber("GyroCenter", driveTrain->gyro->GetCenter());
	SmartDashboard::PutNumber("GyroOffset", driveTrain->gyro->GetOffset());
	
	//Wheel Module Voltages
	SmartDashboard::PutNumber("FrontLeftVol",driveTrain->frontLeftPos->GetAverageVoltage());
	SmartDashboard::PutNumber("FrontRightVol",driveTrain->frontRightPos->GetAverageVoltage());
	SmartDashboard::PutNumber("RearLeftVol",driveTrain->rearLeftPos->GetAverageVoltage());
	SmartDashboard::PutNumber("RearRightVol",driveTrain->rearRightPos->GetAverageVoltage());	
	//Wheel Module Errors
	SmartDashboard::PutNumber("FLError", driveTrain->frontLeft->GetError());
	SmartDashboard::PutNumber("FRError", driveTrain->frontRight->GetError());
	SmartDashboard::PutNumber("RLError", driveTrain->rearLeft->GetError());
	SmartDashboard::PutNumber("RRError", driveTrain->rearRight->GetError());
	//Wheel Module Setpoints
	SmartDashboard::PutNumber("FLSetPoint", driveTrain->frontLeft->GetSetpoint());
	SmartDashboard::PutNumber("FRSetPoint", driveTrain->frontRight->GetSetpoint());
	SmartDashboard::PutNumber("RLSetPoint", driveTrain->rearLeft->GetSetpoint());
	SmartDashboard::PutNumber("RRSetPoint", driveTrain->rearRight->GetSetpoint());
	//Twist Control
	SmartDashboard::PutNumber("PIDTwistOutput", driveTrain->DriveControlTwist->Get());
	SmartDashboard::PutNumber("PIDTwistError", driveTrain->DriveControlTwist->GetError());
	SmartDashboard::PutNumber("PIDTwistSetpoint", driveTrain->DriveControlTwist->GetSetpoint());
	//ShooterValues
	SmartDashboard::PutNumber("CorrectedCamPostion",shooter->GetCorrectedCamPos());
	SmartDashboard::PutNumber("RawPrimaryCamPostion",shooter->camPos->GetAverageVoltage());
	SmartDashboard::PutNumber("RawBackupCamPosition", shooter->backupCamPos->GetAverageVoltage());
	SmartDashboard::PutBoolean("CamPositionSensorStatus", shooter->GetCamPosStatus());
	SmartDashboard::PutBoolean("BackupCamPositionSensorStatus", shooter->GetBackupCamPosStatus());
	SmartDashboard::PutNumber("CamLeftOutput", shooter->camLeft->Get());
	SmartDashboard::PutNumber("CamRightOutput", shooter->camRight->Get());
	SmartDashboard::PutNumber("CamLeftCurrent", shooter->camLeft->GetOutputCurrent());
	SmartDashboard::PutNumber("CamRightCurrent", shooter->camRight->GetOutputCurrent());
	SmartDashboard::PutBoolean("BallNotPresent", shooter->ballNotPresent->Get());
	SmartDashboard::PutBoolean("CamResetComplete", shooter->GetResetCamComplete());
	
	//BeaterBar
	SmartDashboard::PutNumber("BeaterBarCurrent", pickup->beaterBar->GetOutputCurrent());
	SmartDashboard::PutBoolean("PickupBallSensor", pickup->ballInPickup->Get());
	
	//Jaguar Stauses
	SmartDashboard::PutBoolean("06-FLSteerJagAlive",driveTrain->frontLeftSteer->IsAlive());
	SmartDashboard::PutBoolean("07-FRSteerJagAlive",driveTrain->frontRightSteer->IsAlive());
	SmartDashboard::PutBoolean("08-RLSteerJagAlive",driveTrain->rearLeftSteer->IsAlive());
	SmartDashboard::PutBoolean("09-RRSteerJagAlive",driveTrain->rearRightSteer->IsAlive());
	SmartDashboard::PutBoolean("11-CamLeftJagAlive",shooter->camLeft->IsAlive());
	SmartDashboard::PutBoolean("12-CamRightJagAlive",shooter->camRight->IsAlive());
	SmartDashboard::PutBoolean("13-BeaterBarJagAlive",pickup->beaterBar->IsAlive());
			
	//AutoInfo
	//SmartDashboard::PutNumber("AutonomousSelection", (int)(autoChooser->GetSelected())+1);
}
void Robot::LEDSet(int led) {
	if (led == 0) {
		odroid->lEDSelect1->Set(false);
		odroid->lEDSelect2->Set(false);
		odroid->lEDSelect3->Set(false);
	}
	if (led == 1) {
		odroid->lEDSelect1->Set(true);
		odroid->lEDSelect2->Set(false);
		odroid->lEDSelect3->Set(false);
	}
	if (led == 2) {
		odroid->lEDSelect1->Set(false);
		odroid->lEDSelect2->Set(true);
		odroid->lEDSelect3->Set(false);
	}
	if (led == 3) {
		odroid->lEDSelect1->Set(true);
		odroid->lEDSelect2->Set(true);
		odroid->lEDSelect3->Set(false);
	}
	if (led == 4) {
		odroid->lEDSelect1->Set(false);
		odroid->lEDSelect2->Set(false);
		odroid->lEDSelect3->Set(true);
	}
	if (led == 5) {
		odroid->lEDSelect1->Set(true);
		odroid->lEDSelect2->Set(false);
		odroid->lEDSelect3->Set(true);
	}
	if (led == 6) {
		odroid->lEDSelect1->Set(false);
		odroid->lEDSelect2->Set(true);
		odroid->lEDSelect3->Set(true);
	}
	if (led == 7) {
		odroid->lEDSelect1->Set(true);
		odroid->lEDSelect2->Set(true);
		odroid->lEDSelect3->Set(true);
	}
	
}
bool Robot::KinectRightSelect() {
	return kinect->GetSkeleton().GetWristRight().y > kinect->GetSkeleton().GetShoulderRight().y;
}
bool Robot::KinectLeftSelect() {
	return kinect->GetSkeleton().GetWristLeft().y > kinect->GetSkeleton().GetShoulderLeft().y;
}
START_ROBOT_CLASS(Robot);
