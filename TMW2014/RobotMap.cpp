// RobotBuilder Version: 1.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in th future.
#include "RobotMap.h"
#include "LiveWindow/LiveWindow.h"
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION
SpeedController* RobotMap::driveTrainFrontLeftDrive = NULL;
SpeedController* RobotMap::driveTrainFrontRightDrive = NULL;
SpeedController* RobotMap::driveTrainRearLeftDrive = NULL;
SpeedController* RobotMap::driveTrainRearRightDrive = NULL;
AnalogChannel* RobotMap::driveTrainFrontLeftPos = NULL;
CANJaguar* RobotMap::driveTrainFrontLeftSteer = NULL;
PIDController* RobotMap::driveTrainFrontLeft = NULL;
AnalogChannel* RobotMap::driveTrainFrontRightPos = NULL;
CANJaguar* RobotMap::driveTrainFrontRightSteer = NULL;
PIDController* RobotMap::driveTrainFrontRight = NULL;
AnalogChannel* RobotMap::driveTrainRearLeftPos = NULL;
CANJaguar* RobotMap::driveTrainRearLeftSteer = NULL;
PIDController* RobotMap::driveTrainRearLeft = NULL;
AnalogChannel* RobotMap::driveTrainRearRightPos = NULL;
CANJaguar* RobotMap::driveTrainRearRightSteer = NULL;
PIDController* RobotMap::driveTrainRearRight = NULL;
CANJaguar* RobotMap::shooterCamLeft = NULL;
CANJaguar* RobotMap::shooterCamRight = NULL;
AnalogChannel* RobotMap::shooterCamPos = NULL;
AnalogChannel* RobotMap::shooterBackupCamPos = NULL;
Solenoid* RobotMap::shooterFingers = NULL;
DigitalInput* RobotMap::shooterBallNotPresent = NULL;
CANJaguar* RobotMap::pickupBeaterBar = NULL;
Compressor* RobotMap::pickupComp = NULL;
Solenoid* RobotMap::pickupBeaterBarOut = NULL;
Solenoid* RobotMap::pickupWings = NULL;
DigitalInput* RobotMap::pickupBallInPickup = NULL;
DigitalInput* RobotMap::odroidOdroidHeartBeat = NULL;
Solenoid* RobotMap::odroidRingLights = NULL;
DigitalInput* RobotMap::odroidTargetLeft = NULL;
DigitalOutput* RobotMap::odroidSendProcessImage = NULL;
DigitalOutput* RobotMap::odroidLEDSelect1 = NULL;
DigitalOutput* RobotMap::odroidLEDSelect2 = NULL;
DigitalOutput* RobotMap::odroidLEDSelect3 = NULL;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION
//CANJaguar* RobotMap::pickupBeaterBar;
CamPIDOut* RobotMap::shooterCamOut=NULL;
BSGyro* RobotMap::driveTrainGyro=NULL;
CrabSpeed* RobotMap::CrabSpeedTwist = NULL;
PIDController* RobotMap::driveTrainDriveControlTwist = NULL;
void RobotMap::init() {
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	LiveWindow* lw = LiveWindow::GetInstance();
	driveTrainFrontLeftDrive = new Talon(1, 1);
	lw->AddActuator("DriveTrain", "FrontLeftDrive", (Talon*) driveTrainFrontLeftDrive);
	
	driveTrainFrontRightDrive = new Talon(1, 2);
	lw->AddActuator("DriveTrain", "FrontRightDrive", (Talon*) driveTrainFrontRightDrive);
	
	driveTrainRearLeftDrive = new Talon(1, 3);
	lw->AddActuator("DriveTrain", "RearLeftDrive", (Talon*) driveTrainRearLeftDrive);
	
	driveTrainRearRightDrive = new Talon(1, 4);
	lw->AddActuator("DriveTrain", "RearRightDrive", (Talon*) driveTrainRearRightDrive);
	
	driveTrainFrontLeftPos = new AnalogChannel(1, 2);
	lw->AddSensor("DriveTrain", "FrontLeftPos", driveTrainFrontLeftPos);
	
	driveTrainFrontLeftSteer = new CANJaguar(6);
	
	
	driveTrainFrontLeft = new PIDController(3.0, 0.0, 0.0,/* F: 0.0, */ driveTrainFrontLeftPos, driveTrainFrontLeftSteer, 0.02);
	lw->AddActuator("DriveTrain", "FrontLeft", driveTrainFrontLeft);
	driveTrainFrontLeft->SetContinuous(true); driveTrainFrontLeft->SetAbsoluteTolerance(0.2); 
        driveTrainFrontLeft->SetOutputRange(-0.75, 0.75);
	driveTrainFrontRightPos = new AnalogChannel(1, 3);
	lw->AddSensor("DriveTrain", "FrontRightPos", driveTrainFrontRightPos);
	
	driveTrainFrontRightSteer = new CANJaguar(7);
	
	
	driveTrainFrontRight = new PIDController(3.0, 0.0, 0.0,/* F: 0.0, */ driveTrainFrontRightPos, driveTrainFrontRightSteer, 0.02);
	lw->AddActuator("DriveTrain", "FrontRight", driveTrainFrontRight);
	driveTrainFrontRight->SetContinuous(true); driveTrainFrontRight->SetAbsoluteTolerance(0.2); 
        driveTrainFrontRight->SetOutputRange(-0.75, 0.75);
	driveTrainRearLeftPos = new AnalogChannel(1, 4);
	lw->AddSensor("DriveTrain", "RearLeftPos", driveTrainRearLeftPos);
	
	driveTrainRearLeftSteer = new CANJaguar(8);
	
	
	driveTrainRearLeft = new PIDController(3.0, 0.0, 0.0,/* F: 0.0, */ driveTrainRearLeftPos, driveTrainRearLeftSteer, 0.02);
	lw->AddActuator("DriveTrain", "RearLeft", driveTrainRearLeft);
	driveTrainRearLeft->SetContinuous(true); driveTrainRearLeft->SetAbsoluteTolerance(0.2); 
        driveTrainRearLeft->SetOutputRange(-0.75, 0.75);
	driveTrainRearRightPos = new AnalogChannel(1, 5);
	lw->AddSensor("DriveTrain", "RearRightPos", driveTrainRearRightPos);
	
	driveTrainRearRightSteer = new CANJaguar(9);
	
	
	driveTrainRearRight = new PIDController(3.0, 0.0, 0.0,/* F: 0.0, */ driveTrainRearRightPos, driveTrainRearRightSteer, 0.02);
	lw->AddActuator("DriveTrain", "RearRight", driveTrainRearRight);
	driveTrainRearRight->SetContinuous(true); driveTrainRearRight->SetAbsoluteTolerance(0.2); 
        driveTrainRearRight->SetOutputRange(-0.75, 0.75);
	shooterCamLeft = new CANJaguar(11);
	
	
	shooterCamRight = new CANJaguar(12);
	
	
	shooterCamPos = new AnalogChannel(1, 6);
	lw->AddSensor("Shooter", "CamPos", shooterCamPos);
	
	shooterBackupCamPos = new AnalogChannel(1, 7);
	lw->AddSensor("Shooter", "BackupCamPos", shooterBackupCamPos);
	
	shooterFingers = new Solenoid(2, 3);
	lw->AddActuator("Shooter", "Fingers", shooterFingers);
	
	shooterBallNotPresent = new DigitalInput(1, 8);
	lw->AddSensor("Shooter", "BallNotPresent", shooterBallNotPresent);
	
	pickupBeaterBar = new CANJaguar(13);
	
	
	pickupComp = new Compressor(1, 4, 1, 1);
	
	
	pickupBeaterBarOut = new Solenoid(2, 1);
	lw->AddActuator("Pickup", "BeaterBarOut", pickupBeaterBarOut);
	
	pickupWings = new Solenoid(2, 2);
	lw->AddActuator("Pickup", "Wings", pickupWings);
	
	pickupBallInPickup = new DigitalInput(1, 9);
	lw->AddSensor("Pickup", "BallInPickup", pickupBallInPickup);
	
	odroidOdroidHeartBeat = new DigitalInput(1, 3);
	lw->AddSensor("Odroid", "OdroidHeartBeat", odroidOdroidHeartBeat);
	
	odroidRingLights = new Solenoid(2, 5);
	lw->AddActuator("Odroid", "RingLights", odroidRingLights);
	
	odroidTargetLeft = new DigitalInput(1, 2);
	lw->AddSensor("Odroid", "TargetLeft", odroidTargetLeft);
	
	odroidSendProcessImage = new DigitalOutput(1, 1);
	
	
	odroidLEDSelect1 = new DigitalOutput(1, 5);
	
	
	odroidLEDSelect2 = new DigitalOutput(1, 6);
	
	
	odroidLEDSelect3 = new DigitalOutput(1, 7);
	
	
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        
	driveTrainFrontLeftPos->SetAverageBits(256);
	driveTrainFrontRightPos->SetAverageBits(256);
	driveTrainRearLeftPos->SetAverageBits(256);
	driveTrainRearRightPos->SetAverageBits(256);
	shooterCamPos->SetAverageBits(256);
		
	
    driveTrainFrontLeftPos->SetVoltageForPID(true);
    driveTrainFrontRightPos->SetVoltageForPID(true);
    driveTrainRearLeftPos->SetVoltageForPID(true);
    driveTrainRearRightPos->SetVoltageForPID(true);
        
//	driveTrainGyro = new BSGyro(1, 1, 487740, 0);
    driveTrainGyro = new BSGyro(1, 1);
    lw->AddSensor("DriveTrain", "Gyro", driveTrainGyro);
	driveTrainGyro->SetSensitivity(0.007);
	
	CrabSpeedTwist = new CrabSpeed();
	
	driveTrainDriveControlTwist = new PIDController(.035, 0, .1, driveTrainGyro, CrabSpeedTwist, 0.02);
	driveTrainDriveControlTwist->SetContinuous(true);
	driveTrainDriveControlTwist->SetInputRange(-360.0,360.0);
	driveTrainDriveControlTwist->SetAbsoluteTolerance(2.0);
	
}
