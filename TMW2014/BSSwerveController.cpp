#include "BSSwerveController.h"

BSSwerveController::BSSwerveController(PIDController* frontLeft, PIDController* frontRight, PIDController* rearLeft, PIDController* rearRight, 
		SpeedController* frontLeftDrive, SpeedController* frontRightDrive, SpeedController* rearLeftDrive, SpeedController* rearRightDrive, Gyro* gyro,
		Joystick* driverJoystickL, Joystick* driverJoystickR) {
	m_frontLeft = frontLeft;
	m_frontLeft->Enable();
	m_frontRight = frontRight;
	m_frontRight->Enable();
	m_rearLeft = rearLeft;
	m_rearLeft->Enable();
	m_rearRight = rearRight;
	m_rearRight->Enable();
	m_frontLeftDrive = frontLeftDrive;
	m_frontRightDrive = frontRightDrive;
	m_rearLeftDrive = rearLeftDrive;
	m_rearRightDrive = rearRightDrive;
	m_gyro = gyro;
	m_driverJoystickL = driverJoystickL;
	m_driverJoystickR = driverJoystickR;
	m_driveMode = kSteer;
	m_enabled = true;
	m_driveFront = true;
	m_frontTrack = 20.625;
	m_rearTrack = 20.625;
	m_wheelBase = 20.625;
	m_FLInv = 1;
	m_FRInv = 1;
	m_RLInv = 1;
	m_RRInv = 1;
	m_steerSpeed = driverJoystickR;
	m_steerAngle = driverJoystickL;
	m_steerSpeedAxis = 2;
	m_steerAngleAxis = 1;
	RetrieveOffsetsFromFile();
	m_task = new Task("BSSwerveControllerTask", (FUNCPTR)BSSwerveController::PrivateStarter);
	if (!m_task->Start((INT32)this));
}

void BSSwerveController::PrivateStarter(BSSwerveController *task) {
	while (1) {
		if(task->IsEnabled())
		{
			task->Run();
			Wait(0.02);
		}
		else
		{
			Wait(1.0);
		}
	}
}

void BSSwerveController::Run() {
	switch(m_driveMode){
		case kSteer:
			//Steer();
			break;
		case kCrab:
			//Crab();
			break;
		case kLock:
			Lock();
			break;
	}
}

void BSSwerveController::Enable() {
	m_enabled = true;
	m_frontLeft->Enable();
	m_frontRight->Enable();
	m_rearLeft->Enable();
	m_rearRight->Enable();
}

void BSSwerveController::Disable() {
	m_enabled = false;
	m_frontLeft->Disable();
	m_frontRight->Disable();
	m_rearLeft->Disable();
	m_rearRight->Disable();
	m_frontLeftDrive->Set(0);
	m_frontRightDrive->Set(0);
	m_rearLeftDrive->Set(0);
	m_rearRightDrive->Set(0);

}

bool BSSwerveController::IsEnabled() {
	return m_enabled;
}

void BSSwerveController::SetDriveMode(DriveMode driveMode) {
	m_driveMode = driveMode;
}

void BSSwerveController::SetOffsets() {
	m_Offset["FL"] = m_frontLeft->GetSetpoint()-m_frontLeft->GetError() - 2.5;
	m_Offset["FR"]= m_frontRight->GetSetpoint()-m_frontRight->GetError() - 2.5;
	m_Offset["RL"]= m_rearLeft->GetSetpoint()-m_rearLeft->GetError() - 2.5;
	m_Offset["RR"]= m_rearRight->GetSetpoint()-m_rearRight->GetError() - 2.5;
	
	map<string, float>::iterator it; //Iterator for map
	
	ofstream outfile;
	outfile.open ("OffsetFile");
	
	//if it fails, return
	// print content:
	for ( it = m_Offset.begin() ; it != m_Offset.end(); it++ ){
		printf("Writing: %s - %f\r\n", (*it).first.c_str(),(*it).second);
		outfile << (*it).first << ", " << (*it).second << "\r\n";
	}
	
	outfile.close();
}

void BSSwerveController::RetrieveOffsetsFromFile() {
	string key;
	string value;
	
	ifstream infile ("OffsetFile");
	if (infile.is_open())
	{	
		while (!infile.eof() )
		{
			getline (infile, key, ',');
			getline(infile, value);
			stringstream convertor(value);
			convertor >> m_Offset[key];
			}
	    }
		infile.close();
}

void BSSwerveController::Steer() {
	float turningRadius = 0;	//Of the robot
	float angleRC = pi/2;	//Angle of the RearCenter Wheel (relic from 3 wheel swerve days), used in other steering calcs
	float angleFL = pi/2;	//Angle from Front Left Wheel to the center of rotation
	float angleFR = pi/2;	//Angle from Front Right Wheel to the center of rotation
	float angleRL = pi/2;	//Angle from Rear Left Wheel to the center of rotation
	float angleRR = pi/2;	//Angle from Rear Right Wheel to the center of rotation
 	float FL = 1;	//FL, distance from Front Left Wheel to the center of rotation
 	float FR = 1;	//FR, distance from Front Right Wheel to the center of rotation
 	float RL = 1;	//RL, distance from Rear Left Wheel to the center of rotation
 	float RR = 1;	//RR, distance from Rear Right Wheel to the center of rotation
  	float FRRatio = 1;	//Ratio of Speed of Front Right wheel
	float FLRatio = 1;	//Ratio of Speed of Front Left wheel
	float RRRatio = 1;	//Ratio of Speed of Rear Right wheel
	float RLRatio = 1;	//Ratio of Speed of Rear Left wheel

	angleRC = pi - m_steerAngle->GetRawAxis(m_steerAngleAxis);  //convert steering angle to rear center wheel angle
	
	if(angleRC != pi / 2)	//If we are not driving straight forward...
	{
		if(angleRC < pi / 2)	//Right Turn
		{
			turningRadius = ((m_steerMode * m_wheelBase) * tan(angleRC));				//find turning radius
			
			//calculate angles based on turning radius
			angleRL = atan((turningRadius + m_rearTrack/2) / (m_steerMode * m_wheelBase));
			angleRR = atan((turningRadius - m_rearTrack/2) / (m_steerMode * m_wheelBase));
				
			if(m_steerMode != 1)  //not turning about front wheels
			{
				angleFR = pi - atan((turningRadius - m_frontTrack/2) / ((1 - m_steerMode) * m_wheelBase));	//These are identical for right and left turns
				angleFL = pi - atan((turningRadius + m_frontTrack/2) / ((1 - m_steerMode) * m_wheelBase));	//These are identical for right and left turns
			}
			else
			{
				angleFR = pi/2;
				angleFL = pi/2;
			}
			
			//Solve each wheel turning radii (wheel speed)
			FL = (turningRadius + m_frontTrack/2) / sin(pi - angleFL);
			FR = (turningRadius - m_frontTrack/2) / sin(pi - angleFR);
			RL = (turningRadius + m_rearTrack/2) / sin(angleRL);
			RR = (turningRadius - m_rearTrack/2) / sin(angleRR);
;
		}
		else if(angleRC > pi / 2)	//Left Turn
		{
			turningRadius = ((m_steerMode * m_wheelBase) * tan(pi - angleRC));				//find turning radius
			
			//calculate angles based on turning radius
			angleRL = pi - atan((turningRadius - m_rearTrack/2) / (m_steerMode * m_wheelBase));
			angleRR = pi - atan((turningRadius + m_rearTrack/2) / (m_steerMode * m_wheelBase));
			
			if(m_steerMode != 1) //not turning about front wheels
			{
				angleFL = atan((turningRadius - m_frontTrack/2) / ((1 - m_steerMode) * m_wheelBase));	//These are identical for right and left turns
				angleFR = atan((turningRadius + m_frontTrack/2) / ((1 - m_steerMode) * m_wheelBase));	//These are identical for right and left turns
			}
			else
			{
				angleFR = pi/2;
				angleFL = pi/2;
			}

			//Solve each wheel turning radii (wheel speed)
			FL = (turningRadius - m_frontTrack/2) / sin(angleFL);
			FR = (turningRadius + m_frontTrack/2) / sin(angleFR);
			RL = (turningRadius - m_rearTrack/2) / sin(pi - angleRL);
			RR = (turningRadius + m_rearTrack/2) / sin(pi - angleRR);

		}
	}
	else	//angleRC = pi / 2
	{
		angleFL = pi / 2;
		angleFR = pi / 2;
		angleRL = pi / 2;
		angleRR = pi / 2;
		FLRatio = 1;
		FRRatio = 1;
		RLRatio = 1;
		RRRatio = 1;
	}
	
	
	//Solve for fastest wheel speed
	double speedarray[] = {fabs(FL), fabs(FR), fabs(RL), fabs(RR)};
		
	 int length = 4;
     double maxspeed = speedarray[0];
     for(int i = 1; i < length; i++)
     {
          if(speedarray[i] > maxspeed)
                maxspeed = speedarray[i];
     }
	   
	//Set ratios based on maximum wheel speed
	FLRatio = FL/maxspeed;
	FRRatio = FR/maxspeed;
	RLRatio = RL/maxspeed;
	RRRatio = RR/maxspeed;
	
	//Set drive speeds
	SetDriveSpeed(-FLRatio*m_steerSpeed->GetRawAxis(m_steerSpeedAxis), FRRatio*m_steerSpeed->GetRawAxis(m_steerSpeedAxis), -RLRatio*m_steerSpeed->GetRawAxis(m_steerSpeedAxis), RRRatio*m_steerSpeed->GetRawAxis(m_steerSpeedAxis));
	
	//Set Steering PID Setpoints
	float FLSetPoint = (1.25 + 2.5/pi*angleFL);
	float FRSetPoint = (1.25 + 2.5/pi*angleFR);
	float RLSetPoint = (1.25 + 2.5/pi*angleRL);
	float RRSetPoint = (1.25 + 2.5/pi*angleRR);
	
//	SetSteerSetpoint(FLSetPoint, FRSetPoint, RLSetPoint, RRSetPoint, false);
}

void BSSwerveController::SetDriveSpeed(float FLSpeed, float FRSpeed, float RLSpeed, float RRSpeed) {
//applies inversion variables defined in SetSteerSetPoint function	
	if(m_driveFront) {
		m_frontLeftDrive->Set(FLSpeed*m_FLInv);
		m_frontRightDrive->Set(FRSpeed*m_FRInv);
		m_rearLeftDrive->Set(RLSpeed*m_RLInv);
		m_rearRightDrive->Set(RRSpeed*m_RRInv);
	}
	else {
		m_frontLeftDrive->Set(RRSpeed*m_FLInv);
		m_frontRightDrive->Set(RLSpeed*m_FRInv);
		m_rearLeftDrive->Set(FRSpeed*m_RLInv);
		m_rearRightDrive->Set(FLSpeed*m_RRInv);
	}
}
void BSSwerveController::Lock() //locks wheels to prevent robot movement
{
	//SetSteering(3.0, 1.5, 3.0, 1.5, true);
	SetDriveSpeed(0,0,0,0);
}
