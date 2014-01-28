#ifndef BSSWERVECONTROLLER_
#define BSSWERVECONTROLLER_

#include "WPILib.h"
#include "Math.h"
#include <fstream>
#include <sstream>

class BSSwerveController {
public:
	typedef enum {kSteer, kCrab, kLock}DriveMode;
	BSSwerveController(PIDController* frontLeft, PIDController* frontRight, PIDController* rearLeft, PIDController* rearRight, 
			SpeedController* frontLeftDrive, SpeedController* frontRightDrive, SpeedController* rearLeftDrive, SpeedController* rearRightDrive, Gyro* gyro, 
			Joystick* driverJoystickL, Joystick* driverJoystickR);
	void SetWheelbase(float frontTrack, float rearTrack, float wheelBase);
	void SetOffsets();
	void SetDriveMode(DriveMode driveMode);
	void SetSteerMode(float steerMode);
	void ToggleFrontBack();
	void Enable();
	void Disable();
	bool IsEnabled();
	void SetSteerSpeedAxis(Joystick* steerspeed, int axis);
	void SetSteerAngleAxis(Joystick* steerangle, int axis);

private:
	PIDController* m_frontLeft;
	PIDController* m_frontRight;
	PIDController* m_rearLeft;
	PIDController* m_rearRight;
	SpeedController* m_frontLeftDrive;
	SpeedController* m_frontRightDrive;
	SpeedController* m_rearLeftDrive;
	SpeedController* m_rearRightDrive;
	Gyro* m_gyro;
	Joystick* m_driverJoystickL;
	Joystick* m_driverJoystickR;
	DriveMode m_driveMode;
	Task* m_task;
	bool m_enabled;
	static void PrivateStarter(BSSwerveController *task);
	void Run();
	
			
	//Steering Functions
	void Steer();
	void Crab(float twist, float y, float x, bool UseGyro);
	void Lock();
	void LeftTurn4Wheels();	//Calculates a left hand turn
	void RightTurn4Wheels();	//Calculates a right hand turn
	float CorrectSteerSetpoint(float setpoint);
	void SetSteerSetpoint(float FLSetPoint, float FRSetPoint, float RLSetPoint, float RRSetPoint, bool UseShortcut);
//	void SetSteerSetpoint(float setpoint, AnalogChannel* actual, float offset, PIDController* PIDCon, int* inv, bool UseShortcut);
	void SetDriveSpeed(float FLSpeed, float FRSpeed, float RLSpeed, float RRSpeed);
	void SetSteering(float FLSetPoint, float FRSetPoint, float RLSetPoint, float RRSetPoint, bool UseShortcut);
	void RetrieveOffsetsFromFile();

	
	Joystick* m_steerSpeed;
	int m_steerSpeedAxis;
	Joystick* m_steerAngle;
	int m_steerAngleAxis;

	static const float pi=3.14159;

	//DriveDirection
	bool m_driveFront;

	//Steering Variables	
	float m_frontTrack;	//Distance between the front
	float m_rearTrack;	//Distance between the rear wheels
	float m_wheelBase;	//Distance between the front and back wheels
	float m_steerMode;	//The point along the the length of the robot about which the robot turns.  
						//0.0 is at the back wheels - creating front wheel steering
						//0.5 is at the center - crating 4 wheel steering
						//1.0 is at the front wheels
						//values outside of 0-1 can do interesting things...
	

	//Varibles store the inverted state of each wheel module (-1 = running "backwards")
	int m_FLInv;
	int m_FRInv;
	int m_RLInv;
	int m_RRInv;

	//Crab Variables
	float AP;
	float BP;
	float CP;
	float DP;
	float radius; //distance from center to each wheel
	float robotangle; //current robot angle from Gyro	
	
	map<string, float> m_Offset;
};

#endif 
