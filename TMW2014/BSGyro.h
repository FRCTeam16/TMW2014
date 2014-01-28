/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef BSGYRO_H_
#define BSGYRO_H_

#include "SensorBase.h"
#include "PIDSource.h"
#include "LiveWindow/LiveWindowSendable.h"

class AnalogChannel;
class AnalogModule;

/**
 * Use a rate gyro to return the robots heading relative to a starting position.
 * The BSGyro class tracks the robots heading based on the starting position. As the robot
 * rotates the new heading is computed by integrating the rate of rotation returned
 * by the sensor. When the class is instantiated, it does a short calibration routine
 * where it samples the gyro while at rest to determine the default offset. This is
 * subtracted from each sample to determine the heading. This gyro class must be used 
 * with a channel that is assigned one of the Analog accumulators from the FPGA. See
 * AnalogChannel for the current accumulator assignments.
 */
class BSGyro : public SensorBase, public PIDSource, public LiveWindowSendable
{
public:
	static const uint32_t kOversampleBits = 10;
	static const uint32_t kAverageBits = 0;
	static constexpr float kSamplesPerSecond = 50.0;
	static constexpr float kCalibrationSampleTime = 5.0;
	static constexpr float kDefaultVoltsPerDegreePerSecond = 0.007;

	BSGyro(uint8_t moduleNumber, uint32_t channel);
	BSGyro(uint8_t moduleNumber, uint32_t channel, uint32_t center, uint32_t offset);
	explicit BSGyro(uint32_t channel);
	explicit BSGyro(AnalogChannel *channel);
	explicit BSGyro(AnalogChannel &channel);
	virtual ~BSGyro();
	virtual float GetAngle();
	virtual double GetRate();
	void SetSensitivity(float voltsPerDegreePerSecond);
	void SetPIDSourceParameter(PIDSourceParameter pidSource);
	virtual void Reset();
	uint32_t GetCenter();
	uint32_t GetOffset();

	// PIDSource interface
	double PIDGet();
	
	void UpdateTable();
	void StartLiveWindowMode();
	void StopLiveWindowMode();
	std::string GetSmartDashboardType();
	void InitTable(ITable *subTable);
	ITable * GetTable();
	
	void InitGyro();

private:

	AnalogChannel *m_analog;
	float m_voltsPerDegreePerSecond;
	float m_offset;
	bool m_channelAllocated;
	uint32_t m_center;
	PIDSourceParameter m_pidSource;
	
	ITable *m_table;
};
#endif
