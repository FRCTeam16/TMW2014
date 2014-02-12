/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#include "BSGyro.h"
#include "AnalogChannel.h"
#include "AnalogModule.h"
#include "NetworkCommunication/UsageReporting.h"
#include "Timer.h"
#include "WPIErrors.h"
#include "LiveWindow/LiveWindow.h"

const uint32_t BSGyro::kOversampleBits;
const uint32_t BSGyro::kAverageBits;
constexpr float BSGyro::kSamplesPerSecond;
constexpr float BSGyro::kCalibrationSampleTime;
constexpr float BSGyro::kDefaultVoltsPerDegreePerSecond;

/**
 * Initialize the gyro.
 * Calibrate the gyro by running for a number of samples and computing the center value for this
 * part. Then use the center value as the Accumulator center value for subsequent measurements.
 * It's important to make sure that the robot is not moving while the centering calculations are
 * in progress, this is typically done when the robot is first turned on while it's sitting at
 * rest before the competition starts.
 */
void BSGyro::InitGyro()
{
	m_table = NULL;
	if (!m_analog->IsAccumulatorChannel())
	{
		wpi_setWPIErrorWithContext(ParameterOutOfRange,
				"moduleNumber and/or channel (must be accumulator channel)");
		if (m_channelAllocated)
		{
			delete m_analog;
			m_analog = NULL;
		}
		return;
	}

	m_voltsPerDegreePerSecond = kDefaultVoltsPerDegreePerSecond;
	m_analog->SetAverageBits(kAverageBits);
	m_analog->SetOversampleBits(kOversampleBits);
	float sampleRate = kSamplesPerSecond * 
		(1 << (kAverageBits + kOversampleBits));
	m_analog->GetModule()->SetSampleRate(sampleRate);
	Wait(1.0);

	m_analog->InitAccumulator();
	Wait(kCalibrationSampleTime);

	INT64 value;
	uint32_t count;
	m_analog->GetAccumulatorOutput(&value, &count);

	m_center = (uint32_t)((float)value / (float)count + .5);

	m_offset = ((float)value / (float)count) - (float)m_center;

	m_analog->SetAccumulatorCenter(m_center);
	m_analog->SetAccumulatorDeadband(0); ///< TODO: compute / parameterize this
	m_analog->ResetAccumulator();
	
	SetPIDSourceParameter(kAngle);

	nUsageReporting::report(nUsageReporting::kResourceType_Gyro, m_analog->GetChannel(), m_analog->GetModuleNumber() - 1);
	LiveWindow::GetInstance()->AddSensor("BSGyro", m_analog->GetModuleNumber(), m_analog->GetChannel(), this);
}

/**
 * BSGyro constructor given a slot and a channel.
 * 
 * @param moduleNumber The analog module the gyro is connected to (1).
 * @param channel The analog channel the gyro is connected to (1 or 2).
 */
BSGyro::BSGyro(uint8_t moduleNumber, uint32_t channel)
{
	m_analog = new AnalogChannel(moduleNumber, channel);
	m_channelAllocated = true;
	InitGyro();
}

/**
 * BSGyro constructor with only a channel.
 * 
 * Use the default analog module slot.
 * 
 * @param channel The analog channel the gyro is connected to.
 */
BSGyro::BSGyro(uint32_t channel)
{
	m_analog = new AnalogChannel(channel);
	m_channelAllocated = true;
	InitGyro();
}

BSGyro::BSGyro(uint8_t moduleNumber, uint32_t channel, uint32_t center, float offset) {
	m_analog = new AnalogChannel(moduleNumber, channel);
	m_channelAllocated = true;

	m_table = NULL;
	if (!m_analog->IsAccumulatorChannel())
	{
		wpi_setWPIErrorWithContext(ParameterOutOfRange,
				"moduleNumber and/or channel (must be accumulator channel)");
		if (m_channelAllocated)
		{
			delete m_analog;
			m_analog = NULL;
		}
		return;
	}

	m_voltsPerDegreePerSecond = kDefaultVoltsPerDegreePerSecond;
	m_analog->SetAverageBits(kAverageBits);
	m_analog->SetOversampleBits(kOversampleBits);
	float sampleRate = kSamplesPerSecond * 
		(1 << (kAverageBits + kOversampleBits));
	m_analog->GetModule()->SetSampleRate(sampleRate);
	Wait(1.0);

	m_analog->InitAccumulator();
	m_center = center;

	m_offset = offset;

	m_analog->SetAccumulatorCenter(m_center);
	m_analog->SetAccumulatorDeadband(0); ///< TODO: compute / parameterize this
	m_analog->ResetAccumulator();
	
	SetPIDSourceParameter(kAngle);

	nUsageReporting::report(nUsageReporting::kResourceType_Gyro, m_analog->GetChannel(), m_analog->GetModuleNumber() - 1);
	LiveWindow::GetInstance()->AddSensor("BSGyro", m_analog->GetModuleNumber(), m_analog->GetChannel(), this);
}

/**
 * BSGyro constructor with a precreated analog channel object.
 * Use this constructor when the analog channel needs to be shared. There
 * is no reference counting when an AnalogChannel is passed to the gyro.
 * @param channel The AnalogChannel object that the gyro is connected to.
 */
BSGyro::BSGyro(AnalogChannel *channel)
{
	m_analog = channel;
	m_channelAllocated = false;
	if (channel == NULL)
	{
		wpi_setWPIError(NullParameter);
	}
	else
	{
		InitGyro();
	}
}

BSGyro::BSGyro(AnalogChannel &channel)
{
	m_analog = &channel;
	m_channelAllocated = false;
	InitGyro();
}

/**
 * Reset the gyro.
 * Resets the gyro to a heading of zero. This can be used if there is significant
 * drift in the gyro and it needs to be recalibrated after it has been running.
 */
void BSGyro::Reset()
{
	m_analog->ResetAccumulator();
}

/**
 * Delete (free) the accumulator and the analog components used for the gyro.
 */
BSGyro::~BSGyro()
{
	if (m_channelAllocated)
		delete m_analog;
}

/**
 * Return the actual angle in degrees that the robot is currently facing.
 * 
 * The angle is based on the current accumulator value corrected by the oversampling rate, the
 * gyro type and the A/D calibration values.
 * The angle is continuous, that is can go beyond 360 degrees. This make algorithms that wouldn't
 * want to see a discontinuity in the gyro output as it sweeps past 0 on the second time around.
 * 
 * @return the current heading of the robot in degrees. This heading is based on integration
 * of the returned rate from the gyro.
 */
float BSGyro::GetAngle( void )
{
	INT64 rawValue;
	uint32_t count;
	m_analog->GetAccumulatorOutput(&rawValue, &count);

	INT64 value = rawValue - (INT64)((float)count * m_offset);

	double scaledValue = value * 1e-9 * (double)m_analog->GetLSBWeight() * (double)(1 << m_analog->GetAverageBits()) /
		(m_analog->GetModule()->GetSampleRate() * m_voltsPerDegreePerSecond);

	return (float)scaledValue;
}


/**
 * Return the rate of rotation of the gyro
 * 
 * The rate is based on the most recent reading of the gyro analog value
 * 
 * @return the current rate in degrees per second
 */
double BSGyro::GetRate( void )
{
	return (m_analog->GetAverageValue() - ((double)m_center + m_offset)) * 1e-9 * m_analog->GetLSBWeight() 
			/ ((1 << m_analog->GetOversampleBits()) * m_voltsPerDegreePerSecond);
}


/**
 * Set the gyro type based on the sensitivity.
 * This takes the number of volts/degree/second sensitivity of the gyro and uses it in subsequent
 * calculations to allow the code to work with multiple gyros.
 * 
 * @param voltsPerDegreePerSecond The type of gyro specified as the voltage that represents one degree/second.
 */
void BSGyro::SetSensitivity( float voltsPerDegreePerSecond )
{
	m_voltsPerDegreePerSecond = voltsPerDegreePerSecond;
}

void BSGyro::SetPIDSourceParameter(PIDSourceParameter pidSource)
{
	if(pidSource == 0 || pidSource > 2)
		wpi_setWPIErrorWithContext(ParameterOutOfRange, "BSGyro pidSource");
    m_pidSource = pidSource;
}

/**
 * Get the angle in degrees for the PIDSource base object.
 * 
 * @return The angle in degrees.
 */
double BSGyro::PIDGet()
{
	switch(m_pidSource){
	case kRate:
		return GetRate();
	case kAngle:
		return GetAngle();
	default:
		return 0;
	}
}

void BSGyro::UpdateTable() {
	if (m_table != NULL) {
		m_table->PutNumber("Value", GetAngle());
	}
}

void BSGyro::StartLiveWindowMode() {
	
}

void BSGyro::StopLiveWindowMode() {
	
}

std::string BSGyro::GetSmartDashboardType() {
	return "BSGyro";
}

void BSGyro::InitTable(ITable *subTable) {
	m_table = subTable;
	UpdateTable();
}

ITable * BSGyro::GetTable() {
	return m_table;
}

uint32_t BSGyro::GetCenter() {
	return m_center;
}

float BSGyro::GetOffset() {
	return m_offset;
}

