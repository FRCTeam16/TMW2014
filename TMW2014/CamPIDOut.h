#ifndef CAMPIDOUT_H
#define CAMPIDOUT_H

#include "PIDOutput.h"
#include "WPILib.h"

class CamPIDOut : public PIDOutput
{
public:
	CamPIDOut(CANJaguar *jag1, CANJaguar *jag2, CANJaguar *jag3);
	void PIDWrite(float output);
	void SetMultiplier(float multiplier); 
	float Get();
	
private:
	float m_output;
	float m_multiplier;
	CANJaguar* m_jag1;
	CANJaguar* m_jag2;
	CANJaguar* m_jag3;
};

#endif
