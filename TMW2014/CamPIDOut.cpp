#include "CamPIDOut.h"

CamPIDOut::CamPIDOut(CANJaguar *jag1, CANJaguar *jag2, CANJaguar *jag3) :
m_output(0)
{
	m_jag1 = jag1;
	m_jag2 = jag2;
	m_jag3 = jag3;
}

void CamPIDOut::PIDWrite(float output) {
	m_output = output;
	m_jag1->Set(m_output);
	m_jag2->Set(m_multiplier*m_output);
	m_jag3->Set(m_multiplier*m_output);
}

void CamPIDOut::SetMultiplier(float multiplier) {
	m_multiplier = multiplier;
}

float CamPIDOut::Get() {
	return m_output;
}
