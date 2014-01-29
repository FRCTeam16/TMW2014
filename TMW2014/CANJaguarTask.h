#ifndef _CANJAGUARTASK_H
#define _CANJAGUARTASK_H
#include "WPILib.h"

class CANJaguarTask {
public:
	CANJaguarTask(int address);
	virtual ~CANJaguarTask(void);
	void SetOutput(float output);
	CANJaguar* Get();
private:
	void Run();
	static void PrivateStarter(CANJaguarTask *task);
	bool CheckJaguar();
	float m_output;
	CANJaguar* m_jaguar;
	Task* m_task;
};

#endif
