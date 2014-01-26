#ifndef _JAGUARSETTASK_H
#define _JAGUARSETTASK_H
#include "WPILib.h"

class JaguarSetTask {
public:
	JaguarSetTask(const char* name, CANJaguar *jaguar);
	virtual ~JaguarSetTask(void);
	void SetOutput(float output);
private:
	void Run();
	static void PrivateStarter(JaguarSetTask *task);
	bool CheckJaguar();
	float m_output;
	CANJaguar* m_jaguar;
	Task* m_task;
};

#endif
