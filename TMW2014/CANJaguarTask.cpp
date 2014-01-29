#include "CANJaguarTask.h"
#include "WPILib.h"
#include <iostream.h>
#include <stdio.h>

CANJaguarTask::CANJaguarTask(int address)
{
	m_jaguar = new CANJaguar(address);
	m_output = 0;
	m_task = new Task(strcat("CANJaguarTask at Address: ", (const char*) address), (FUNCPTR)CANJaguarTask::PrivateStarter);
	if (!m_task->Start((INT32)this));
}

CANJaguarTask::~CANJaguarTask()
{
	
}

void CANJaguarTask::SetOutput(float output)
{
	m_output = output;
}

void CANJaguarTask::PrivateStarter(CANJaguarTask *task)
{
	while (1) {
		if(task->CheckJaguar())
		{
			task->Run();
			Wait(0.02);
		}
		else
		{
			printf ("Jaguar is Dead");
			Wait(1.0);
		}
	}
}

void CANJaguarTask::Run()
{
	m_jaguar->Set(m_output);
}

bool CANJaguarTask::CheckJaguar()
{
	return m_jaguar->IsAlive();
}
CANJaguar* CANJaguarTask::Get() {
	return m_jaguar;
}
