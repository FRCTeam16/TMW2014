#include "JaguarSetTask.h"
#include "WPILib.h"
#include <iostream.h>
#include <stdio.h>

JaguarSetTask::JaguarSetTask(const char* name, CANJaguar *jaguar)
{
	m_jaguar = jaguar;
	m_output = 0;
	m_task = new Task(name, (FUNCPTR)JaguarSetTask::PrivateStarter);
	if (!m_task->Start((INT32)this));
}

JaguarSetTask::~JaguarSetTask()
{
	
}

void JaguarSetTask::SetOutput(float output)
{
	m_output = output;
}

void JaguarSetTask::PrivateStarter(JaguarSetTask *task)
{
	while (1) {
		if(task->CheckJaguar())
		{
			task->Run();
			Wait(0.02);
		}
		else
		{
			cout << "Jaguar is dead" << endl;
			printf ("Jaguar is Dead");
			Wait(1.0);
		}
	}
}

void JaguarSetTask::Run()
{
	m_jaguar->Set(m_output);
}

bool JaguarSetTask::CheckJaguar()
{
	return m_jaguar->IsAlive();
}
