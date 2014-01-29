/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2011. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __BS_CHOOSER_H__
#define __BSCHOOSER_H__

#include "SmartDashboard/Sendable.h"
#include "tables/ITable.h"
#include <map>
#include <string>
#include "Autoprogramming.h"

/**
 * The {@link BSChooser} class is a useful tool for presenting a selection of options
 * to the {@link SmartDashboard}.
 *
 * <p>For instance, you may wish to be able to select between multiple autonomous modes.
 * You can do this by putting every possible {@link Command} you want to run as an autonomous into
 * a {@link BSChooser} and then put it into the {@link SmartDashboard} to have a list of options
 * appear on the laptop.  Once autonomous starts, simply ask the {@link BSChooser} what the selected
 * value is.</p>
 *
 * @see SmartDashboard
 */
class BSChooser : public Sendable
{
public:
	BSChooser();
	virtual ~BSChooser() {};

	void AddObject(const char *name, AutoProgram object);
	void AddDefault(const char *name, AutoProgram object);
	AutoProgram GetSelected();

	virtual void InitTable(ITable* subtable);
	virtual ITable* GetTable();
	virtual std::string GetSmartDashboardType();

private:
	std::string m_defaultChoice;
	std::map<std::string, AutoProgram> m_choices;
	ITable *m_table;
};

#endif
