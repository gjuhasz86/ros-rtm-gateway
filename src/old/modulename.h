// -*- C++ -*-
/*!
 * @file  modulename.h
 * @brief ModuleDescription
 * @date  $Date$
 *
 * $Id$
 */

#ifndef MODULENAME_H
#define MODULENAME_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>
#include "BasicDataTypeStub.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <boost/bind.hpp>
#include <boost/function.hpp>

using namespace RTC;

class modulename: public RTC::DataFlowComponentBase {
public:
	modulename(RTC::Manager* manager);

	~modulename();

	virtual RTC::ReturnCode_t onInitialize();

	virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

	virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

	virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

	virtual void chatterCallback(const std_msgs::Int32::ConstPtr& msg);

protected:
	TimedLong m_v_out;
	OutPort<TimedLong> m_v_outOut;
	ros::NodeHandle n;
	ros::Subscriber sub;

private:

};

extern "C" {
DLL_EXPORT void modulenameInit(RTC::Manager* manager);
}
;

#endif // MODULENAME_H
