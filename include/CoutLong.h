#ifndef COUT_LONG_H
#define COUT_LONG_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

#include <iostream>


using namespace RTC;

class CoutLong: public RTC::DataFlowComponentBase {
public:
	CoutLong(RTC::Manager* manager);

	~CoutLong();

	virtual RTC::ReturnCode_t onInitialize();
	virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
	  TimedLong m_in;
	  InPort<TimedLong> m_inIn;
};

extern "C" {
DLL_EXPORT void CoutLongInit(RTC::Manager* manager);
}
;

#endif // COUT_LONG_H
