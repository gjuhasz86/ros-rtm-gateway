#ifndef COUT_STRING_H
#define COUT_STRING_H

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

class CoutString: public RTC::DataFlowComponentBase {
public:
	CoutString(RTC::Manager* manager);

	~CoutString();

	virtual RTC::ReturnCode_t onInitialize();
	virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
	TimedString m_in;
	InPort<TimedString> m_inIn;
};

extern "C" {
DLL_EXPORT void CoutStringInit(RTC::Manager* manager);
}
;

#endif // COUT_STRING_H
