#ifndef GATEWAY_H
#define GATEWAY_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>

#include <cstring>
#include <vector>
#include <string>
#include <iostream>

#include "BasicDataTypeStub.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include "GatewayHelper.h"

using namespace RTC;

class Gateway: public RTC::DataFlowComponentBase {
private:

	GatewayFactory::Config config;


public:
	//Gateway(RTC::Manager* manager);
	Gateway(RTC::Manager* manager);

	~Gateway();

	virtual RTC::ReturnCode_t onInitialize();
	virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

	virtual void init();
};


/*
extern "C" {
	DLL_EXPORT void HybridInit(RTC::Manager* manager);
}
;
*/

////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION

Gateway::Gateway(RTC::Manager* manager) :
		RTC::DataFlowComponentBase(manager){
}

Gateway::~Gateway() {
}

////////////////////////////////////////////////////////////////////////////////
// Public methods - methods comes with the RT component


void Gateway::init(){

	boost::function2<bool, const char*, OutPortBase&> addOutPortFn = boost::bind(&Gateway::addOutPort, this, _1, _2);
	config.setRegisterRtcOutPortFn(addOutPortFn);

	//config.createNewRosToRtmLink()
}

RTC::ReturnCode_t Gateway::onInitialize() {
	//config.createRtcOutPorts(this);
	//config.createRtcInPorts(this);
	init();
	config.init();
	config.doRegisterRtcOutPort();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onActivated(RTC::UniqueId ec_id) {
	config.doSubscibe();
	config.doAdvertise();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onDeactivated(RTC::UniqueId ec_id) {
	config.doUnsubscribe();
	config.doStopAdvertise();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onExecute(RTC::UniqueId ec_id) {
	ros::spinOnce();
	config.onExec();
	return RTC::RTC_OK;
}

#endif // GATEWAY_H
