#include "Gateway.h"


////////////////////////////////////////////////////////////////////////////////
// Constructor and Destructor

Gateway::Gateway(RTC::Manager* manager, GatewayFactory::Config<Gateway>* config) :
		RTC::DataFlowComponentBase(manager), config(*config){
}

Gateway::~Gateway() {
}

////////////////////////////////////////////////////////////////////////////////
// Public methods - methods comes with the RT component

RTC::ReturnCode_t Gateway::onInitialize() {
	init();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onActivated(RTC::UniqueId ec_id) {
	//doSubscibe();
	//doAdvertise();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onDeactivated(RTC::UniqueId ec_id) {
	//doUnsubscribe();
	//doStopAdvertise();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onExecute(RTC::UniqueId ec_id) {
	ros::spinOnce();
	//onExec();
	return RTC::RTC_OK;
}

////////////////////////////////////////////////////////////////////////////////
// Protected methods - methods specific to the gateway component


template <class _New>
RTObject_impl* Create(Manager* manager)
{
  return new _New(manager);
}

template <class _Delete>
void Delete(RTObject_impl* rtc)
{
  delete rtc;
}


/*
extern "C" {

void HybridInit(RTC::Manager* manager) {
	coil::Properties profile(hybrid_spec);
	manager->registerFactory(profile, RTC::Create<Gateway>, RTC::Delete<Gateway>);
}

}
;

*/
