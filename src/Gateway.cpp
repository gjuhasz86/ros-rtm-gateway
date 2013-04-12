#include "Gateway.h"

static const char* hybrid_spec[] = { //
		//
				"implementation_id", "Hybrid", //
				"type_name", "Hybrid", //
				"description", "A hybrid ROS RTC module", //
				"version", "1.0.0", //
				"vendor", "Gabor Juhasz", //
				"category", "Category", //
				"activity_type", //
				"PERIODIC", //
				"kind", "DataFlowComponent", //
				"max_instance", "1", //
				"language", "C++", //
				"lang_type", "compile", //
				"" };

////////////////////////////////////////////////////////////////////////////////
// Constructor and Destructor

Gateway::Gateway(RTC::Manager* manager) :
		RTC::DataFlowComponentBase(manager) {
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
	doSubscibe();
	doAdvertise();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onDeactivated(RTC::UniqueId ec_id) {
	doUnsubscribe();
	doStopAdvertise();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onExecute(RTC::UniqueId ec_id) {
	ros::spinOnce();
	onExec();
	return RTC::RTC_OK;
}

////////////////////////////////////////////////////////////////////////////////
// Protected methods - methods specific to the gateway component




extern "C" {

void HybridInit(RTC::Manager* manager) {
	coil::Properties profile(hybrid_spec);
	manager->registerFactory(profile, RTC::Create<Gateway>, RTC::Delete<Gateway>);
}

}
;

