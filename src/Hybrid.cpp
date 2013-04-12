#include "Hybrid.h"

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

Hybrid::Hybrid(RTC::Manager* manager) :
		RTC::DataFlowComponentBase(manager) {
}

Hybrid::~Hybrid() {
}

RTC::ReturnCode_t Hybrid::onInitialize() {
	boost::function2<bool, const char*, OutPortBase&> addOutPortFn = boost::bind(&Hybrid::addOutPort, this, _1, _2);
	setRegisterRtcOutPortFn(addOutPortFn);

	boost::function2<bool, const char*, InPortBase&> addInPortFn = boost::bind(&Hybrid::addInPort, this, _1, _2);
	setRegisterRtcInPortFn(addInPortFn);

	init();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Hybrid::onActivated(RTC::UniqueId ec_id) {
	doSubscibe();
	doAdvertise();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Hybrid::onDeactivated(RTC::UniqueId ec_id) {
	doUnsubscribe();
	doStopAdvertise();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Hybrid::onExecute(RTC::UniqueId ec_id) {
	ros::spinOnce();
	onExec();
	return RTC::RTC_OK;
}

extern "C" {

void HybridInit(RTC::Manager* manager) {
	coil::Properties profile(hybrid_spec);
	manager->registerFactory(profile, RTC::Create<Hybrid>, RTC::Delete<Hybrid>);
}

}
;

