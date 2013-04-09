#include "CoutString.h"

static const char* coutstring_spec[] = { //
		//
				"implementation_id", "CoutString", //
				"type_name", "CoutString", //
				"description", "desc", //
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

CoutString::CoutString(RTC::Manager* manager) :
		RTC::DataFlowComponentBase(manager), m_inIn("in", m_in) {
}

CoutString::~CoutString() {
}

RTC::ReturnCode_t CoutString::onInitialize() {
	addInPort("in", m_inIn);
	return RTC::RTC_OK;
}

RTC::ReturnCode_t CoutString::onActivated(RTC::UniqueId ec_id) {
	return RTC::RTC_OK;
}

RTC::ReturnCode_t CoutString::onDeactivated(RTC::UniqueId ec_id) {
	return RTC::RTC_OK;
}

RTC::ReturnCode_t CoutString::onExecute(RTC::UniqueId ec_id) {
	if (m_inIn.isNew()) {
		m_inIn.read();
		std::cout << "RTC_str: " << m_in.data << std::endl;
	}
	//coil::usleep(100);

	return RTC::RTC_OK;
}

extern "C" {

void CoutStringInit(RTC::Manager* manager) {
	coil::Properties profile(coutstring_spec);
	manager->registerFactory(profile, RTC::Create<CoutString>, RTC::Delete<CoutString>);
}

}
;

