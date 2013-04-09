#include "CoutLong.h"

static const char* coutlong_spec[] = { //
		//
				"implementation_id", "CoutLong", //
				"type_name", "CoutLong", //
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

CoutLong::CoutLong(RTC::Manager* manager) :
		RTC::DataFlowComponentBase(manager),
		m_inIn("in", m_in)
{
}

CoutLong::~CoutLong() {
}

RTC::ReturnCode_t CoutLong::onInitialize() {
	addInPort("in", m_inIn);
	return RTC::RTC_OK;
}

RTC::ReturnCode_t CoutLong::onActivated(RTC::UniqueId ec_id) {
	return RTC::RTC_OK;
}

RTC::ReturnCode_t CoutLong::onDeactivated(RTC::UniqueId ec_id) {
	return RTC::RTC_OK;
}

RTC::ReturnCode_t CoutLong::onExecute(RTC::UniqueId ec_id) {
	  if (m_inIn.isNew())
	    {
	      m_inIn.read();
	      std::cout << "RTC_lng: " << m_in.data << std::endl;
	    }
	  //coil::usleep(100);

	return RTC::RTC_OK;
}

extern "C" {

void CoutLongInit(RTC::Manager* manager) {
	coil::Properties profile(coutlong_spec);
	manager->registerFactory(profile, RTC::Create<CoutLong>, RTC::Delete<CoutLong>);
}

}
;

