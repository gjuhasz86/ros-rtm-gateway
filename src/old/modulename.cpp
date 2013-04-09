// -*- C++ -*-
/*!
 * @file  modulename.cpp
 * @brief ModuleDescription
 * @date $Date$
 *
 * $Id$
 */

#include "modulename.h"
#include <cstring>

// Module specification
static const char* modulename_spec[] = { "implementation_id", "modulename",
		"type_name", "modulename", "description", "ModuleDescription",
		"version", "1.0.0", "vendor", "vendername", "category", "Category",
		"activity_type", "PERIODIC", "kind", "DataFlowComponent",
		"max_instance", "1", "language", "C++", "lang_type", "compile", "" };

void convert(const std_msgs::Int32::ConstPtr& in, TimedLong& out) {
	out.data = in->data;
}

void modulename::chatterCallback(const std_msgs::Int32::ConstPtr& msg) {
	ROS_INFO("I heard: [%d]", msg->data);
	convert(msg, m_v_out);
	m_v_outOut.write();
}

modulename::modulename(RTC::Manager* manager) :
		RTC::DataFlowComponentBase(manager), m_v_outOut("out", m_v_out) {
}

modulename::~modulename() {
}

RTC::ReturnCode_t modulename::onInitialize() {
	addOutPort("out", m_v_outOut);
	std::cout << "onInitialize is called." << std::endl;
	return RTC::RTC_OK;
}

RTC::ReturnCode_t modulename::onActivated(RTC::UniqueId ec_id) {
	std::cout << "onActivated is called." << std::endl;

	sub = n.subscribe("chatter", 1000, &modulename::chatterCallback, this);
	return RTC::RTC_OK;
}

RTC::ReturnCode_t modulename::onDeactivated(RTC::UniqueId ec_id) {
	std::cout << "onDeactivated is called." << std::endl;
	sub.shutdown();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t modulename::onExecute(RTC::UniqueId ec_id) {
	std::cout << "onExecute is called." << std::endl;
	ros::spinOnce();
	return RTC::RTC_OK;
}

extern "C" {

void modulenameInit(RTC::Manager* manager) {
	coil::Properties profile(modulename_spec);
	manager->registerFactory(profile, RTC::Create<modulename>,
			RTC::Delete<modulename>);
}

}
;

