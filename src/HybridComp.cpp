// -*- C++ -*-
/*!
 * @file HybridComp.cpp
 * @brief Standalone component
 * @date $Date$
 *
 * $Id$
 */

#include <rtm/Manager.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include "Gateway.h"

static const char* gateway_spec[] = { //
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

void convert1(const boost::shared_ptr<std_msgs::Int32 const>& in, TimedLong& out) {
	out.data = in->data;
}

void convert2(const boost::shared_ptr<std_msgs::String const>& in, TimedString& out) {
	out.data = in->data.c_str();
}

void convert3(const TimedLong& in, std_msgs::Int32& out) {
	out.data = in.data;
}

void callback(const boost::shared_ptr<std_msgs::Int32 const>& in, TimedLong& out, const RosToRtmLink<TimedLong>& link) {
	std::cout << "[";
	std::cout << link.name.c_str();
	std::cout << "] : [";
	std::cout << boost::lexical_cast<std::string>(in->data).c_str();
	std::cout << "]->[";
	std::cout << boost::lexical_cast<std::string>(out.data).c_str() << "]";
	std::cout << std::endl;
}

 class MyGateway:public Gateway{
 public:
 MyGateway(RTC::Manager* manager) :
 Gateway(manager){
 }

 virtual void setUpPorts(){
 RosToRtmConverter<std_msgs::Int32, TimedLong> c1(&convert1, &callback);
 config.addNewRosToRtmLink<std_msgs::Int32, TimedLong>("chatterInt1", c1);
 }
 };


GatewayFactory::Config* config2;

template<class _New>
RTC::RTObject_impl* CreateGateway(RTC::Manager* manager) {
	std::cout << "creating" << std::endl;
	return new _New(manager);
}


void HybridInit(RTC::Manager* manager, GatewayFactory::Config* config) {
	coil::Properties profile(gateway_spec);
	manager->registerFactory(profile, CreateGateway<MyGateway>, RTC::Delete<MyGateway>);
}

void MyModuleInit(RTC::Manager* manager) {
	std::cout << "Starting Hybrid" << std::endl;

	HybridInit(manager, config2);
	RTC::RtcBase* comp;

	comp = manager->createComponent("Hybrid");

	if (comp == NULL) {
		std::cerr << "Component create failed." << std::endl;
		abort();
	}

	return;
}

void createNewGateway(int argc, char** argv, bool block = true) {
	RTC::Manager* manager;
	manager = RTC::Manager::init(argc, argv);
	manager->init(argc, argv);

	manager->setModuleInitProc(MyModuleInit);
	manager->activateManager();
	manager->runManager(!block);
}

int main(int argc, char** argv) {
	std::cout << "Starting" << std::endl;
	ros::init(argc, argv, "Gateway", ros::init_options::NoSigintHandler);

	//config2 = new GatewayFactory::Config(gateway_spec);

	//RosToRtmConverter<std_msgs::Int32, TimedLong> c1(&convert1, &callback);
	//config2->addNewRosToRtmLink<std_msgs::Int32, TimedLong>("chatterInt1", c1);

	//GatewayFactory::createNewGateway<Gateway>(argc, argv, true);
	createNewGateway(argc, argv, true);

	return 0;
}
