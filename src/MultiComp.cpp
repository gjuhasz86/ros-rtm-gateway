#include <rtm/Manager.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include "Gateway.h"
#include "CoutLong.h"
#include "CoutString.h"
/*
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

void HybridInit(RTC::Manager* manager) {
	coil::Properties profile(hybrid_spec);
	manager->registerFactory(profile, RTC::Create<Gateway>, RTC::Delete<Gateway>);
}

void MyModuleInit(RTC::Manager* manager) {
	std::cout << "Starting Gateway" << std::endl;
	HybridInit(manager);
	RTC::RtcBase* comp;

	comp = manager->createComponent("Gateway");

	if (comp == NULL) {
		std::cerr << "Component create failed." << std::endl;
		abort();
	}

	return;
}

void MyModuleInit2(RTC::Manager* manager) {
	std::cout << "Starting CoutLong" << std::endl;
	CoutLongInit(manager);
	RTC::RtcBase* comp;

	comp = manager->createComponent("CoutLong");

	if (comp == NULL) {
		std::cerr << "Component create failed." << std::endl;
		abort();
	}

	return;
}

void MyModuleInit3(RTC::Manager* manager) {
	std::cout << "Starting CoutString" << std::endl;
	CoutStringInit(manager);
	RTC::RtcBase* comp;

	comp = manager->createComponent("CoutString");

	if (comp == NULL) {
		std::cerr << "Component create failed." << std::endl;
		abort();
	}

	return;
}

void createComp(int argc, char** argv,RTC::ModuleInitProc proc, bool block){
	RTC::Manager* manager;
	manager = RTC::Manager::init(argc, argv);
	manager->init(argc, argv);
	manager->setModuleInitProc(proc);
	manager->activateManager();
	manager->runManager(!block);
}

int main(int argc, char** argv) {
	std::cout << "Starting" << std::endl;
	ros::init(argc, argv, "Gateway", ros::init_options::NoSigintHandler);

	createComp(argc,argv,MyModuleInit,false);
	createComp(argc,argv,MyModuleInit2,false);
	createComp(argc,argv,MyModuleInit3,true);

	return 0;
}
*/
int main(int argc, char** argv) {
	return 0;
}
