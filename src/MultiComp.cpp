#include <rtm/Manager.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include "Hybrid.h"
#include "CoutLong.h"
#include "CoutString.h"

void MyModuleInit(RTC::Manager* manager) {
	std::cout << "Starting Hybrid" << std::endl;
	HybridInit(manager);
	RTC::RtcBase* comp;

	comp = manager->createComponent("Hybrid");

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
	ros::init(argc, argv, "Hybrid", ros::init_options::NoSigintHandler);

	createComp(argc,argv,MyModuleInit,false);
	createComp(argc,argv,MyModuleInit2,false);
	createComp(argc,argv,MyModuleInit3,true);

	return 0;
}
