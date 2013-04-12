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


int main(int argc, char** argv) {
	std::cout << "Starting" << std::endl;
	ros::init(argc, argv, "Gateway", ros::init_options::NoSigintHandler);

	RTC::Manager* manager;
	manager = RTC::Manager::init(argc, argv);
	manager->init(argc, argv);
	manager->setModuleInitProc(MyModuleInit);
	manager->activateManager();
	manager->runManager();

	return 0;
}
