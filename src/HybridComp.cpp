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
#include "Hybrid.h"

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


int main(int argc, char** argv) {
	std::cout << "Starting" << std::endl;
	ros::init(argc, argv, "Hybrid", ros::init_options::NoSigintHandler);

	RTC::Manager* manager;
	manager = RTC::Manager::init(argc, argv);
	manager->init(argc, argv);
	manager->setModuleInitProc(MyModuleInit);
	manager->activateManager();
	manager->runManager();

	return 0;
}
