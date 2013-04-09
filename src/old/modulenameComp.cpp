// -*- C++ -*-
/*!
 * @file modulenameComp.cpp
 * @brief Standalone component
 * @date $Date$
 *
 * $Id$
 */

#include <rtm/Manager.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include "modulename.h"
#include "ros/ros.h"
#include "std_msgs/String.h"


void MyModuleInit(RTC::Manager* manager)
{
  modulenameInit(manager);
  RTC::RtcBase* comp;

  // Create a component
  comp = manager->createComponent("modulename");

  if (comp==NULL)
  {
    std::cerr << "Component create failed." << std::endl;
    abort();
  }

  return;
}

int main (int argc, char** argv)
{
  RTC::Manager* manager;
  manager = RTC::Manager::init(argc, argv);

  // Initialize manager
  manager->init(argc, argv);
  ros::init(argc, argv, "listener",ros::init_options::NoSigintHandler);

  // Set module initialization proceduer
  // This procedure will be invoked in activateManager() function.
  manager->setModuleInitProc(MyModuleInit);

  // Activate manager and register to naming service
  manager->activateManager();

  // run the manager in blocking mode
  // runManager(false) is the default.
  manager->runManager();

  // If you want to run the manager in non-blocking mode, do like this
  // manager->runManager(true);

  return 0;
}
