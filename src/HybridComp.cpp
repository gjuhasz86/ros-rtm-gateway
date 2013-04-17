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

void callback1(const boost::shared_ptr<std_msgs::Int32 const>& in, TimedLong& out,
		const RosToRtmLink<TimedLong>& link) {
	std::cout << "[";
	std::cout << link.name.c_str();
	std::cout << "] : [";
	std::cout << boost::lexical_cast<std::string>(in->data).c_str();
	std::cout << "]->[";
	std::cout << boost::lexical_cast<std::string>(out.data).c_str() << "]";
	std::cout << std::endl;
}

void callback2(TimedLong& in, std_msgs::Int32& out, const RtmToRosLink<TimedLong>& link) {
	std::cout << "[";
	std::cout << link.name.c_str();
	std::cout << "] : [";
	std::cout << boost::lexical_cast<std::string>(in.data).c_str();
	std::cout << "]->[";
	std::cout << boost::lexical_cast<std::string>(out.data).c_str() << "]";
	std::cout << std::endl;
}

int main(int argc, char** argv) {
	std::cout << "Starting" << std::endl;
	ros::init(argc, argv, "Gateway", ros::init_options::NoSigintHandler);

	GatewayFactory::Config config(gateway_spec);

	RosToRtmHandler<std_msgs::Int32, TimedLong> handler1(&convert1, &callback1);
	config.addNewRosToRtmLink<std_msgs::Int32, TimedLong>("chatterInt1", handler1);

	RtmToRosHandler<TimedLong, std_msgs::Int32> handler2(&convert3, &callback2);
	config.addNewRtmToRosLink<TimedLong, std_msgs::Int32>("chatter", handler2);

	GatewayFactory::createNewGateway<Gateway>(argc, argv, config, true);

	return 0;
}
