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

#include <rtm/idl/ExtendedDataTypesSkel.h>

#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>

#include <cstring>
#include <vector>

#include "BasicDataTypeStub.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Imu.h"



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

void convert1(const boost::shared_ptr<sensor_msgs::Imu const>& in, Velocity3D& out) {
	double x = in->orientation.x;
	double y = in->orientation.y;
	double z = in->orientation.z;
	double w = in->orientation.w;

	out.vx = x;
	out.vy = y;
	out.vz = z;
	out.vr = w;
}

/*
void callback1(const boost::shared_ptr<sensor_msgs::Imu const>& in, Velocity3D& out,
		const RosToRtmLink<Velocity3D>& link) {
	std::cout << "[";
	std::cout << link.name.c_str();
	std::cout << "] : [";
	std::cout << boost::lexical_cast<std::string>(in->data).c_str();
	std::cout << "]->[";
	std::cout << boost::lexical_cast<std::string>(out.data.x).c_str() << ", ";
	std::cout << boost::lexical_cast<std::string>(out.data.y).c_str() << ", ";
	std::cout << boost::lexical_cast<std::string>(out.data.z).c_str() << "]";
	std::cout << std::endl;
}
*/

void callback2(const boost::shared_ptr<sensor_msgs::Imu const>& in, Velocity3D& out,
		const GatewayFactory::RosToRtmLink<Velocity3D>& link) {

	std::cout << in->orientation.x << std::endl;
	std::cout << in->orientation.y << std::endl;
	std::cout << in->orientation.z << std::endl;
	std::cout << in->orientation.w << std::endl;
	std::cout << "----------------" << std::endl;
}



int main(int argc, char** argv) {
	std::cout << "Starting Gw" << std::endl;
	ros::init(argc, argv, "Gateway", ros::init_options::NoSigintHandler);

	GatewayFactory::Config config(gateway_spec);

	GatewayFactory::RosToRtmHandler<sensor_msgs::Imu, Velocity3D> handler1(&convert1, &callback2);
	config.addNewRosToRtmLink<sensor_msgs::Imu, Velocity3D>("/android/imu", handler1);

	GatewayFactory::createNewGateway<Gateway>(argc, argv, "Hybrid", config, true);

	return 0;
}
