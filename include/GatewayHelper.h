#ifndef GATEWAYHELPER_H_
#define GATEWAYHELPER_H_

#include <vector>

#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "ros/ros.h"

//TODO remove these headers
#include "std_msgs/Int32.h"
#include "BasicDataTypeStub.h"
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>

#include <cstring>
#include <vector>
#include <string>
#include <iostream>

#include "BasicDataTypeStub.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include "GatewayHelper.h"

template<class RtmType>
struct RtmToRosLink {
	RtmToRosLink(const std::string name) :
			name(name), rtcInPort(name.c_str(), rtcInPortBuffer) {
	}
	std::string name;
	RTC::InPort<RtmType> rtcInPort;
	RtmType rtcInPortBuffer;
	ros::Publisher rosPublisher;
};

template<class RtmType>
struct RosToRtmLink {
	RosToRtmLink(const std::string name) :
			name(name), rtcOutPort(name.c_str(), rtcOutPortBuffer) {
	}
	std::string name;
	RTC::OutPort<RtmType> rtcOutPort;
	RtmType rtcOutPortBuffer;
	ros::Subscriber rosSubscriber;
};

template<class RosType, class RtmType>
struct RosToRtmConverter {
public:
	RosToRtmConverter(boost::function2<void, const boost::shared_ptr<RosType const>&, RtmType&> convert,
			boost::function3<void, const boost::shared_ptr<RosType const>&, RtmType&, RosToRtmLink<RtmType>&> const callback) :
			convert(convert), callback(callback), hasCallback(true) {
	}

	RosToRtmConverter(boost::function2<void, const boost::shared_ptr<RosType const>&, RtmType&> convert) :
			convert(convert), hasCallback(false) {
	}

	boost::function2<void, const boost::shared_ptr<RosType const>&, RtmType&> convert;

	bool const hasCallback;
	boost::function3<void, const boost::shared_ptr<RosType const>&, RtmType&, RosToRtmLink<RtmType>&> callback;
};

namespace GatewayFactory {

	template<class Component>
	class Config {
	public:
		Config(const char** gateway_spec) :
				hybrid_spec(gateway_spec) {
		}

		const char** hybrid_spec;

	public:
		/*!
		 * A function to be called to add out port to the RTC.
		 */
		std::vector<boost::function1<void, Component*> > registerRtcOutPortFnList;

		/*!
		 * A function to be called to add in port to the RTC.
		 */
		std::vector<boost::function1<void, Component*> > registerRtcInPortFnList;

		std::vector<boost::function0<void> > rosSubscriberFnList;
		std::vector<boost::function0<void> > rosAdvertiserFnList;

		std::vector<boost::function0<void> > copyFromRtcToRosFnList;

		/*!
		 * A vector containing Subscriber/Publisher objects (not references). We have to hold onto them as
		 * letting the Subscriber go out of scope would cause the node unsubscribe from the topic.
		 */
		std::vector<ros::Subscriber*> rosSubscriberList;
		std::vector<ros::Publisher*> rosPublisherList;

		ros::NodeHandle n;

	public:
		void init();

		/*!
		 * Links a ROS topic to an RTC out port.
		 *
		 * Note: RtmType has to have a parameterless constructor.
		 * TODO: should the name be reference?
		 */
		template<class RosType, class RtmType>
		void createNewRosToRtmLink(const std::string& name, RosToRtmConverter<RosType, RtmType>& converter,
				const boost::function2<bool, const char*, RTC::OutPortBase&> fn);

		/*!
		 * Links an RTC in port to a ROS topic.
		 *
		 * Note: RtmType has to have a parameterless constructor.
		 */
		template<class RtmType, class RosType>
		void createNewRtmToRosLink(const std::string& name, boost::function2<void, const RtmType&, RosType&> convertFn);

		/*!
		 * This member function subscribes to a ROS topic, and links its input to the given RTC out port.
		 * It also requires a converter function which converts the input data from ROS to the output type of RTC.
		 */
		template<class RosType, class RtmType>
		void subscribeToRosTopic(RosToRtmLink<RtmType>* link, RosToRtmConverter<RosType, RtmType>& converter);

		/*!
		 * This member function advertises a ROS topic and links the given RTC in port to the topic.
		 */
		template<class RosType, class RtmType>
		void advertiseRosTopic(RtmToRosLink<RtmType>* link);

		/*!
		 * This member function is a callback function for the ROS node. It is called whenever a message arrives
		 * on the ROS topic.
		 */
		template<class RosType, class RtmType>
		void copyFromRosToRtc(const boost::shared_ptr<RosType const>& msgIn, RosToRtmLink<RtmType>* link,
				RosToRtmConverter<RosType, RtmType> converter);

		/*!
		 * This member function is a callback function for the RTC port. It is called periodically, and passes the
		 * Link object containing an RTC port, and a ROS publisher.
		 */
		template<class RtmType, class RosType>
		void copyFromRtcToRos(RtmToRosLink<RtmType>* link, boost::function2<void, const RtmType&, RosType&> convertFn);

		/*!
		 * Checks whether new data has arrived to the given RTC port. If it has, it loads the data into the buffer
		 * associated with the port.
		 */
		template<class RtmType>
		bool readFromRtcPort(RTC::InPort<RtmType>* inPort);

		/*!
		 * Sends the data to the given port from the buffer associated with it.
		 */
		template<class RtmType>
		void writeToRtcPort(RTC::OutPort<RtmType>* outPort);

	public:
		/*
		 void setRegisterRtcOutPortFn(boost::function2<bool, const char*, RTC::OutPortBase&> fn) {
		 registerRtcOutPortFn = fn;
		 }

		 void setRegisterRtcInPortFn(boost::function2<bool, const char*, RTC::InPortBase&> fn) {
		 registerRtcInPortFn = fn;
		 }
		 */

		void createRtcOutPorts(Component* comp);
		void createRtcInPorts(Component* comp);

		void doAdvertise();
		void doStopAdvertise();

		void doSubscibe();
		void doUnsubscribe();

		void onExec();
	};

	template<class _New>
	RTC::RTObject_impl* Create(RTC::Manager* manager, Config<_New>* config) {
		return new _New(manager, config);
	}

	template<class Component>
	void GatewayInit(RTC::Manager* manager, Config<Component>* config) {

		coil::Properties profile(config->hybrid_spec);

		boost::function1<RTC::RTObject_impl*, RTC::Manager*> bNewFn = boost::bind(&Create<Component>, _1, config);
		RTC::RtcNewFunc* newFn = bNewFn.target<RTC::RtcNewFunc>();

		manager->registerFactory(profile, **newFn, RTC::Delete<Component>);
	}

	template<class Component>
	void ModuleInit(RTC::Manager* manager, Config<Component>* config) {
		std::cout << "Starting Gateway" << std::endl;
		GatewayInit<Component>(manager, config);

		RTC::RtcBase* comp;
		comp = manager->createComponent("Gateway");

		if (comp == NULL) {
			std::cerr << "Component create failed." << std::endl;
			abort();
		}

		return;
	}

	template<class Component>
	void createNewGateway(int argc, char** argv, Config<Component>* config, bool block = true) {
		RTC::Manager* manager;
		manager = RTC::Manager::init(argc, argv);
		manager->init(argc, argv);

		boost::function1<void, RTC::Manager*> mFn = boost::bind(&ModuleInit<Component>, _1, config);
		RTC::ModuleInitProc* moduleInitProc = mFn.target<RTC::ModuleInitProc>();
		manager->setModuleInitProc(**moduleInitProc);
		manager->activateManager();
		manager->runManager(!block);
	}

}

///////////////////////////////////////////////////////////////////
// IMLEMENTATION
///////////////////////////////////////////////////////////////////

template<class Component>
void GatewayFactory::Config<Component>::init() {
	//RosToRtmConverter<std_msgs::Int32, RTC::TimedLong> c1(&convert1, &callback);

	//createNewRosToRtmLink<std_msgs::Int32, RTC::TimedLong>("chatterInt1", c1);
	//		createNewRosToRtmLink<std_msgs::String, TimedString>("chatterString", &convert2);
	//		createNewRtmToRosLink<TimedLong, std_msgs::Int32>("inPort1", &convert3);
	//createNewRtmToRosLink<TimedLong, std_msgs::Int32>("inPort2", &convert3);
}

/*!
 * Links a ROS topic to an RTC out port.
 *
 * Note: RtmType has to have a parameterless constructor.
 * TODO: should the name be reference?
 */
template<class Component>
template<class RosType, class RtmType>
void GatewayFactory::Config<Component>::createNewRosToRtmLink(const std::string& name,
		RosToRtmConverter<RosType, RtmType>& converter, const boost::function2<bool, const char*, RTC::OutPortBase&> fn)
{

	RosToRtmLink<RtmType>* link = new RosToRtmLink<RtmType>(name);

	boost::bind(fn, name.c_str(), link->rtcOutPort);
	//registerRtcOutPortFnList.push_back(registerRtcOutPortFn);
	//registerRtcOutPortFn(name.c_str(), link->rtcOutPort);

	//registerRtcOutPortFn();
	fn(name.c_str(), link->rtcOutPort);

	boost::function0<void> rosSubscriberFn;
	rosSubscriberFn = boost::bind(&Config::subscribeToRosTopic<RosType, RtmType>, this, link, converter);
	rosSubscriberFnList.push_back(rosSubscriberFn);
}

/*!
 * Links an RTC in port to a ROS topic.
 *
 * Note: RtmType has to have a parameterless constructor.
 */
template<class Component>
template<class RtmType, class RosType>
void GatewayFactory::Config<Component>::createNewRtmToRosLink(const std::string& name,
		boost::function2<void, const RtmType&, RosType&> convertFn) {

	RtmToRosLink<RtmType>* link = new RtmToRosLink<RtmType>(name);

	boost::function1<void, Component*> registerRtcInPortFn = boost::bind(&Component::addInPort, _1, name.c_str(),
			link->rtcInPort);
	registerRtcInPortFnList.push_back(registerRtcInPortFn);
	//registerRtcInPortFn(name.c_str(), link->rtcInPort);

	boost::function0<void> copyFromRtcToRosFn = boost::bind(&copyFromRtcToRos<RtmType, RosType>, this, link, convertFn);
	copyFromRtcToRosFnList.push_back(copyFromRtcToRosFn);

	boost::function0<void> rosAdvertiserFn;
	rosAdvertiserFn = boost::bind(&advertiseRosTopic<RosType, RtmType>, this, link);
	rosAdvertiserFnList.push_back(rosAdvertiserFn);
}

/*!
 * This member function subscribes to a ROS topic, and links its input to the given RTC out port.
 * It also requires a converter function which converts the input data from ROS to the output type of RTC.
 */
template<class Component>
template<class RosType, class RtmType>
void GatewayFactory::Config<Component>::subscribeToRosTopic(RosToRtmLink<RtmType>* link,
		RosToRtmConverter<RosType, RtmType>& converter) {
	link->rosSubscriber = n.subscribe<RosType>(link->name, 1000,
			boost::bind(&Config::copyFromRosToRtc<RosType, RtmType>, this, _1, link, converter));
	rosSubscriberList.push_back(&link->rosSubscriber);
}

/*!
 * This member function advertises a ROS topic and links the given RTC in port to the topic.
 */
template<class Component>
template<class RosType, class RtmType>
void GatewayFactory::Config<Component>::advertiseRosTopic(RtmToRosLink<RtmType>* link) {
	link->rosPublisher = n.advertise<RosType>(link->name, 1000);
	rosPublisherList.push_back(&link->rosPublisher);
}

/*!
 * This member function is a callback function for the ROS node. It is called whenever a message arrives
 * on the ROS topic.
 */
template<class Component>
template<class RosType, class RtmType>
void GatewayFactory::Config<Component>::copyFromRosToRtc(const boost::shared_ptr<RosType const>& msgIn,
		RosToRtmLink<RtmType>* link, RosToRtmConverter<RosType, RtmType> converter) {

	converter.convert(msgIn, link->rtcOutPortBuffer);

	if (converter.hasCallback) {
		converter.callback(msgIn, link->rtcOutPortBuffer, *link);
	}

	writeToRtcPort(&link->rtcOutPort);

}

/*!
 * This member function is a callback function for the RTC port. It is called periodically, and passes the
 * Link object containing an RTC port, and a ROS publisher.
 */
template<class Component>
template<class RtmType, class RosType>
void GatewayFactory::Config<Component>::copyFromRtcToRos(RtmToRosLink<RtmType>* link,
		boost::function2<void, const RtmType&, RosType&> convertFn) {

	RTC::InPort<RtmType>* inPort = &link->rtcInPort;
	bool hasResult = readFromRtcPort<RtmType>(inPort);
	if (hasResult) {
		//std::cout << toString(link->rtcInPortBuffer) << std::endl;
		RosType msg;
		convertFn(link->rtcInPortBuffer, msg);
		link->rosPublisher.publish(msg);
	}

}

/*!
 * Checks whether new data has arrived to the given RTC port. If it has, it loads the data into the buffer
 * associated with the port.
 */
template<class Component>
template<class RtmType>
bool GatewayFactory::Config<Component>::readFromRtcPort(RTC::InPort<RtmType>* inPort) {
	if (inPort->isNew()) {
		inPort->read();
		return true;
	} else {
		return false;
	}
}

/*!
 * Sends the data to the given port from the buffer associated with it.
 */
template<class Component>
template<class RtmType>
void GatewayFactory::Config<Component>::writeToRtcPort(RTC::OutPort<RtmType>* outPort) {
	outPort->write();
}

///////////////////////////////////////////////////////////////////
//

template<class Component>
void GatewayFactory::Config<Component>::createRtcOutPorts(Component* comp) {
	for (int i = 0; i < registerRtcOutPortFnList.size(); ++i) {
		registerRtcOutPortFnList(comp);
	}
}

template<class Component>
void GatewayFactory::Config<Component>::createRtcInPorts(Component* comp) {
	for (int i = 0; i < registerRtcInPortFnList.size(); ++i) {
		registerRtcInPortFnList(comp);
	}
}

template<class Component>
void GatewayFactory::Config<Component>::doAdvertise() {
	for (int i = 0; i < rosAdvertiserFnList.size(); ++i) {
		rosAdvertiserFnList[i]();
	}
}

template<class Component>
void GatewayFactory::Config<Component>::doStopAdvertise() {
	for (int i = 0; i < rosPublisherList.size(); ++i) {
		rosPublisherList[i]->shutdown();
	}
}

template<class Component>
void GatewayFactory::Config<Component>::doSubscibe() {
	for (int i = 0; i < rosSubscriberFnList.size(); ++i) {
		rosSubscriberFnList[i]();
	}
}

template<class Component>
void GatewayFactory::Config<Component>::doUnsubscribe() {
	for (int i = 0; i < rosSubscriberList.size(); ++i) {
		rosSubscriberList[i]->shutdown();
	}
}

template<class Component>
void GatewayFactory::Config<Component>::onExec() {
	for (int i = 0; i < copyFromRtcToRosFnList.size(); ++i) {
		copyFromRtcToRosFnList[i]();
	}
}

#endif /* GATEWAYHELPER_H_ */
