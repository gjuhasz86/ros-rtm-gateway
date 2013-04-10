#ifndef HYBRID_H
#define HYBRID_H

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

using namespace RTC;

void convert1(const boost::shared_ptr<std_msgs::Int32 const>& in, TimedLong& out) {
	out.data = in->data;
}

void convert2(const boost::shared_ptr<std_msgs::String const>& in, TimedString& out) {
	out.data = in->data.c_str();
}

void convert3(const TimedLong& in, std_msgs::Int32& out) {
	out.data = in.data;
}

template<class T>
std::string toString(const boost::shared_ptr<T const>& in) {
	return "";
}

template<>
std::string toString<std_msgs::String>(const boost::shared_ptr<std_msgs::String const>& in) {
	return in->data;
}

template<>
std::string toString<std_msgs::Int32>(const boost::shared_ptr<std_msgs::Int32 const>& in) {
	return boost::lexical_cast<std::string>(in->data);
}

std::string toString(TimedLong& in) {
	return boost::lexical_cast<std::string>(in.data);
}

class HybridConfig {

public:
	HybridConfig() {
	}
	virtual ~HybridConfig() {

	} //why virtual?

	void init() {
		//createNewRosToRtmLink<std_msgs::Int32, TimedLong>("chatterInt1", &convert1);
		//createNewRosToRtmLink<std_msgs::String, TimedString>("chatterString", &convert2);
		//boost::function2<void, TimedLong&, std_msgs::Int32&> convertFn = &convert3;
		createNewRtmToRosLink<TimedLong, std_msgs::Int32>("inPort", &convert3);
	}

private:

	/*!
	 * A function to be called to add out port to the RTC.
	 */
	boost::function2<bool, const char*, OutPortBase&> registerRtcOutPortFn;
	boost::function2<bool, const char*, InPortBase&> registerRtcInPortFn;

	/*!
	 * A vector containing references to all of the RTC out ports.
	 * TODO: should be objects (not references)?
	 */
	std::vector<boost::any> rtcOutPortList;
	std::vector<boost::any> rtcInPortList;

	/*!
	 * A vector containing references to all of the RTC out port buffers.
	 * TODO: should be objects (not references)?
	 */
	std::vector<boost::any> rtcOutPortBufferList;
	std::vector<boost::any> rtcInPortBufferList;

	/*!
	 * A vector containing 'HybridConfig::subscribeToRosTopic' function objects.
	 * TODO: leak?
	 */
	std::vector<boost::function0<void> > rosSubscriberFnList;
	std::vector<boost::function0<void> > rosAdvertiserFnList;

	std::vector<boost::function0<void> > copyFromRtcToRosFnList;

	/*!
	 * A vector containing Subscriber objects (not references). We have to hold onto them as
	 * letting the Subscriber go out of scope would cause the topic unsubscribed.
	 */
	std::vector<ros::Subscriber> rosSubscriberList;
	std::vector<ros::Publisher> rosPublisherList;

	ros::NodeHandle n;

	/*!
	 * Note: RtmType has to have a parameterless constructor.
	 */
	template<class RosType, class RtmType>
	void createNewRosToRtmLink(const std::string& name,
			boost::function2<void, const boost::shared_ptr<RosType const>&, RtmType&> convertFn) {

		RtmType* rtcOutPortBuffer = new RtmType();
		OutPort<RtmType>* rtcOutPort = new OutPort<RtmType>(name.c_str(), *rtcOutPortBuffer);
		registerRtcOutPortFn(name.c_str(), *rtcOutPort);
		rtcOutPortList.push_back(rtcOutPort);
		rtcOutPortBufferList.push_back(rtcOutPortBuffer);

		boost::function0<void> rosSubscriberFn;
		rosSubscriberFn = boost::bind(&HybridConfig::subscribeToRosTopic<RosType, RtmType>, this, name, rtcOutPort,
				rtcOutPortBuffer, convertFn);
		rosSubscriberFnList.push_back(rosSubscriberFn);
	}

	/*!
	 * Note: RtmType has to have a parameterless constructor.
	 */
	template<class RtmType, class RosType>
	void createNewRtmToRosLink(const std::string& name, boost::function2<void, const RtmType&, RosType&> convertFn) {

		RtmType* rtcInPortBuffer = new RtmType();
		InPort<RtmType>* rtcInPort = new InPort<RtmType>(name.c_str(), *rtcInPortBuffer);
		registerRtcInPortFn(name.c_str(), *rtcInPort);
		rtcInPortList.push_back(rtcInPort);
		rtcInPortBufferList.push_back(rtcInPortBuffer);

		boost::function4<void, HybridConfig*, InPort<RtmType>*, RtmType*,
				boost::function2<void, const RtmType&, RosType&> > fn =
				&HybridConfig::copyFromRtcToRos<RtmType, RosType>;
		boost::function0<void> copyFromRtcToRosFn = boost::bind(fn, this, rtcInPort, rtcInPortBuffer, convertFn);
		copyFromRtcToRosFnList.push_back(copyFromRtcToRosFn);

		boost::function0<void> rosAdvertiserFn;
		rosAdvertiserFn = boost::bind(&HybridConfig::advertiseRosTopic<RosType>, this, name);
		rosAdvertiserFnList.push_back(rosAdvertiserFn);
	}

	/*!
	 * This member function subscribes to a ROS topic, and links its input to the given RTC out port.
	 * It also requires a converter function which converts the input data from ROS to the output type of RTC.
	 */
	template<class RosType, class RtmType>
	void subscribeToRosTopic(const std::string& name, OutPort<RtmType>* outPort, RtmType* outPortBuffer,
			boost::function2<void, const boost::shared_ptr<RosType const>&, RtmType&> convertFn) {
		ros::Subscriber sub = n.subscribe<RosType>(name, 1000,
				boost::bind(&HybridConfig::onRosInput<RosType, RtmType>, this, _1, outPort, outPortBuffer, convertFn));
		rosSubscriberList.push_back(sub);
	}

	template<class RosType>
	void advertiseRosTopic(const std::string& name) {
		ros::Publisher pub = n.advertise<RosType>(name, 1000);
		rosPublisherList.push_back(pub);
	}

	/*!
	 * This member function is a callback function for the ROS node. It is called whenever a message arrives
	 * on the ROS topic.
	 */
	template<class RosType, class RtmType>
	void onRosInput(const boost::shared_ptr<RosType const>& msgIn, OutPort<RtmType>* outPort, RtmType* outPortBuffer,
			boost::function2<void, const boost::shared_ptr<RosType const>&, RtmType&> convertFn) {
		std::string s = toString<RosType>(msgIn);
		std::cout << "ROS: " << s.c_str() << std::endl;
		convertFn(msgIn, *outPortBuffer);
		writeToRtcPort(outPort);
	}

	template<class RtmType, class RosType>
	void copyFromRtcToRos(InPort<RtmType>* inPort, RtmType* inPortBuffer,
			boost::function2<void, const RtmType&, RosType&> convertFn) {

		bool hasResult = readFromRtcPort(inPort);
		if (hasResult) {
			std::cout << toString(*inPortBuffer) << std::endl;
			RosType msg;
			convertFn(*inPortBuffer, msg);
			rosPublisherList[0].publish(msg); //TODO publish to multiple
		}

	}

	template<class RtmType>
	bool readFromRtcPort(InPort<RtmType>* inPort) {
		if (inPort->isNew()) {
			inPort->read();
			return true;
		} else {
			return false;
		}
	}

	template<class RtmType>
	void writeToRtcPort(OutPort<RtmType>* outPort) {
		outPort->write();
	}

public:

	void setRegisterRtcOutPortFn(boost::function2<bool, const char*, OutPortBase&> fn) {
		registerRtcOutPortFn = fn;
	}

	void setRegisterRtcInPortFn(boost::function2<bool, const char*, InPortBase&> fn) {
		registerRtcInPortFn = fn;
	}

	void doAdvertise() {
		for (int i = 0; i < rosAdvertiserFnList.size(); ++i) {
			rosAdvertiserFnList[i]();
		}
	}

	void doSubscibe() {
		for (int i = 0; i < rosSubscriberFnList.size(); ++i) {
			rosSubscriberFnList[i]();
		}
	}

	void doUnsubscribe() {
		for (int i = 0; i < rosSubscriberList.size(); ++i) {
			rosSubscriberList[i].shutdown();
		}
	}

	void onExec() {
		for (int i = 0; i < copyFromRtcToRosFnList.size(); ++i) {
			copyFromRtcToRosFnList[i]();
		}
	}

};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

class Hybrid: public RTC::DataFlowComponentBase {
public:
	Hybrid(RTC::Manager* manager);

	~Hybrid();

	virtual RTC::ReturnCode_t onInitialize();
	virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
	HybridConfig hybConf;

};

extern "C" {
DLL_EXPORT void HybridInit(RTC::Manager* manager);
}
;

#endif // HYBRID_H
