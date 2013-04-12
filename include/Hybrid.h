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
	//std::cout << "convert called" << std::endl;
	out.data = in->data;
}

void convert2(const boost::shared_ptr<std_msgs::String const>& in, TimedString& out) {
	out.data = in->data.c_str();
}

void convert3(const TimedLong& in, std_msgs::Int32& out) {
	out.data = in.data;
}

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

void callback(const boost::shared_ptr<std_msgs::Int32 const>& in, TimedLong& out,
		const RosToRtmLink<TimedLong>& link) {
	std::cout << "[";
	std::cout << link.name.c_str();
	std::cout << "] : [";
	std::cout << boost::lexical_cast<std::string>(in->data).c_str();
	std::cout << "]->[";
	std::cout << boost::lexical_cast<std::string>(out.data).c_str() << "]";
	std::cout << std::endl;
}

class Hybrid: public RTC::DataFlowComponentBase {
public:
	Hybrid(RTC::Manager* manager);

	~Hybrid();

	virtual RTC::ReturnCode_t onInitialize();
	virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

	void init() {
		RosToRtmConverter<std_msgs::Int32, TimedLong> c1(&convert1, &callback);

		createNewRosToRtmLink<std_msgs::Int32, TimedLong>("chatterInt1", c1);
//		createNewRosToRtmLink<std_msgs::String, TimedString>("chatterString", &convert2);
//		createNewRtmToRosLink<TimedLong, std_msgs::Int32>("inPort1", &convert3);
		//createNewRtmToRosLink<TimedLong, std_msgs::Int32>("inPort2", &convert3);
	}
private:

	/*!
	 * A function to be called to add out port to the RTC.
	 */
	boost::function2<bool, const char*, OutPortBase&> registerRtcOutPortFn;

	/*!
	 * A function to be called to add in port to the RTC.
	 */
	boost::function2<bool, const char*, InPortBase&> registerRtcInPortFn;

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

	/*!
	 * Note: RtmType has to have a parameterless constructor.
	 * TODO: should the name be reference?
	 */
	template<class RosType, class RtmType>
	void createNewRosToRtmLink(const std::string& name, RosToRtmConverter<RosType, RtmType>& converter) {

		RosToRtmLink<RtmType>* link = new RosToRtmLink<RtmType>(name);
		registerRtcOutPortFn(name.c_str(), link->rtcOutPort);

		boost::function0<void> rosSubscriberFn;
		rosSubscriberFn = boost::bind(&Hybrid::subscribeToRosTopic<RosType, RtmType>, this, link, converter);
		rosSubscriberFnList.push_back(rosSubscriberFn);
	}

	/*!
	 * Note: RtmType has to have a parameterless constructor.
	 */
	template<class RtmType, class RosType>
	void createNewRtmToRosLink(const std::string& name, boost::function2<void, const RtmType&, RosType&> convertFn) {

		RtmToRosLink<RtmType>* link = new RtmToRosLink<RtmType>(name);
		registerRtcInPortFn(name.c_str(), link->rtcInPort);

		boost::function0<void> copyFromRtcToRosFn = boost::bind(&Hybrid::copyFromRtcToRos<RtmType, RosType>, this,
				link, convertFn);
		copyFromRtcToRosFnList.push_back(copyFromRtcToRosFn);

		boost::function0<void> rosAdvertiserFn;
		rosAdvertiserFn = boost::bind(&Hybrid::advertiseRosTopic<RosType, RtmType>, this, link);
		rosAdvertiserFnList.push_back(rosAdvertiserFn);
	}

	/*!
	 * This member function subscribes to a ROS topic, and links its input to the given RTC out port.
	 * It also requires a converter function which converts the input data from ROS to the output type of RTC.
	 */
	template<class RosType, class RtmType>
	void subscribeToRosTopic(RosToRtmLink<RtmType>* link, RosToRtmConverter<RosType, RtmType>& converter) {
		link->rosSubscriber = n.subscribe<RosType>(link->name, 1000,
				boost::bind(&Hybrid::onRosInput<RosType, RtmType>, this, _1, link, converter));
		rosSubscriberList.push_back(&link->rosSubscriber);
	}

	template<class RosType, class RtmType>
	void advertiseRosTopic(RtmToRosLink<RtmType>* link) {
		link->rosPublisher = n.advertise<RosType>(link->name, 1000);
		rosPublisherList.push_back(&link->rosPublisher);
	}

	/*!
	 * This member function is a callback function for the ROS node. It is called whenever a message arrives
	 * on the ROS topic.
	 */
	template<class RosType, class RtmType>
	void onRosInput(const boost::shared_ptr<RosType const>& msgIn, RosToRtmLink<RtmType>* link,
			RosToRtmConverter<RosType, RtmType> converter) {

		converter.convert(msgIn, link->rtcOutPortBuffer);

		//TODO: callback not working
		//if (converter.hasCallback) {
		converter.callback(msgIn, link->rtcOutPortBuffer, *link);
		//}

		writeToRtcPort(&link->rtcOutPort);

	}

	template<class RtmType, class RosType>
	void copyFromRtcToRos(RtmToRosLink<RtmType>* link, boost::function2<void, const RtmType&, RosType&> convertFn) {

		InPort<RtmType>* inPort = &link->rtcInPort;
		bool hasResult = readFromRtcPort(inPort);
		if (hasResult) {
			//std::cout << toString(link->rtcInPortBuffer) << std::endl;
			RosType msg;
			convertFn(link->rtcInPortBuffer, msg);
			link->rosPublisher.publish(msg);
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
			rosSubscriberList[i]->shutdown();
		}
	}

	void doStopAdvertise() {
		for (int i = 0; i < rosPublisherList.size(); ++i) {
			rosPublisherList[i]->shutdown();
		}
	}

	void onExec() {
		for (int i = 0; i < copyFromRtcToRosFnList.size(); ++i) {
			copyFromRtcToRosFnList[i]();
		}
	}
};

extern "C" {
DLL_EXPORT void HybridInit(RTC::Manager* manager);
}
;

#endif // HYBRID_H
