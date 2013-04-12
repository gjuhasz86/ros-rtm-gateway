#ifndef GATEWAY_H
#define GATEWAY_H

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

void callback(const boost::shared_ptr<std_msgs::Int32 const>& in, TimedLong& out, const RosToRtmLink<TimedLong>& link) {
	std::cout << "[";
	std::cout << link.name.c_str();
	std::cout << "] : [";
	std::cout << boost::lexical_cast<std::string>(in->data).c_str();
	std::cout << "]->[";
	std::cout << boost::lexical_cast<std::string>(out.data).c_str() << "]";
	std::cout << std::endl;
}

class Gateway: public RTC::DataFlowComponentBase {
private:

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
	Gateway(RTC::Manager* manager);

	~Gateway();

	virtual RTC::ReturnCode_t onInitialize();
	virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
	void init() {
		RosToRtmConverter<std_msgs::Int32, TimedLong> c1(&convert1, &callback);

		createNewRosToRtmLink<std_msgs::Int32, TimedLong>("chatterInt1", c1);
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
	template<class RosType, class RtmType>
	void createNewRosToRtmLink(const std::string& name, RosToRtmConverter<RosType, RtmType>& converter) {

		RosToRtmLink<RtmType>* link = new RosToRtmLink<RtmType>(name);
		addOutPort(name.c_str(), link->rtcOutPort);

		boost::function0<void> rosSubscriberFn;
		rosSubscriberFn = boost::bind(&Gateway::subscribeToRosTopic<RosType, RtmType>, this, link, converter);
		rosSubscriberFnList.push_back(rosSubscriberFn);
	}

	/*!
	 * Links an RTC in port to a ROS topic.
	 *
	 * Note: RtmType has to have a parameterless constructor.
	 */
	template<class RtmType, class RosType>
	void createNewRtmToRosLink(const std::string& name, boost::function2<void, const RtmType&, RosType&> convertFn) {

		RtmToRosLink<RtmType>* link = new RtmToRosLink<RtmType>(name);
		addInPort(name.c_str(), link->rtcInPort);

		boost::function0<void> copyFromRtcToRosFn = boost::bind(&Gateway::copyFromRtcToRos<RtmType, RosType>, this, link,
				convertFn);
		copyFromRtcToRosFnList.push_back(copyFromRtcToRosFn);

		boost::function0<void> rosAdvertiserFn;
		rosAdvertiserFn = boost::bind(&Gateway::advertiseRosTopic<RosType, RtmType>, this, link);
		rosAdvertiserFnList.push_back(rosAdvertiserFn);
	}

	/*!
	 * This member function subscribes to a ROS topic, and links its input to the given RTC out port.
	 * It also requires a converter function which converts the input data from ROS to the output type of RTC.
	 */
	template<class RosType, class RtmType>
	void subscribeToRosTopic(RosToRtmLink<RtmType>* link, RosToRtmConverter<RosType, RtmType>& converter) {
		link->rosSubscriber = n.subscribe<RosType>(link->name, 1000,
				boost::bind(&Gateway::copyFromRosToRtc<RosType, RtmType>, this, _1, link, converter));
		rosSubscriberList.push_back(&link->rosSubscriber);
	}

	/*!
	 * This member function advertises a ROS topic and links the given RTC in port to the topic.
	 */
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
	void copyFromRosToRtc(const boost::shared_ptr<RosType const>& msgIn, RosToRtmLink<RtmType>* link,
			RosToRtmConverter<RosType, RtmType> converter) {

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
	template<class RtmType, class RosType>
	void copyFromRtcToRos(RtmToRosLink<RtmType>* link, boost::function2<void, const RtmType&, RosType&> convertFn) {

		InPort<RtmType>* inPort = &link->rtcInPort;
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
	template<class RtmType>
	bool readFromRtcPort(InPort<RtmType>* inPort) {
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
	template<class RtmType>
	void writeToRtcPort(OutPort<RtmType>* outPort) {
		outPort->write();
	}

public:

	void doAdvertise() {
		for (int i = 0; i < rosAdvertiserFnList.size(); ++i) {
			rosAdvertiserFnList[i]();
		}
	}

	void doStopAdvertise() {
		for (int i = 0; i < rosPublisherList.size(); ++i) {
			rosPublisherList[i]->shutdown();
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

#endif // GATEWAY_H
