#include "GatewayHelper.h"

#include "std_msgs/Int32.h"
#include "BasicDataTypeStub.h"

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
		RosToRtmConverter<RosType, RtmType>& converter) {

	RosToRtmLink<RtmType>* link = new RosToRtmLink<RtmType>(name);
	registerRtcOutPortFn(name.c_str(), link->rtcOutPort);

	boost::function0<void> rosSubscriberFn;
	rosSubscriberFn = boost::bind(&subscribeToRosTopic<RosType, RtmType>, this, link, converter);
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
	registerRtcInPortFn(name.c_str(), link->rtcInPort);

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
			boost::bind(&copyFromRosToRtc<RosType, RtmType>, this, _1, link, converter));
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
