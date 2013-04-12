#include "Gateway.h"


////////////////////////////////////////////////////////////////////////////////
// Constructor and Destructor

Gateway::Gateway(RTC::Manager* manager) :
		RTC::DataFlowComponentBase(manager) {
}

Gateway::Gateway(RTC::Manager* manager, GatewayFactory::Config<Gateway>* config) :
		RTC::DataFlowComponentBase(manager) {
}

Gateway::~Gateway() {
}

////////////////////////////////////////////////////////////////////////////////
// Public methods - methods comes with the RT component

RTC::ReturnCode_t Gateway::onInitialize() {
	init();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onActivated(RTC::UniqueId ec_id) {
	doSubscibe();
	doAdvertise();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onDeactivated(RTC::UniqueId ec_id) {
	doUnsubscribe();
	doStopAdvertise();
	return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onExecute(RTC::UniqueId ec_id) {
	ros::spinOnce();
	onExec();
	return RTC::RTC_OK;
}

////////////////////////////////////////////////////////////////////////////////
// Protected methods - methods specific to the gateway component

void Gateway::init() {
	//RosToRtmConverter<std_msgs::Int32, TimedLong> c1(&convert1, &callback);

	//createNewRosToRtmLink<std_msgs::Int32, TimedLong>("chatterInt1", c1);
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
void Gateway::createNewRosToRtmLink(const std::string& name, RosToRtmConverter<RosType, RtmType>& converter) {

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
void Gateway::createNewRtmToRosLink(const std::string& name,
		boost::function2<void, const RtmType&, RosType&> convertFn) {

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
void Gateway::subscribeToRosTopic(RosToRtmLink<RtmType>* link, RosToRtmConverter<RosType, RtmType>& converter) {
	link->rosSubscriber = n.subscribe<RosType>(link->name, 1000,
			boost::bind(&Gateway::copyFromRosToRtc<RosType, RtmType>, this, _1, link, converter));
	rosSubscriberList.push_back(&link->rosSubscriber);
}

/*!
 * This member function advertises a ROS topic and links the given RTC in port to the topic.
 */
template<class RosType, class RtmType>
void Gateway::advertiseRosTopic(RtmToRosLink<RtmType>* link) {
	link->rosPublisher = n.advertise<RosType>(link->name, 1000);
	rosPublisherList.push_back(&link->rosPublisher);
}

/*!
 * This member function is a callback function for the ROS node. It is called whenever a message arrives
 * on the ROS topic.
 */
template<class RosType, class RtmType>
void Gateway::copyFromRosToRtc(const boost::shared_ptr<RosType const>& msgIn, RosToRtmLink<RtmType>* link,
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
void Gateway::copyFromRtcToRos(RtmToRosLink<RtmType>* link,
		boost::function2<void, const RtmType&, RosType&> convertFn) {

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
bool Gateway::readFromRtcPort(InPort<RtmType>* inPort) {
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
void Gateway::writeToRtcPort(OutPort<RtmType>* outPort) {
	outPort->write();
}


void Gateway::doAdvertise() {
	for (int i = 0; i < rosAdvertiserFnList.size(); ++i) {
		rosAdvertiserFnList[i]();
	}
}

void Gateway::doStopAdvertise() {
	for (int i = 0; i < rosPublisherList.size(); ++i) {
		rosPublisherList[i]->shutdown();
	}
}

void Gateway::doSubscibe() {
	for (int i = 0; i < rosSubscriberFnList.size(); ++i) {
		rosSubscriberFnList[i]();
	}
}

void Gateway::doUnsubscribe() {
	for (int i = 0; i < rosSubscriberList.size(); ++i) {
		rosSubscriberList[i]->shutdown();
	}
}

void Gateway::onExec() {
	for (int i = 0; i < copyFromRtcToRosFnList.size(); ++i) {
		copyFromRtcToRosFnList[i]();
	}
}

template <class _New>
RTObject_impl* Create(Manager* manager)
{
  return new _New(manager);
}

template <class _Delete>
void Delete(RTObject_impl* rtc)
{
  delete rtc;
}


/*
extern "C" {

void HybridInit(RTC::Manager* manager) {
	coil::Properties profile(hybrid_spec);
	manager->registerFactory(profile, RTC::Create<Gateway>, RTC::Delete<Gateway>);
}

}
;

*/
