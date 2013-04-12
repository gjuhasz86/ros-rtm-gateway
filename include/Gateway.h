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

#include "GatewayHelper.h"

using namespace RTC;

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
	Gateway(RTC::Manager* manager, GatewayFactory::Config<Gateway>* config);

	~Gateway();

	virtual RTC::ReturnCode_t onInitialize();
	virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
	void init();

	/*!
	 * Links a ROS topic to an RTC out port.
	 *
	 * Note: RtmType has to have a parameterless constructor.
	 * TODO: should the name be reference?
	 */
	template<class RosType, class RtmType>
	void createNewRosToRtmLink(const std::string& name, RosToRtmConverter<RosType, RtmType>& converter);

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
	bool readFromRtcPort(InPort<RtmType>* inPort);

	/*!
	 * Sends the data to the given port from the buffer associated with it.
	 */
	template<class RtmType>
	void writeToRtcPort(OutPort<RtmType>* outPort);

	void doAdvertise();
	void doStopAdvertise();
	void doSubscibe();
	void doUnsubscribe();
	void onExec();
};


/*
extern "C" {
	DLL_EXPORT void HybridInit(RTC::Manager* manager);
}
;
*/
#endif // GATEWAY_H
