#ifndef GATEWAYHELPER_H_
#define GATEWAYHELPER_H_

#include <vector>

//#include <rtm/Manager.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>

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
		Config(const char** hybrid_spec) :
				hybrid_spec(hybrid_spec) {
		}

		const char** hybrid_spec;

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
		bool readFromRtcPort(RTC::InPort<RtmType>* inPort);

		/*!
		 * Sends the data to the given port from the buffer associated with it.
		 */
		template<class RtmType>
		void writeToRtcPort(RTC::OutPort<RtmType>* outPort);

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

#endif /* GATEWAYHELPER_H_ */
