#include "Hybrid.h"

static const char* hybrid_spec[] = { //
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

Hybrid::Hybrid(RTC::Manager* manager) :
		RTC::DataFlowComponentBase(manager), m_v_outOut("out", m_v_out) {
}

Hybrid::~Hybrid() {
}

void convert(const std_msgs::Int32::ConstPtr& in, TimedLong& out) {
	out.data = in->data;
}

void Hybrid::chatterCallback(const std_msgs::Int32::ConstPtr& msg) {
	ROS_INFO("I heard: [%d]", msg->data);
	convert(msg, m_v_out);
	m_v_outOut.write();
}

void chatterCallback2(const std_msgs::Int32::ConstPtr& msg) {
	ROS_INFO("I heard: [%d]", msg->data);
}

RTC::ReturnCode_t Hybrid::onInitialize() {
	boost::function2<bool, const char*, OutPortBase&> addOutPortFn =
			boost::bind(&Hybrid::addOutPort, this, _1, _2);
	hybConf.blah(addOutPortFn);
	return RTC::RTC_OK;
}

void (Hybrid::* Hybrid::getCallback())(const boost::shared_ptr<const std_msgs::Int32_<std::allocator<void> > >&) {
			void (Hybrid::*fp)(
					const boost::shared_ptr<
							const std_msgs::Int32_<std::allocator<void> > >&) = &Hybrid::chatterCallback;
			return fp;
		}

		void Hybrid::callback(
				const boost::shared_ptr<std_msgs::Int32 const>& msg) {
			ROS_INFO("I heard: [%d]", msg->data);
		}

		RTC::ReturnCode_t Hybrid::onActivated(RTC::UniqueId ec_id) {

			//sub = n.subscribe("chatter", 1000, this->getGenericCallback<Hybrid,	std_msgs::Int32_<std::allocator<void> > >(), this);
			//sub = n.subscribe("chatter", 1000, this->getGenericCallback<Hybrid,	std_msgs::Int32>(), this);
			//sub = n.subscribe("chatter", 1000, &Hybrid::callback, this);
			hybConf.init();
			return RTC::RTC_OK;
		}

		RTC::ReturnCode_t Hybrid::onDeactivated(RTC::UniqueId ec_id) {
			//sub.shutdown();
			return RTC::RTC_OK;
		}

		RTC::ReturnCode_t Hybrid::onExecute(RTC::UniqueId ec_id) {
			ros::spinOnce();
			return RTC::RTC_OK;
		}

		extern "C" {

		void HybridInit(RTC::Manager* manager) {
			coil::Properties profile(hybrid_spec);
			manager->registerFactory(profile, RTC::Create<Hybrid>,
					RTC::Delete<Hybrid>);
		}

		}
		;

