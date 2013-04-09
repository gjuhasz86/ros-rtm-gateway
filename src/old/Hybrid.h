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
#include <cstring>

#include "BasicDataTypeStub.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"

using namespace RTC;

class HybridConfig {

public:
	HybridConfig():
		m_v_outOut("out",dummy)
	{
		//hyb.addOutPort("out", m_v_outOut);
	}
	virtual ~HybridConfig(){} //why virtual?
	void init() {
		addPort<std_msgs::Int32>();
	}
	OutPort<TimedLong> m_v_outOut;

	void blah(boost::function2<bool, const char* , OutPortBase&> addOutPortFn){
		addOutPortFn("out", m_v_outOut);
	}

private:
	void convert(const std_msgs::Int32::ConstPtr& in, TimedLong& out) {
		out.data = in->data;
	}

	ros::NodeHandle n;
	ros::Subscriber sub;
	TimedLong dummy;
	TimedLong m_v_out;

	//bool addOutPort(const char* name, OutPortBase& outport);
	//template<class RosType, class RtcType>
	//void addPort(RtcType* (*convert)(RosType&)) {
	template<class RosType>
	void addPort() {
		sub = n.subscribe<RosType>("chatter", 1000, boost::bind(&HybridConfig::callback<RosType>,this,_1));
	}

	template<class RosType>
	void callback(const boost::shared_ptr<RosType const>& msg){
		ROS_INFO("I heard: [%d]", msg->data);
		//convert(msg,m_v_out);
		m_v_out.data= msg->data;
		m_v_outOut.write(m_v_out);
	}

};


class Hybrid: public RTC::DataFlowComponentBase {
public:
	Hybrid(RTC::Manager* manager);

	~Hybrid();

	virtual RTC::ReturnCode_t onInitialize();
	virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
	virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

	virtual void chatterCallback(const std_msgs::Int32::ConstPtr& msg);

	//virtual void chatterCallbackExtra(int n, const std_msgs::Int32::ConstPtr& msg);

	virtual void (Hybrid::*getCallback())(const boost::shared_ptr<const std_msgs::Int32_<std::allocator<void> > >&);

	virtual void callback(const boost::shared_ptr<std_msgs::Int32 const>& msg);

template	<class T, class M>
	void (T::*getGenericCallback())(const boost::shared_ptr<M const>&) {
		void(T::*fp)(const boost::shared_ptr<M const>&) = &Hybrid::callback;
		return fp;
	}

	int (*functionFactory(int n))(int, int) {
		printf("Got parameter %d", n);
	}

	//static variable should hold the module name

protected:
	TimedLong m_v_out;
	OutPort<TimedLong> m_v_outOut;
	//ros::NodeHandle n;
	//ros::Subscriber sub;
	HybridConfig hybConf;

private:

};

//template<class RosType, class RtcType>
//RtcType convert(RosType);



extern "C" {
	DLL_EXPORT void HybridInit(RTC::Manager* manager);
}
;

#endif // HYBRID_H
