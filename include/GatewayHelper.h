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

using namespace boost;

namespace GatewayFactory {

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
struct RosToRtmHandler {
public:
  RosToRtmHandler(
      function2<void, const shared_ptr<RosType const>&, RtmType&> convert,
      function3<void, const shared_ptr<RosType const>&, RtmType&,
          RosToRtmLink<RtmType>&> const callback) :
      convert(convert), callback(callback), hasCallback(true) {
  }

  RosToRtmHandler(
      function2<void, const shared_ptr<RosType const>&, RtmType&> convert) :
      convert(convert), hasCallback(false) {
  }

  function2<void, const shared_ptr<RosType const>&, RtmType&> convert;

  bool const hasCallback;
  function3<void, const shared_ptr<RosType const>&, RtmType&,
      RosToRtmLink<RtmType>&> callback;
};

template<class RtmType, class RosType>
struct RtmToRosHandler {
public:
  RtmToRosHandler(function2<void, RtmType&, RosType&> convert,
      function3<void, RtmType&, RosType&, RtmToRosLink<RtmType>&> const callback) :
      convert(convert), callback(callback), hasCallback(true) {
  }

  RtmToRosHandler(function2<void, RtmType&, RosType&> convert) :
      convert(convert), hasCallback(false) {
  }

  function2<void, RtmType&, RosType&> convert;

  bool const hasCallback;
  function3<void, RtmType&, RosType&, RtmToRosLink<RtmType>&> callback;
};

class Config {
public:
  Config() :
      comp_spec(0) {
  }

  Config(const char** spec) :
      comp_spec(spec) {
  }

  const char** comp_spec;

private:
  typedef function2<bool, const char*, RTC::OutPortBase&> RegisterRtcOutPortFn;
  typedef function1<void, RegisterRtcOutPortFn> RegisterRtcOutPortFnWrapper;
  typedef function2<bool, const char*, RTC::InPortBase&> RegisterRtcInPortFn;
  typedef function1<void, RegisterRtcInPortFn> RegisterRtcInPortFnWrapper;

  typedef function0<void> RosSubscriberFn;
  typedef function0<void> RosAdvertiserFn;
  typedef function0<void> CopyFromRtcToRosFn;

  std::vector<RosSubscriberFn> rosSubscriberFnList;
  std::vector<RosAdvertiserFn> rosAdvertiserFnList;

  std::vector<CopyFromRtcToRosFn> copyFromRtcToRosFnList;

  /*!
   * A vector containing Subscriber/Publisher objects (not references). We have to hold onto them as
   * letting the Subscriber go out of scope would cause the node unsubscribe from the topic.
   */
  std::vector<ros::Subscriber*> rosSubscriberList;
  std::vector<ros::Publisher*> rosPublisherList;

  ros::NodeHandle n;

  /*!
   * A function to be called to add out port to the RTC.
   */
  RegisterRtcOutPortFn registerRtcOutPortFn;
  std::vector<RegisterRtcOutPortFnWrapper> registerRtcOutPortFnWrapperList;

  /*!
   * A function to be called to add in port to the RTC.
   */
  RegisterRtcInPortFn registerRtcInPortFn;
  std::vector<RegisterRtcInPortFnWrapper> registerRtcInPortFnWrapperList;

public:

  void setRegisterRtcOutPortFn(
      function2<bool, const char*, RTC::OutPortBase&> fn) {
    registerRtcOutPortFn = fn;
  }

  void setRegisterRtcInPortFn(
      function2<bool, const char*, RTC::InPortBase&> fn) {
    registerRtcInPortFn = fn;
  }

  void doRegisterRtcOutPort() {
    for (int i = 0; i < registerRtcOutPortFnWrapperList.size(); ++i) {
      registerRtcOutPortFnWrapperList[i](registerRtcOutPortFn);
    }
  }

  void doRegisterRtcInPort() {
    for (int i = 0; i < registerRtcInPortFnWrapperList.size(); ++i) {
      registerRtcInPortFnWrapperList[i](registerRtcInPortFn);
    }
  }

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

  void checkRtcInPort() {
    for (int i = 0; i < copyFromRtcToRosFnList.size(); ++i) {
      copyFromRtcToRosFnList[i]();
    }
  }

public:

  /*!
   * Links a ROS topic to an RTC out port.
   *
   * Note: RtmType has to have a parameterless constructor.
   * TODO: should the name be reference?
   */
  template<class RosType, class RtmType>
  void addNewRosToRtmLink(const std::string& name,
      RosToRtmHandler<RosType, RtmType>& handler) {

    RosToRtmLink<RtmType>* link = new RosToRtmLink<RtmType>(name);

    RegisterRtcOutPortFnWrapper regRtcOutPortFnWrap;
    regRtcOutPortFnWrap = bind(&Config::addOutPortWrapper, this, _1,
        name.c_str(), &link->rtcOutPort);
    registerRtcOutPortFnWrapperList.push_back(regRtcOutPortFnWrap);

    RosSubscriberFn rosSubscriberFn;
    rosSubscriberFn = bind(
        &Config::subscribeToRosTopic<RosType, RtmType>, this, link,
        handler);
    rosSubscriberFnList.push_back(rosSubscriberFn);
  }

  void addOutPortWrapper(RegisterRtcOutPortFn fn, const char* name,
      RTC::OutPortBase* outport) {
    fn(name, *outport);
  }

  /*!
   * This member function subscribes to a ROS topic, and links its input to the given RTC out port.
   * It also requires a converter function which converts the input data from ROS to the output type of RTC.
   */
  template<class RosType, class RtmType>
  void subscribeToRosTopic(RosToRtmLink<RtmType>* link,
      RosToRtmHandler<RosType, RtmType>& handler) {
    link->rosSubscriber = n.subscribe<RosType>(link->name, 1000,
        bind(&Config::copyFromRosToRtc<RosType, RtmType>, this, _1,
            link, handler));
    rosSubscriberList.push_back(&link->rosSubscriber);
  }

  /*!
   * This member function is a callback function for the ROS node. It is called whenever a message arrives
   * on the ROS topic.
   */
  template<class RosType, class RtmType>
  void copyFromRosToRtc(const shared_ptr<RosType const>& msgIn,
      RosToRtmLink<RtmType>* link,
      RosToRtmHandler<RosType, RtmType> handler) {

    handler.convert(msgIn, link->rtcOutPortBuffer);

    if (handler.hasCallback) {
      handler.callback(msgIn, link->rtcOutPortBuffer, *link);
    }

    writeToRtcPort(&link->rtcOutPort);

  }

  /*!
   * Sends the data to the given port from the buffer associated with it.
   */
  template<class RtmType>
  void writeToRtcPort(RTC::OutPort<RtmType>* outPort) {
    outPort->write();
  }

  /*!
   * Links an RTC in port to a ROS topic.
   *
   * Note: RtmType has to have a parameterless constructor.
   */
  template<class RtmType, class RosType>
  void addNewRtmToRosLink(const std::string& name,
      RtmToRosHandler<RtmType, RosType>& handler) {

    RtmToRosLink<RtmType>* link = new RtmToRosLink<RtmType>(name);

    RegisterRtcInPortFnWrapper regRtcInPortFnWrap;
    regRtcInPortFnWrap = bind(&Config::addInPortWrapper, this, _1,
        name.c_str(), &link->rtcInPort);
    registerRtcInPortFnWrapperList.push_back(regRtcInPortFnWrap);

    CopyFromRtcToRosFn copyFromRtcToRosFn = bind(
        &Config::copyFromRtcToRos<RtmType, RosType>, this, link,
        handler);
    copyFromRtcToRosFnList.push_back(copyFromRtcToRosFn);

    function0<void> rosAdvertiserFn;
    rosAdvertiserFn = bind(
        &Config::advertiseRosTopic<RosType, RtmType>, this, link);
    rosAdvertiserFnList.push_back(rosAdvertiserFn);
  }

  void addInPortWrapper(RegisterRtcInPortFn fn, const char* name,
      RTC::InPortBase* inport) {
    fn(name, *inport);
  }

  /*!
   * This member function is a callback function for the RTC port. It is called periodically, and passed the
   * Link object containing an RTC port, and a ROS publisher.
   */
  template<class RtmType, class RosType>
  void copyFromRtcToRos(RtmToRosLink<RtmType>* link,
      RtmToRosHandler<RtmType, RosType>& handler) {

    RTC::InPort<RtmType>* inPort = &link->rtcInPort;
    bool hasResult = readFromRtcPort<RtmType>(inPort);
    if (hasResult) {
      RosType msgOut;
      handler.convert(link->rtcInPortBuffer, msgOut);

      if (handler.hasCallback) {
        handler.callback(link->rtcInPortBuffer, msgOut, *link);
      }

      link->rosPublisher.publish(msgOut);
    }

  }

  /*!
   * Checks whether new data has arrived to the given RTC port. If it has, it loads the data into the buffer
   * associated with the port.
   */
  template<class RtmType>
  bool readFromRtcPort(RTC::InPort<RtmType>* inPort) {
    if (inPort->isNew()) {
      inPort->read();
      return true;
    } else {
      return false;
    }
  }

  /*!
   * This member function advertises a ROS topic and links the given RTC in port to the topic.
   */
  template<class RosType, class RtmType>
  void advertiseRosTopic(RtmToRosLink<RtmType>* link) {
    link->rosPublisher = n.advertise<RosType>(link->name, 1000);
    rosPublisherList.push_back(&link->rosPublisher);
  }

};

template<class Component>
void createNewGateway(int argc, char** argv, Config config,
    bool block = true) {

  coil::Properties profile(config.comp_spec);

  RTC::Manager* manager;
  RTC::RtcBase* comp;

  manager = RTC::Manager::init(argc, argv);
  manager->init(argc, argv);
  manager->registerFactory(profile, RTC::Create<Component>,
      RTC::Delete<Component>);
  manager->activateManager();

  comp = manager->createComponent("Hybrid");
  if (comp == NULL) {
    std::cerr << "Component create failed." << std::endl;
    abort();
  }

  Component* gw = static_cast<Component*>(comp);
  gw->setConfig(&config);
  gw->setUpPorts();

  manager->runManager(!block);
}
}

#endif /* GATEWAYHELPER_H_ */
