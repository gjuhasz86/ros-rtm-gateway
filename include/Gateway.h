#ifndef GATEWAY_H
#define GATEWAY_H
#include "GatewayHelper.h"

class Gateway: public RTC::DataFlowComponentBase {
protected:

  GatewayFactory::Config config;

public:
  Gateway(RTC::Manager* manager);
  ~Gateway();

  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  virtual void setUpPorts();
  virtual void setConfig(GatewayFactory::Config*);
};

////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
//

////////////////////////////////////////////////////////////////////////////////
// constructors

Gateway::Gateway(RTC::Manager* manager) :
    RTC::DataFlowComponentBase(manager), config() { // creating an empty config, will be thrown away
}

Gateway::~Gateway() {
}

////////////////////////////////////////////////////////////////////////////////
// Public methods

void Gateway::setUpPorts() {
  boost::function2<bool, const char*, RTC::OutPortBase&> addOutPortFn =
      boost::bind(&Gateway::addOutPort, this, _1, _2);
  config.setRegisterRtcOutPortFn(addOutPortFn);
  config.doRegisterRtcOutPort();

  boost::function2<bool, const char*, RTC::InPortBase&> addInPortFn =
      boost::bind(&Gateway::addInPort, this, _1, _2);
  config.setRegisterRtcInPortFn(addInPortFn);
  config.doRegisterRtcInPort();
}

void Gateway::setConfig(GatewayFactory::Config* c) {
  config = *c;
}

RTC::ReturnCode_t Gateway::onActivated(RTC::UniqueId ec_id) {
  config.doSubscibe();
  config.doAdvertise();
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onDeactivated(RTC::UniqueId ec_id) {
  config.doUnsubscribe();
  config.doStopAdvertise();
  return RTC::RTC_OK;
}

RTC::ReturnCode_t Gateway::onExecute(RTC::UniqueId ec_id) {
  config.checkRosInput();
  config.checkRtcInPort();
  return RTC::RTC_OK;
}

#endif // GATEWAY_H
