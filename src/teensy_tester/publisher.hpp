#ifndef	PUBLISHER_HPP
#define	PUBLISHER_HPP

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <uavcan/equipment/ahrs/RawIMU.hpp>
#include <uavcan/equipment/actuator/Command.hpp>

#include "phoenix_can_shield.h"

using namespace uavcan;

// publisher
Publisher<protocol::debug::KeyValue> *keyPublisher;
Publisher<equipment::ahrs::RawIMU> *RawIMUPublisher;
Publisher<equipment::actuator::Command> *commandPublisher;

float zero = 0.0;
float one = 1.0;
float two = 2.0;
float three = 3.0;
uint8_t CID = 1;
uint8_t CT = 1;
float servoMid = 1.5708;

void initPublisher(Node<NodeMemoryPoolSize> *node)
{
  // create publishers
  keyPublisher = new Publisher<protocol::debug::KeyValue>(*node);
  RawIMUPublisher = new Publisher<equipment::ahrs::RawIMU>(*node);
  commandPublisher = new Publisher<equipment::actuator::Command>(*node);

  // initiliaze publishers
  if(keyPublisher->init() < 0)
  {
    Serial.println("Unable to initialize key message publisher!");
  }
  if(RawIMUPublisher->init() < 0)
  {
    Serial.println("Unable to initialize RawIMU message publisher!");
  }
  if(commandPublisher->init() < 0)
  {
    Serial.println("Unable to initialize command message publisher!");
  }

  // set TX timeout
  keyPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  RawIMUPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  commandPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
}

static int counter = 0;
MonotonicTime lastPub = MonotonicTime::fromMSec(0);

void cyclePublisher(const int pubFreq)
{
  // send everyone the truth
  if(lastPub + MonotonicDuration::fromMSec(1000/(float)pubFreq) < systemClock->getMonotonic())
  {
    {
      protocol::debug::KeyValue msg;

      msg.value = counter++;
      msg.key = "this is a longer message for testing with longer messages";

      //Serial.print("Trans: ");
      //Serial.println(msg.value);

      const int pres = keyPublisher->broadcast(msg);
      if (pres < 0)
      {
        Serial.println("Error while broadcasting key message");
      }
    }
    lastPub = systemClock->getMonotonic();
  }
}

MonotonicTime lastPubIMU = MonotonicTime::fromMSec(0);

void cyclePublisherIMU(const int pubFreq)
{
  // send everyone the truth
  if(lastPubIMU + MonotonicDuration::fromMSec(1000/(float)pubFreq) < systemClock->getMonotonic())
  {
    {
      // Serial.println("Hi!");
      equipment::ahrs::RawIMU msg3;

      msg3.integration_interval = one;
      
      msg3.rate_gyro_latest[0] = one;
      msg3.rate_gyro_latest[1] = two;
      msg3.rate_gyro_latest[2] = three;

      msg3.rate_gyro_integral[0] = one;
      msg3.rate_gyro_integral[1] = two;
      msg3.rate_gyro_integral[2] = three;

      msg3.accelerometer_latest[0] = one;
      msg3.accelerometer_latest[1] = two;
      msg3.accelerometer_latest[2] = three;

      msg3.accelerometer_integral[0] = one;
      msg3.accelerometer_integral[1] = two;
      msg3.accelerometer_integral[2] = three;

      // msg3.covariance[0] = zero;
      /*
      msg3.covariance[0] = three;
      msg3.covariance[1] = three;
      msg3.covariance[2] = three;
      msg3.covariance[3] = three;*/

      //Serial.print("Trans: ");
      //Serial.println(msg.value);

      const int pres = RawIMUPublisher->broadcast(msg3);
      if (pres < 0)
      {
        Serial.println("Error while broadcasting key message");
      }
    }
    lastPubIMU = systemClock->getMonotonic();
  }
}

MonotonicTime lastPubCommand = MonotonicTime::fromMSec(0);

void cyclePublisherCommand(const int pubFreq)
{
  // send everyone the truth
  if(lastPubCommand + MonotonicDuration::fromMSec(1000/(float)pubFreq) < systemClock->getMonotonic())
  {
    {
      // Serial.println("Hi!");
      equipment::actuator::Command msg6;

      msg6.actuator_id = CID;
      msg6.command_type = CT;
      msg6.command_value = servoMid;

      const int pres = commandPublisher->broadcast(msg6);
      if (pres < 0)
      {
        Serial.println("Error while broadcasting command message");
      }
    }
    lastPubCommand = systemClock->getMonotonic();
  }
}


#endif
