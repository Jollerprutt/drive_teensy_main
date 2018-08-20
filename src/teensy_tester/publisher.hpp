#ifndef	PUBLISHER_HPP
#define	PUBLISHER_HPP

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <uavcan/equipment/ahrs/RawIMU.hpp>
#include "phoenix_can_shield.h"

using namespace uavcan;

// publisher
Publisher<protocol::debug::KeyValue> *keyPublisher;
Publisher<equipment::ahrs::RawIMU> *RawIMUPublisher;


void initPublisher(Node<NodeMemoryPoolSize> *node)
{
  // create publishers
  keyPublisher = new Publisher<protocol::debug::KeyValue>(*node);
  RawIMUPublisher = new Publisher<equipment::ahrs::RawIMU>(*node);

  // initiliaze publishers
  if(keyPublisher->init() < 0)
  {
    Serial.println("Unable to initialize key message publisher!");
  }
  if(RawIMUPublisher->init() < 0)
  {
    Serial.println("Unable to initialize RawIMU message publisher!");
  }

  // set TX timeout
  keyPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
  RawIMUPublisher->setTxTimeout(MonotonicDuration::fromUSec(500));
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
      equipment::ahrs::RawIMU msg3;

      msg3.integration_interval = 2;
      
      msg3.rate_gyro_latest[0] = 1;
      msg3.rate_gyro_latest[1] = 2;
      msg3.rate_gyro_latest[2] = 3;

      msg3.rate_gyro_integral[0] = 1;
      msg3.rate_gyro_integral[1] = 2;
      msg3.rate_gyro_integral[2] = 3;

      msg3.accelerometer_latest[0] = 1;
      msg3.accelerometer_latest[1] = 2;
      msg3.accelerometer_latest[2] = 3;

      msg3.accelerometer_integral[0] = 1;
      msg3.accelerometer_integral[1] = 2;
      msg3.accelerometer_integral[2] = 3;

      msg3.covariance[0] = 1;
      msg3.covariance[1] = 2;
      msg3.covariance[2] = 3;

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


#endif
