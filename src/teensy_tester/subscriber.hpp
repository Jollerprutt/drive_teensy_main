#ifndef	SUBSCRIBER_HPP
#define	SUBSCRIBER_HPP

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <uavcan/equipment/actuator/Command.hpp>
#include "phoenix_can_shield.h"

using namespace uavcan;

Subscriber<protocol::debug::KeyValue> *keySubscriber;
Subscriber<equipment::actuator::Command> *commandSubscriber;

MonotonicTime last_time;

void keyMessageCallback(const uavcan::protocol::debug::KeyValue& msg)
{
  Serial.print("Key Diff: ");
  MonotonicTime curr_time = systemClock->getMonotonic();

  Serial.println((uint32_t)(curr_time - last_time).toUSec());
  last_time = curr_time;

}

void commandMessageCallback(const uavcan::equipment::actuator::Command& msg4)
{
  
  Serial.print("Command: ");
  
  Serial.println(" rad");

}

void initSubscriber(Node<NodeMemoryPoolSize> *node)
{
  // create a subscriber
  keySubscriber = new Subscriber<protocol::debug::KeyValue>(*node);
  commandSubscriber = new Subscriber<equipment::actuator::Command>(*node);

  if(keySubscriber->start(keyMessageCallback) < 0)
  {
    Serial.println("Unable to start log message subscriber!");
  }

  if(commandSubscriber->start(commandMessageCallback) < 0)
  {
    Serial.println("Unable to start command message subscriber!");
  }
}

#endif
