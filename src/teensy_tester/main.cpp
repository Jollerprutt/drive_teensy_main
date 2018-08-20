#include "Arduino.h"
#include "phoenix_can_shield.h"
#include "publisher.hpp"
#include "subscriber.hpp"
#include "parameter.hpp"

// Node settings
static constexpr uint32_t nodeID = 100;
static constexpr uint8_t swVersion = 1;
static constexpr uint8_t hwVersion = 1;
static const char* nodeName = "test_IMU_node";


// application framerate
static constexpr float framerate = 1000;


void setup()
{
  delay(3000);
  Serial.begin(115200);
  Serial.println("Setup");

  // init LEDs
  initLeds();

  // Create a node
  systemClock = &initSystemClock();
  canDriver = &initCanDriver();
  node = new Node<NodeMemoryPoolSize>(*canDriver, *systemClock);
  initNode(node, nodeID, nodeName, swVersion, hwVersion);

  // init publisher
  initPublisher(node);

  // init subscriber
  initSubscriber(node);

  // set up filters (must be after publisher & subscriber setup!)
  configureCanAcceptanceFilters(*node);

  // init parameter
  initParameter(node);

  // start up node
  node->setModeOperational();

  Serial.println("Setup Finished");
}



void loop()
{
  const int error = node->spinOnce();
  // // wait in cycle
  // cycleWait(framerate);

  // // do some CAN stuff
  // cycleNode(node);

  // publish messages
  // cyclePublisher(50);
  cyclePublisherIMU(50);

  // // toggle heartbeat
  // toggleHeartBeat();
}
