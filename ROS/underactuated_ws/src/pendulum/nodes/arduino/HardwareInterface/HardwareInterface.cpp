#include <ros.h>
#include <geometry_msgs/Wrench.h>
#include <gazebo_msgs/ModelState.h>
#include <dynamixel.h>

const double MAX_TORQUE = 1.5;

ros::NodeHandle node; 

gazebo_msgs::ModelState modelState;
ros::Publisher statePublisher("pendulum_x", &modelState);

// Returns 0 is positive, 1 if negative
uint8_t sign(double x)
{
  return x > 0.0 ? 0 : 1;
}

uint16_t NM2Uint16(double torque)
{
  // 0-1023 is positive torque, 1024-2043 is negative torque
  const double torquePercent = torque/MAX_TORQUE;

  return static_cast<uint16_t>(1023.0*(sign(torquePercent) + torquePercent));
}

void dynamixelInterface(const geometry_msgs::Wrench& wrench)
{
  double u = wrench.torque.z;

  g_controller.m_packet.setMovingSpeed(NM2Uint16(u));
  g_controller.send();
  g_controller.m_packet.enableTorque(true);
  g_controller.send();
}

ros::Subscriber<geometry_msgs::Wrench> controlSubscriber("control_u", &dynamixelInterface);

Dynamixel::Controller g_controller;

void setup()
{
  // Setup ROS
  node.initNode();
  node.advertise(statePublisher);
  node.subscribe(controlSubscriber);

  // Setup Dynamixel
  pinMode(13, OUTPUT);

	g_controller.setId(4); // Need to set the id ahead of time
	g_controller.m_packet.ledOn();
	g_controller.send();
  g_controller.m_packet.setMaxTorque(NM2Uint16(MAX_TORQUE));
	g_controller.send();
  g_controller.m_packet.setMovingSpeed(0);
  g_controller.send();
	g_controller.m_packet.enableTorque(true);
	g_controller.send();

  // Setting angle limits will put the Dynamixel in "WheelMode"
	g_controller.m_packet.SetCWAngleLimit(0);
	g_controller.send();
	g_controller.m_packet.SetCCWAngleLimit(0);
	g_controller.send();
}

Dynamixel::Packet packet;
Dynamixel::Monitor monitor;

void loop()
{
  
  modelState.pose.orientation.w = 1.0;
  modelState.twist.angular.z = 1.0;

  monitor.read(Serial1, packet);
  if(monitor.isReady())
  {
    // TODO
  }

  // Publish state to ROS
  statePublisher.publish(&modelState);
  node.spinOnce();
  delay(1);
}