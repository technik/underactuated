#pragma once

#include <cstdint>
#include <functional>

class RobotHAL
{
public:

	struct ActuatorState
	{
		uint8_t id;
		uint16_t position;
		uint16_t velocity;
		uint16_t torque;
	};

	using StateReadDelegate = std::function<void(RobotHAL* hal, int numActuators, const ActuatorState* stateArray)>;

	// Write torque to an array of actuators with contiguous ids
	virtual void writeTorque(int numActuators, int actuatorId0, uint16_t* torqueArray) = 0;

	// Register a callback to be called when HAL has a received new robot state
	virtual void onStateRead(const StateReadDelegate& cb) = 0;

};
