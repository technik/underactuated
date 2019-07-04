#pragma once

#include "RobotHAL.h"
#include <thread>

class Serial;

class SerialRobotBackend : public RobotHAL
{
public:
	SerialRobotBackend();
	~SerialRobotBackend();

	void init(const char* serialPortName);

	void writeTorque(int numActuators, int actuatorId0, uint16_t* torqueArray) override;
	void onStateRead(const StateReadDelegate& cb) override
	{
		m_onStateRead = cb;
	}

private:
	void readDataByte();
	void processReceivedPacket();

private:
	StateReadDelegate m_onStateRead;
	Serial* m_comm = nullptr;
	std::thread m_commThread;
	bool m_mustClose = false;

	static constexpr int READ_BUFFER_SIZE = 256;
	uint8_t m_readBuffer[READ_BUFFER_SIZE] = {};
	int m_writePos = 0;
	int m_readPos = 0;

	bool m_haveValidPacket = false;

	static constexpr uint8_t PACKET_HEADER = 0x55;
	enum PackedProcessorState
	{
		Empty,
		HaveHeader,
		HaveSize,
		HaveData
	};

	PackedProcessorState m_processorState = Empty;
};
