//-------------------------------------------------------------------------------------------------
#include "SerialRobotBackend.h"
#include "Serial.h"

#include <cassert>
#include <thread>

//-------------------------------------------------------------------------------------------------
SerialRobotBackend::SerialRobotBackend()
{
	// TODO: Init logging
}

//-------------------------------------------------------------------------------------------------
SerialRobotBackend::~SerialRobotBackend()
{
	// End communication thread
	if (m_commThread.joinable())
	{
		m_mustClose = true;
		m_commThread.join();
	}
	// Close serial port, if open
	if (m_comm)
		delete m_comm;
	// Flush and stop logging
}

//-------------------------------------------------------------------------------------------------
void SerialRobotBackend::readDataByte()
{
	switch (m_processorState)
	{
	case Empty:
	{
		uint8_t dataByte = m_comm->read();
		if (dataByte == PACKET_HEADER)
			m_processorState = HaveHeader;
		break;
	}
	case HaveHeader:
		m_readBuffer[0] = m_comm->read();
		m_processorState = HaveSize;
		break;
	case HaveSize:
		m_comm->read(m_readBuffer + 1, m_readBuffer[0]);
		m_processorState = HaveData;
		break;
	case HaveData:
		uint8_t crc = m_readBuffer[0];
		for (uint8_t i = 0; i < m_readBuffer[0]; ++i)
		{
			crc ^= m_readBuffer[i + 1];
		}
		uint8_t readCrc = m_comm->read();
		if (readCrc == crc)
		{
			m_haveValidPacket = true;
		}
		m_processorState = Empty;
		break;
	}
}

//-------------------------------------------------------------------------------------------------
void SerialRobotBackend::init(const char* serialPortName)
{
	assert(m_onStateRead);
	// Open serial port
	m_comm = new Serial(serialPortName, 9600);

	// Start reading thread
	m_commThread = std::thread([this]() {
		while (!m_mustClose)
		{
			readDataByte();
			if (m_haveValidPacket)
			{
				processReceivedPacket();
			}
		}
	});
}

//-------------------------------------------------------------------------------------------------
void SerialRobotBackend::processReceivedPacket()
{
	assert(m_haveValidPacket);
	m_haveValidPacket = false;
	uint8_t numActuators = m_readBuffer[0] / 3;

	auto actuators = reinterpret_cast<ActuatorState*>(&m_readBuffer[1]);

	if (m_onStateRead)
	{
		m_onStateRead(this, numActuators, actuators);
	}
}