#include <Arduino.h>

namespace Dynamixel
{
	struct EEPROMControlTable
	{
		uint16_t 	modelNumber;
		uint8_t		firmwareVersion;
		uint8_t		id;
		uint8_t 	baudRate;
		uint8_t 	returnDelay;
		uint16_t	cwAngleLimit;
		uint16_t	ccwAngleLimit;
		uint8_t		temperatureLimit;
		uint8_t		minVoltage;
		uint8_t		maxVoltage;
		uint16_t	maxTorque;
		uint8_t		statusReturnLevel;
		uint8_t		AlarmLED;
		uint8_t		shutdown;
	};

	struct RAMControlTable
	{
		uint8_t		torqueEnable;
		uint8_t		led;
		uint8_t		cwComplianceMargin;
		uint8_t		ccwComplianceMargin;
		uint8_t		cwComplianceSlope;
		uint8_t		ccwComplianceSlope;
		uint16_t	goalPosition;
		uint16_t	movingSpeed;
		uint16_t	torqueLimit;
		uint16_t	presentPosition;
		uint16_t	presentSpeed;
		uint16_t	presentLoad;
		uint8_t		presentVoltage;
		uint8_t		presentTemperature;
		uint8_t 	registered;
		uint8_t 	moving;
		uint8_t 	lock;
		uint16_t 	punch;
	};

	enum class Address : uint8_t
	{
		// EEPROM Addresses
		ModelNumber = 0,
		FirmwareVersion = 2,
		Id,
		BaudRate,
		ReturnDelay,
		CWAngleLimit,
		CCWAngleLimit = 8,
		TemperatureLimit = 11,
		MinVoltage,
		MaxVoltage,
		MaxTorque,
		StatusReturnLevel = 16,
		AlarmLED,
		Shutdown,

		// RAM Addresses
		TorqueEnable = 24,
		LED,
		CWComplianceMargin,
		CCWComplianceMargin,
		CWComplianceSlope,
		CCWComplianceSlope,
		GoalPosition,
		MovingSpeed = 32,
		TorqueLimit = 34,
		PresentPosition = 36
	};

	enum class Instruction : uint8_t
	{
		Ping = 0x01,
		Read = 0x02,
		Write = 0x03,
		RegWrite = 0x04,
		Action = 0x05,
		FactoryReset = 0x06,
		Reboot = 0x08,
		SyncWrite = 0x83,
		BulkRead = 0x92,
	};

	enum class Status : uint8_t
	{
		Success = 0,
		InputVoltageError = 1,
		AngleLimitError = 2,
		OverheatingError = 4,
		RangeError = 8,
		ChecksumError = 16,
		OverloadError = 32,
		InstructionError = 64
	};

	struct Packet
	{
		static constexpr uint8_t cMaxPayload = sizeof(RAMControlTable) + 2; 
		uint8_t m_id;
		uint8_t m_length;
		uint8_t m_opcode;
		uint8_t m_payload[cMaxPayload];
		uint8_t m_checksum;

		uint8_t computeChecksum()
		{
			uint8_t checksum = m_id + m_length + m_opcode;
			
			for(int  i = 0; i < m_length-1; ++i)
			{
				checksum += m_payload[i];
			}

			return ~checksum;
		}

		void setChecksum()
		{
			m_checksum = computeChecksum();
		}

		bool verifyChecksum()
		{
			return m_checksum == computeChecksum();
		}
	};

	struct InstructionPacket : Packet
	{
		void ledOn()
		{
			setRegister8(Address::LED, 1);
		}

		void ledOff()
		{
			setRegister8(Address::LED, 0);
		}

		void setMaxTorque(uint16_t maxTorque)
		{
			setRegister16(Address::MaxTorque, maxTorque);
		}

		void enableTorque(bool on)
		{
			setRegister8(Address::TorqueEnable, on);
		}

		void setTorqueLimit(uint16_t limit)
		{
			setRegister16(Address::TorqueLimit, limit);
		}

		void setCWAngleLimit(uint16_t limit)
		{
			setRegister16(Address::CWAngleLimit, limit);
		}

		void setCCWAngleLimit(uint16_t limit)
		{
			setRegister16(Address::CCWAngleLimit, limit);
		}

		// When in WheelMode: 0-1023 is positive torque, 1024-2047 is negative torque
		void setMovingSpeed(uint16_t speed)
		{
			setRegister16(Address::MovingSpeed, speed);
		}

		void setGoalPos(uint16_t pos)
		{
			setRegister16(Address::GoalPosition, pos);
		}

		void setRegister8(Address address, uint8_t x)
		{
			write(address, &x);
		}

		void setRegister16(Address address, uint16_t x)
		{
			write(address, &x);
		}

		template<class T>
		void write(Address address, const T* data)
		{
			m_opcode = (uint8_t)Instruction::Write;

			// Set payload
			m_payload[0] = (uint8_t)address;
			auto rawData = (const uint8_t*)data;
			for(int i = 0; i < sizeof(T); ++i)
				m_payload[i+1] = rawData[i];
			
			setPayloadSize(sizeof(T)+1);
		}

		void read(Address address, uint8_t byteCount)
		{
			m_opcode = (uint8_t)Instruction::Read;

			m_payload[0] = (uint8_t)address;
			m_payload[1] = byteCount;

			setPayloadSize(2);
		}

		void setPayloadSize(uint8_t payloadSize)
		{
			m_length = payloadSize + 2;
		}
	};

	struct StatusPacket : Packet
	{
		Status getStatus() const { return (Status)m_opcode; }
	};

	class Monitor
	{
	public:
		enum State
		{
			Header1 = 0,
			Header2,
			PacketId,
			Length,
			Payload,
			Ready
		};

		bool isReady() const { return m_state == State::Ready; }

		uint8_t m_state = Header1;
		uint8_t m_payloadPos;

		void reset()
		{
			m_state = State::Header1;
			m_payloadPos = 0;
		}

		void read(HardwareSerial& serial, Packet& packet)
		{
			if(!serial.available())
			{
				return;
			}
			uint8_t c = serial.read();
			switch (m_state)
			{
			case State::Header1:
			case State::Header2:
			{
				if(c == 0xff)
					m_state++;
				else
					m_state = State::Header1;
				break;
			}
			case State::PacketId:
			{
				m_state++;
				packet.m_id = c;
			}
			case State::Length:
			{
				m_state++;
				packet.m_length = c;
				m_payloadPos = 0;
			}
			case State::Payload:
			{
				uint8_t* payloadPos = &packet.m_opcode;
				payloadPos[m_payloadPos] = c;
				m_payloadPos++;
				if(m_payloadPos == packet.m_length)
				{
					if(packet.verifyChecksum())
					{
						m_state = State::Ready;
					}
					else
					{
						m_state = State::Header1;
					}
				}
			}
			default:
				m_state = State::Header1;
				break;
			}
		}
	};

	#define DXSerial Serial1

	class Controller
	{
	public:
		void send()
		{
			// Disable listening on this port
			UCSR1B &= ~(1<<RXEN1);
			// Header
			DXSerial.write(0xff);
			DXSerial.write(0xff);
			DXSerial.write(m_packet.m_id);
			// Length
			DXSerial.write(m_packet.m_length);
			// Instruction
			DXSerial.write(uint8_t(m_packet.m_opcode));
			// Payload
			DXSerial.write(m_packet.m_payload, m_packet.m_length-2);
			// Checksum
			DXSerial.write(m_packet.computeChecksum());

			delayMicroseconds(10); // Avoid transient noise before listening for a response

			// Enable listening on this port again
			UCSR1B |= (1<<RXEN1);
		}

		// Returns false on timeout
		bool receive()
		{
			static constexpr uint32_t TimeOutMs = 5; 
			auto t = millis();
			Monitor monitor;
			while(!monitor.isReady())
			{
				if(millis() - t > TimeOutMs)
				{
					return false; // Timeout
				}
				monitor.read(DXSerial, m_packet);
			}

			return true;
		}

		void setId(uint8_t id)
		{
			m_packet.m_id = id;
		}

		template<class T>
		bool read(Address address, T& dst)
		{
			auto& instruction = *(InstructionPacket*)(&m_packet);
			instruction.read(address, sizeof(T));
			send();

			if(receive())
			{
				memcpy(&dst, m_packet.m_payload, sizeof(T));
				return true;
			}

			return false; // Didn't receive anything
		}

		Packet m_packet;
	};
}

Dynamixel::Controller g_controller;
Dynamixel::Monitor g_pcMonitor;

void setup()
{
	// put your setup code here, to run once:
	pinMode(13, OUTPUT);

	Serial.begin(115200);
	Serial1.begin(1000000);

	g_controller.setId(4); // Need to set the id ahead of time
	//g_controller.m_packet.ledOn();
	//g_controller.send();
	//g_controller.m_packet.enableTorque(true);
	//g_controller.send();
}

unsigned long lastTick = 0;
bool ledOn = false;

struct CircularBuffer
{
	uint8_t m_readPos = 0;
	uint8_t m_writePos = 0;
	uint8_t m_buffer[64];

	void write(uint8_t c)
	{
		m_buffer[m_writePos & 0x3f] = c;
		m_writePos++;
	}

	bool available() const
	{
		return m_writePos > m_readPos;
	}

	uint8_t read()
	{
		auto c = m_buffer[m_readPos & 0x3f];
		m_readPos++;
		m_readPos &= 0x7f;
		m_writePos &= 0x7f;
		return c;
	}
};

CircularBuffer g_ringBuffer;

void loop()
{
	uint16_t currentPos = 0;
	bool ok = g_controller.read(Dynamixel::Address::PresentPosition, currentPos);
	if(ok)
	{
		Serial.print("pos: ");
		Serial.println(currentPos);
	}
	else
	{
		Serial.println("error");
	}
	delay(500);
	/*
	// Read serial message
	g_pcMonitor.read(Serial, g_controller.m_packet);
	if(g_pcMonitor.isReady())
	{
		g_controller.send();
	}
	// Execute command
	g_controller.send();
	// Read response
	while(Serial1.available())
	{
		g_ringBuffer.write(Serial1.read());
	}
	// Report response
	while(g_ringBuffer.available())
	{
		Serial.write(g_ringBuffer.read());
	}
	// Tick
	auto t = millis();
	if(t - lastTick)
	{
		lastTick = t;
		if(ledOn)
		{
			g_controller.m_packet.ledOff();
			g_controller.send();
		}
		else
		{
			g_controller.m_packet.ledOn();
			g_controller.send();
		}
		ledOn = !ledOn;
	}
	*/
}