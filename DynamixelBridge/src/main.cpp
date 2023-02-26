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

	enum class RAMAddress : uint8_t
	{
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

	struct Packet
	{
		static constexpr uint8_t cMaxPayload = sizeof(RAMControlTable) + 2; 
		uint8_t m_id;
		uint8_t m_length;
		Instruction m_instruction;
		uint8_t m_payload[cMaxPayload];
		uint8_t m_checksum;

		void setPayloadSize(uint8_t payloadSize)
		{
			m_length = payloadSize + 2;
		}

		uint8_t computeChecksum()
		{
			uint8_t checksum = m_id + m_length + (uint8_t)m_instruction;
			
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

	#define DXSerial Serial1

	class Controller
	{
	public:
		void LedOn()
		{
			SetRegister8(RAMAddress::LED, 1);
		}

		void LedOff()
		{
			SetRegister8(RAMAddress::LED, 0);
		}

		void EnableTorque(bool on)
		{
			SetRegister8(RAMAddress::TorqueEnable, on);
		}

		void SetGoalPos(uint16_t pos)
		{
			SetRegister16(RAMAddress::GoalPosition, pos);
		}

		void SetRegister8(RAMAddress address, uint8_t x)
		{
			Write(address, &x);
		}

		void SetRegister16(RAMAddress address, uint16_t x)
		{
			Write(address, &x);
		}

		template<class T>
		void Write(RAMAddress address, const T* data)
		{
			m_packet.m_instruction = Instruction::Write;

			// Set payload
			m_packet.m_payload[0] = (uint8_t)address;
			auto rawData = (const uint8_t*)data;
			for(int i = 0; i < sizeof(T); ++i)
				m_packet.m_payload[i+1] = rawData[i];
			
			m_packet.setPayloadSize(sizeof(T)+1);

			Send();
		}

		void Send()
		{
			// Header
			DXSerial.write(0xff);
			DXSerial.write(0xff);
			DXSerial.write(m_packet.m_id);
			// Length
			DXSerial.write(m_packet.m_length);
			// Instruction
			DXSerial.write(uint8_t(m_packet.m_instruction));
			// Payload
			DXSerial.write(m_packet.m_payload, m_packet.m_length-2);
			// Checksum
			DXSerial.write(m_packet.computeChecksum());

			delay(2); // Ignore the response message
		}

		void setId(uint8_t id)
		{
			m_packet.m_id = id;
		}

		Packet m_packet;
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

		void Read(HardwareSerial& serial, Packet& packet)
		{
			while(!serial.available())
			{}
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
				uint8_t* payloadPos = (uint8_t*)&packet.m_instruction;
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
}

Dynamixel::Packet g_writePackage;
Dynamixel::Packet g_readPackage;

Dynamixel::Monitor g_monitor;
Dynamixel::Controller g_controller;

void setup()
{
	// put your setup code here, to run once:
	pinMode(13, OUTPUT);

	Serial.begin(115200);
	Serial1.begin(1000000);

	g_controller.setId(4);
	g_controller.LedOn();
	g_controller.EnableTorque(true);
}

void loop()
{
	// Read serial message
	Dynamixel::Packet pcPacket;
	Dynamixel::Monitor pcMonitor;
	while(!pcMonitor.isReady())
	{
		pcMonitor.Read(Serial, pcPacket);
	}
	// Execute command
	// Read response
	// Report response
	// put your main code here, to run repeatedly:
	g_controller.LedOff();
	g_controller.SetGoalPos(100);
	digitalWrite(13, HIGH);
	delay(250);
	
	g_controller.LedOn();
	g_controller.SetGoalPos(200);
	digitalWrite(13, LOW);
	delay(500);
}