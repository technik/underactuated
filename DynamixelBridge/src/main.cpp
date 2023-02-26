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

	struct Packet
	{};

	class Controller
	{
	public:
		//
	};
}

void setup()
{
	// put your setup code here, to run once:
	pinMode(13, OUTPUT);
}

void loop()
{
	// put your main code here, to run repeatedly:
	digitalWrite(13, HIGH);
	delay(500);
	digitalWrite(13, LOW);
	delay(500);
}