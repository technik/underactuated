#include <Arduino.h>

#include <CAN.h>
using MCP2515 = MCP2515Class;

const int LedPin = 13;

class ODriveMotor
{
public:
	static constexpr int kCSPin = 10;
	static constexpr int kFrequency = 250e3;
	ODriveMotor(int motorId)
		: m_motorId(motorId << 5)
	{}

	void Init()
	{
		m_bus.setPins(kCSPin);
		m_bus.setClockFrequency(8e3);
		m_bus.setSPIFrequency(125e3);
		m_bus.begin(kFrequency);
		m_bus.clearWriteError(); // Reset any errors
	}

	bool Ping(); // Returns whether the odrive answered a Heartbeat message

	bool SetPosition(float x)
	{
		if(!SetControlMode(ControlMode::Position))
			return false; // Unable to set control mode
			
		if(!SetAxisState(AxisState::ClosedLoopControl))
			return false;
		
		SetPayload(x);
		SetPayload(0, 4); // 0 feed foirward velocity and torque
		return SendCommand(Command::SecInputPos, 8);
	}

private:

	enum class ControlMode : uint32_t
	{
		Voltage = 0,
		Torque,
		Velocity,
		Position,

		Undefined
	};

	enum class InputMode : uint32_t
	{
		Inactive = 0,
		PassThrough = 1,
		VelRamp = 2,
		PosFilter = 3,
		MixChannels = 4,
		TrapTraj = 5,
		TorqueRamp = 6,
		Mirror = 7,
		Tuning = 8
	};

	enum class Command : uint8_t
	{
		SetAxisState = 7,
		GetEncoderEstimates = 9,
		SetControllerMode = 0xb,
		SecInputPos = 0x0c,
		SetInputVel = 0x0d,
		SetInputTorque = 0x0e,
		SetLimits = 0x0f,
		ClearErrors = 0x18
	};

	enum class AxisState : uint32_t
	{
		Undefined = 0,
		Idle,
		StartupSequence,
		FullCalibrationSequence,
		MotorCalibration,
		EncoderIndexSearch,
		EncoderOffsetCalibration,
		ClosedLoopControl,
		LockinSpin,
		Homing,
		EncoderHallPolarityCalibration,
		EncoderHallPhaseCalibration
	};


	bool SetControlMode(ControlMode mode, InputMode inputMode = InputMode::PassThrough)
	{
		if(m_currentControlMode != mode)
		{
			SetPayload((uint32_t)mode);
			SetPayload((uint32_t)inputMode, 4);
			return SendCommand(Command::SetControllerMode, 8);
		}
		return true;
	}


	bool SetAxisState(AxisState state)
	{
		SetPayload((uint32_t)state);
		SetPayload((uint32_t)0, 4);
		return SendCommand(Command::SetAxisState, 8);
	}

	template<class T>
	void SetPayload(T value, int offset = 0)
	{
		*reinterpret_cast<T*>(&m_payload[offset]) = value;
	}

	bool SendCommand(Command cmd, int payloadSize = 0)
	{
		const int canId = m_motorId | ((uint8_t)cmd);
		m_bus.beginPacket(canId);
		if(payloadSize > 0)
		{
			m_bus.write(m_payload, payloadSize);
		}
		return m_bus.endPacket();
	}

	uint8_t m_payload[8];

	int m_motorId;
	MCP2515 m_bus;
	ControlMode m_currentControlMode = ControlMode::Undefined;
};

constexpr int odriveId = 3;
ODriveMotor motor(odriveId);

void setup() {
	Serial.begin(9600);
	//Serial.println("Running motor calibration");
	//bool ok = motor.RunMotorCalibration();
	Serial.println("Type anything to start");
	while(!Serial.available())
	{}
	Serial.println("Starting");
}

void loop() {
	Serial.println("Pos 0");
	motor.SetPosition(0);
	delay(1000);
	Serial.println("Pos 0.25");
	motor.SetPosition(0.25);
	delay(1000);
}