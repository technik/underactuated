#pragma once

#include <Arduino.h>
#include <pin.h>

// Several low level drivers for BLDC motors
template<class dirPin, int enablePin>
struct BLDCAnalogChannel
{
    static constexpr int EnablePin = enablePin;
    using DirPin = dirPin;

    void Disable()
    {
        digitalWrite(enablePin, LOW);
    }

    void Set(int16_t duty)
    {
        if(duty == 0)
        {
            digitalWrite(enablePin, 0);
        }
        else if(duty > 0)
        {
            dirPin::setHigh();
        }
        else
        {
            dirPin::setLow();
        }
        analogWrite(enablePin, abs(duty));
    }
};

template<class ChannelA, class ChannelB, class ChannelC>
struct TrapezoidalController
{
    ChannelA m_channelA;
    ChannelB m_channelB;
    ChannelC m_channelC;
    
    void Init()
    {
        Disable();
    }

    void SetAngle(float t, int16_t speed)
    {
        // Clear all channels to avoid spikes
        Disable();

        // Find the correct step
        int step = int(t * 6) % 6; // Allow up to 1 negative turn
        if(step < 0)
            step += 6;

        auto config = steps[step];
        m_channelA.Set(config & 2 ? (config & 1 ? speed : -speed) : 0);
        m_channelB.Set(config & 8 ? (config & 4 ? speed : -speed) : 0);
        m_channelC.Set(config & 32 ? (config & 16 ? speed : -speed) : 0);
    }

    void Disable()
    {
        m_channelA.Disable();
        m_channelB.Disable();
        m_channelC.Disable();
    }

    inline static constexpr uint8_t steps[6] = 
    { // For each channel (enable, dir)
        0b100011,
        0b101100,
        0b001110,
        0b110010,
        0b111000,
        0b001011
    };

    uint8_t m_nextStep;
};

// Designed to drive a single BLDC motor with 3 L298 channels (1.5 ICs),
// All connected to a signle port with a pin for dir and a pin for enable.
template<class Port> // Port must be a GPIO port
class SinglePortController
{
public:
	using DDR = typename Port::DDR;
	using PORT = typename Port::PORT;
	// Low level control API
	void Init()
    {
		// Set all pins as output.
		// Pins 0-5 for BLDC control, plus pins 6,7 for protection.
		// Pin 6 can be used to debug the cycle
		DDR::set(0b11111111);
        Disable();
    }

    void SetAngle(float t, int16_t speed)
	{
		// Find the correct step
        int step = int(t * 6) % 6; // Allow up to 1 negative turn
        if(step < 0)
            step += 6;

        auto config = kConfigs[step];
		PORT::set(config | (PORT::get() & 0b11000000));

		// Simulated PWM
		speed = min(255, abs(speed));
		delayMicroseconds(speed);
		Disable();
		delayMicroseconds(260 - speed);
	}

    void Disable()
	{
		PORT::set(0); // Clear all pins
	}

	inline static constexpr uint8_t kConfigs[6] = 
    { // For each channel (enable, dir)
        0b100011,
        0b101100,
        0b001110,
        0b110010,
        0b111000,
        0b001011
    };
};