#pragma once

#include <cstdint>

template<int PinA, int PinB>
class HBridge
{
public:
	HBridge()
	{
		pinMode(PinA, OUTPUT);
		digitalWrite(PinA, LOW);
		pinMode(PinB, OUTPUT);
		digitalWrite(PinB, LOW);
	}

	void Disable()
	{
		digitalWrite(PinA, LOW);
		digitalWrite(PinB, LOW);
	}

	void Write(int16_t speed)
	{
		if(speed >= 0)
		{
			speed = min(255,speed);
			analogWrite(PinA, speed);
			analogWrite(PinB, 0);
		}
		else
		{
			speed = min(255, -speed);
			analogWrite(PinA, 0);
			analogWrite(PinB, speed);
		}
	}
};

template<
int OutPin1, int OutPin2,
int OutPin3, int OutPin4>
class L298
{
public:
	using ChannelA = HBridge<OutPin1, OutPin2>;
	using ChannelB = HBridge<OutPin3, OutPin4>;

	// 
};