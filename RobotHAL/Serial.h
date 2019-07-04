// Simple serial port communications
#pragma once

#include <cstdint>
#include "SerialWin32.h"

class Serial : public SerialBase {
public:
	Serial(const char* _port, unsigned _baudRate) : SerialBase(_port, _baudRate) {}
};
