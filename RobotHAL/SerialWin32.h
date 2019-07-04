#pragma once

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <cstdint>

class SerialWin32 {
public:
	unsigned	write(const void* _src, unsigned _nBytes);
	bool		write(uint8_t);

	unsigned	read(void* _dst, unsigned _nBytes);
	uint8_t		read();


protected:
	SerialWin32(const char* _port, unsigned _baudRate);

private:
	void openPort(const char* _port);
	void setBaudRate(unsigned _baudRate);

private:
	HANDLE		mPortHandle;
};

typedef SerialWin32 SerialBase;

#endif // _WIN32