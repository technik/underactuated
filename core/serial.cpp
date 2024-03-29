#if defined(__linux__)

extern "C" {
	#include <fcntl.h>
	#include <termios.h>
	#include <unistd.h>
}

#include <iostream>
#include <cstring>
#include <cassert>

#include "serial.h"

//------------------------------------------------------------------------------------------------------------------
SerialLinux::SerialLinux(const char* _port, unsigned _baudRate)
{
	// Enforce correct data
	assert(nullptr != _port && '\0' != _port[0]);

	openPortFile(_port);

	// Gett addess info
	mPortConfig = new termios;
	memset (mPortConfig, 0, sizeof(struct termios)); // Clear memory

	if ( tcgetattr ( mFileDesc, mPortConfig) != 0 ) // Get port address
	{
		std::cout << "Error: Unable to open serial port " << _port << "\n";
	}
	setBaudRate(mPortConfig, _baudRate, _port);

	mPortConfig->c_cflag &= ~PARENB;    // Set no parity, no stop bits. Byte size = 8
	mPortConfig->c_cflag &= ~CSTOPB;
	mPortConfig->c_cflag &= ~CSIZE;
	mPortConfig->c_cflag |= CS8; // 8-bit frame size
	mPortConfig->c_cflag |= CREAD; // Enable reading
	mPortConfig->c_cc[VMIN] = 1; // Always read at least one character
	mPortConfig->c_cc[VTIME] = 0; // Disable time-out?

	cfmakeraw(mPortConfig); // Apply configuration;
	tcflush( mFileDesc, TCIFLUSH );	// Flush port to set atributes

	if ( tcsetattr ( mFileDesc, TCSANOW, mPortConfig ) != 0) // Set attributes
	std::cout << "Error: Unable to set serial port attributes\n";

	clearInputBuffer();
}

//------------------------------------------------------------------------------------------------------------------
void SerialLinux::setBlocking(bool _blocking)
{
	memset (mPortConfig, 0, sizeof(struct termios));
	if (tcgetattr (mFileDesc, mPortConfig) != 0)
	{
		std::cout << "error " << errno << " getting term settings set_blocking\n";
		return;
	}

	mPortConfig->c_cc[VMIN]  = _blocking ? 1 : 0;
	mPortConfig->c_cc[VTIME] = _blocking ? 5 : 0; // 0.5 seconds read timeout

	if (tcsetattr (mFileDesc, TCSANOW, mPortConfig) != 0)
	{
		std::cout << "error setting term " << (_blocking ? "" : "not-") << "sblocking\n";
	}
}

//------------------------------------------------------------------------------------------------------------------
void SerialLinux::clearInputBuffer()
{
	setBlocking(false);
	char buf [10000];
	int n;
	do {
		n = ::read (mFileDesc, buf, sizeof(buf));
	} while (n > 0);
	setBlocking(true);
}

//------------------------------------------------------------------------------------------------------------------
unsigned SerialLinux::write(const void* _src, unsigned _nBytes)
{
	// Enforce correct input data
	assert(nullptr != _src);

	int writtenBytes = ::write( mFileDesc, _src, _nBytes);

	assert(-1 != writtenBytes);

	return writtenBytes;
}

//------------------------------------------------------------------------------------------------------------------
bool SerialLinux::write(uint8_t _data)
{
	return -1 != ::write( mFileDesc, &_data, 1);
}


//------------------------------------------------------------------------------------------------------------------
unsigned SerialLinux::read(void * _dst, unsigned _nBytes)
{
	//Enforce correct output data
	assert(nullptr != _dst);

	int readBytes = ::read(mFileDesc, _dst , _nBytes);

	if(readBytes < 0)
	{
		std::cout << "Error: reading from serial port failed\n";
	}

	return readBytes;
}

//------------------------------------------------------------------------------------------------------------------
uint8_t SerialLinux::read()
{
	uint8_t data;
	unsigned nBytesRead = ::read(mFileDesc, &data , 1);
	if(nBytesRead < 0)
	{
		std::cout << "Error: reading from serial port failed\n";
	}

	assert(nBytesRead == 1);
	return data;
}

//------------------------------------------------------------------------------------------------------------------
void SerialLinux::openPortFile(const char* _port)
{
	mFileDesc = open(
		_port, // Port name
		O_RDWR ); // Read and write

	if(mFileDesc <= 0)
	{
		std::cout << "Error: Unable to access file << " << _port << std::endl;
	}
}

//------------------------------------------------------------------------------------------------------------------
void SerialLinux::setBaudRate(struct termios* _serialPortInfo, unsigned _baudRate, const char* _port)
{
	// Actually select address
	speed_t commBaudRate;
	switch (_baudRate) {
		case 4800:
			commBaudRate = (speed_t)B4800;
			break;
		case 9600:
			commBaudRate = (speed_t)B9600;
			break;
		case 19200:
			commBaudRate = (speed_t)B19200;
			break;
		case 38400:
			commBaudRate = (speed_t)B38400;
			break;
		case 57600:
			commBaudRate = (speed_t)B57600;
			break;
		case 115200:
			commBaudRate = (speed_t)B115200;
			break;
		default:
			commBaudRate = (speed_t)B115200;
			assert(false); // Unsupported baudrate
			return;
	}

	cfsetospeed (_serialPortInfo, commBaudRate);
	cfsetispeed (_serialPortInfo, commBaudRate);
}

#endif // __linux__

#if _WIN32

#include <cassert>
#include "serial.h"

//------------------------------------------------------------------------------------------------------------------
SerialWin32::SerialWin32(const char* _port, unsigned _baudRate){
	// Enforce correct data
	assert(nullptr != _port && '\0' != _port[0]);

	openPort(_port);
	setBaudRate(_baudRate);
}

//------------------------------------------------------------------------------------------------------------------
unsigned SerialWin32::write(const void* _src, unsigned _nBytes) {
	// Enforce correct input data
	assert(nullptr != _src);

	DWORD writtenBytes;
	::WriteFile(mPortHandle, _src, _nBytes, &writtenBytes, NULL);
	return writtenBytes;
}

//------------------------------------------------------------------------------------------------------------------
unsigned SerialWin32::read(void * _dstBuffer, unsigned _nBytes)
{
	DWORD readBytes;
	::ReadFile(mPortHandle, _dstBuffer, _nBytes, &readBytes, NULL);
	return readBytes;

}

//------------------------------------------------------------------------------------------------------------------
uint8_t SerialWin32::read()
{
	uint8_t result;
	DWORD readBytes;
	::ReadFile(mPortHandle, &result, 1, &readBytes, NULL);
	return result;
}
//------------------------------------------------------------------------------------------------------------------
bool SerialWin32::write(uint8_t _data) {
	DWORD writtenBytes;
	::WriteFile(mPortHandle, &_data, 1, &writtenBytes, NULL);
	return (0 != writtenBytes);
}

//------------------------------------------------------------------------------------------------------------------
void SerialWin32::openPort(const char* _port) {	
	mPortHandle = ::CreateFileA(_port,
		GENERIC_READ|GENERIC_WRITE,
		0, // Don't share the port
		0, // No security attributes
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0); // No templates
	assert(INVALID_HANDLE_VALUE != mPortHandle);
}

//------------------------------------------------------------------------------------------------------------------
void SerialWin32::setBaudRate(unsigned _baudRate)
{
	DCB dcb = {0};
	dcb.DCBlength = sizeof(DCB);
	if (!::GetCommState (mPortHandle,&dcb))
		assert(false);
	switch(_baudRate) {
		case 4800:	dcb.BaudRate    = CBR_4800;		break;
		case 9600:	dcb.BaudRate    = CBR_9600;		break;
		case 19200:	dcb.BaudRate    = CBR_19200;	break;
		case 57600:	dcb.BaudRate    = CBR_57600;	break;
		case 115200:dcb.BaudRate    = CBR_115200;	break;
		case 128000:dcb.BaudRate    = CBR_128000;	break;
		case 256000:dcb.BaudRate    = CBR_256000;	break;
		default:
			assert(false); // Unsupported baudrate
			return;
	}
	dcb.ByteSize = 8;
	dcb.fParity = FALSE;
	dcb.Parity = 0;
	dcb.StopBits = 0;

	if (!::SetCommState (mPortHandle,&dcb))
		assert(false);
}

#endif // _WIN32