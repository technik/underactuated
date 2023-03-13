#pragma once

#include <cstdint>

#if defined(__linux__)

#include <cstdint>
extern "C" {
#include <termios.h>
}

class SerialLinux {
public:
	unsigned	write		(const void* _src, unsigned _nBytes);
	bool		write		(uint8_t);

	unsigned	read		(void * _dst, unsigned _nBytes); // Returns the amount of bytes read
	uint8_t		read		(); // Reads one byte

protected:
	SerialLinux						(const char* _port, unsigned _baudRate);
	void		setBlocking			(bool);
	void		clearInputBuffer	();
	void		openPortFile		(const char* _port);
	void		setBaudRate			(struct termios* _serialPortInfo, unsigned _baudRate, const char* _port);
	
private:
	int				mFileDesc;
	struct termios*	mPortConfig;
};

typedef SerialLinux SerialBase;

#endif // __linux__

#ifdef _WIN32
#include <Windows.h>
#include <cstdint>


class SerialWin32 {
	public:
		unsigned	write	(const void* _src, unsigned _nBytes);
		bool		write	(uint8_t);

		unsigned	read	(void * _dst, unsigned _nBytes); 
		uint8_t		read	(); 


	protected:
		SerialWin32			(const char* _port, unsigned _baudRate);

	private:
		void openPort		(const char* _port);
		void setBaudRate	(unsigned _baudRate);
		
	private:
		HANDLE		mPortHandle;
	};

	typedef SerialWin32 SerialBase;
#endif // _WIN32

class SerialPort : public SerialBase
{
public:
	SerialPort(const char* _port, unsigned _baudRate) : SerialBase(_port, _baudRate){}
};
