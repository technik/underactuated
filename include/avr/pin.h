#pragma once

#include <cstdint>
#include <register.h>

//--------------------------------------------------------------------------
template<typename Port_, std::uint8_t bit_>
class Pin {
public:
	static void setOutput	();
	static void setLow 		();
	static void setHigh		();
	static void toggle		();
	
	static void setInput	();
	static bool isLow		();
	static bool isHigh		();
	
private:
	typedef typename GPIOPort<Port_>::PORT	PORT;
	typedef typename GPIOPort<Port_>::PIN	PIN;
	typedef typename GPIOPort<Port_>::DDR	DDR;

	static constexpr std::uint8_t bit = bit_;
	static constexpr std::uint8_t bitMask = 1<<bit;

	static PORT	sPort;
	static PIN	sPin;
	static DDR	sDdr;
};

template<class Pin>
class OutputPin
{
public:
	OutputPin()
	{
		Pin::setOutput();
	}

	void setHigh() { Pin::setHigh(); }
	void setLow() { Pin::setLow(); }
	void toggle() { Pin::toggle(); }
};

// Inline implementation
//--------------------------------------------------------------------------
template<typename Port_, std::uint8_t bit_>
inline void Pin<Port_,bit_>::setOutput() { sDdr.get() |= bitMask; }
//--------------------------------------------------------------------------
template<typename Port_, std::uint8_t bit_>
inline void Pin<Port_,bit_>::setLow() { sPort.get() &= ~bitMask; }
//--------------------------------------------------------------------------
template<typename Port_, std::uint8_t bit_>
inline void Pin<Port_,bit_>::setHigh() { sPort.get() |= bitMask; }
//--------------------------------------------------------------------------
template<typename Port_, std::uint8_t bit_>
inline void Pin<Port_,bit_>::toggle() { sPin.get() = bitMask; }
//--------------------------------------------------------------------------
template<typename Port_, std::uint8_t bit_>
inline void Pin<Port_,bit_>::setInput() { sDdr.get() &= ~bitMask; }
//--------------------------------------------------------------------------
template<typename Port_, std::uint8_t bit_>
inline bool Pin<Port_,bit_>::isLow() { return !(sPin.get() & bitMask); }
//--------------------------------------------------------------------------
template<typename Port_, std::uint8_t bit_>
inline bool Pin<Port_,bit_>::isHigh() { return sPin.get() & bitMask; }


#if defined (__AVR_ATmega328P__)
// Analog pins
typedef Pin<RegPORTC,0>	PinA0;
typedef Pin<RegPORTC,1>	PinA1;
typedef Pin<RegPORTC,2>	PinA2;
typedef Pin<RegPORTC,3>	PinA3;
typedef Pin<RegPORTC,4>	PinA4;
typedef Pin<RegPORTC,5>	PinA5;
// Digital pins
typedef Pin<RegPORTD,0>	Pin0;
typedef Pin<RegPORTD,1>	Pin1;
typedef Pin<RegPORTD,2>	Pin2;
typedef Pin<RegPORTD,3>	Pin3;
typedef Pin<RegPORTD,4>	Pin4;
typedef Pin<RegPORTD,5>	Pin5;
typedef Pin<RegPORTD,6>	Pin6;
typedef Pin<RegPORTD,7>	Pin7;
typedef Pin<RegPORTB,0>	Pin8;
typedef Pin<RegPORTB,1>	Pin9;
typedef Pin<RegPORTB,2>	Pin10;
typedef Pin<RegPORTB,3>	Pin11;
typedef Pin<RegPORTB,4>	Pin12;
typedef Pin<RegPORTB,5>	Pin13;

typedef Pin13 LedPin; // direct to led's pin.
#endif // __AVR_ATmega328p__