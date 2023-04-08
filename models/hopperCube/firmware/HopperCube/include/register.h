#pragma once

#include <cstdint>

template<class RegT_, std::uint16_t location_>
class RegisterBase {
public:
	static constexpr std::uint16_t	location = location_; // Used for relative register referencing
	typedef RegT_					PointedT;
	
	static void set(RegT_ _x) 	{ *reinterpret_cast<volatile RegT_*>(location_) = _x; }
	static volatile RegT_& get() 		{ return *reinterpret_cast<volatile RegT_*>(location_); }
	void	operator=					(RegT_ _r)			{ *reinterpret_cast<volatile RegT_*>(location_) = _r; }
			operator RegT_				()			const	{ return *reinterpret_cast<volatile RegT_*>(location_); }
			operator volatile RegT_&	()					{ return *reinterpret_cast<volatile RegT_*>(location_); }

protected:
	RegisterBase() = default; // This prevents the class to be instantiated without deriving
};

// Declaring the template instead of defining it will
// yield compilation errors in case anyone tried to use it with non-specialied types
template<class, std::uint16_t> class Register;

// 8-bits register
template<std::uint16_t location_>
class Register<std::uint8_t, location_> : public RegisterBase<uint8_t, location_>
{
public:
	using RegisterBase<uint8_t, location_>::operator=;
};

// 16-bits register
template<std::uint16_t location_>
class Register<std::uint16_t, location_> : public RegisterBase<uint16_t, location_>
{
public:
	using RegisterBase<uint16_t, location_>::operator=;

	typedef Register<uint8_t, location_>	Low;
	typedef Register<uint8_t, location_+1>	High;
};

template<typename PORT_>
struct GPIOPort final
{
public:
	typedef PORT_													PORT;
	typedef Register<typename PORT_::PointedT, PORT_::location-1>	DDR;
	typedef Register<typename PORT_::PointedT, PORT_::location-2>	PIN;
public:
	DDR		ddr;
	PIN		pin;
	PORT	port;
};

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega2560__)

using RegPINB = Register<std::uint8_t, 0x23>;
using RegDDRB = Register<std::uint8_t, 0x24>;
using RegPORTB = Register<std::uint8_t, 0x25>;
using RegPINC = Register<std::uint8_t, 0x26>;
using RegDDRC = Register<std::uint8_t, 0x27>;
using RegPORTC = Register<std::uint8_t, 0x28>;
using RegPIND = Register<std::uint8_t, 0x29>;
using RegDDRD = Register<std::uint8_t, 0x2a>;
using RegPORTD = Register<std::uint8_t, 0x2b>;

#endif // defined (__AVR_ATmega328P__) || defined (__AVR_ATmega2560__)