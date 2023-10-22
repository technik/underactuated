#pragma once
#include <cassert>
#include <iostream>
#include <numbers>
#include <math/vector.h>
#include <chrono>

using namespace std::chrono;
using namespace std::chrono_literals;
using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

// Math constants
static constexpr auto Pi = std::numbers::pi_v<double>;
static constexpr auto TwoPi = 2 * std::numbers::pi_v<double>;
constexpr long double operator "" _km(long double distance)
{
	return distance * 1000;
}

inline constexpr double RadFromDeg(double deg)
{
	return deg * (Pi / 180);
}

constexpr long double operator "" _deg(long double degrees)
{
	return RadFromDeg(degrees);
}

// Physical constants
static constexpr double G = 6.67259e-11;

// Solar system constants
static constexpr double SolarRadius = 696e6;

// Celestial body masses
static constexpr double SolarMass = 1.9884e30;
static constexpr double EarthMass = 5.9722e24;
static constexpr double MarsMass = 6.4171e23;
static constexpr double MoonMass = 7.34767309e22;
static constexpr double SolarGravitationalConstant = 1.32712440018e20;

// Mean distances from the sun

// Data sourced from https://nssdc.gsfc.nasa.gov/planetary/factsheet/fact_notes.html
// Referred to the J2000 epoch
static constexpr double EarthAphelion = 152.1e6_km;
static constexpr double EarthPerihelion = 147.095e6_km;
static constexpr double EarthSemimajorAxis = 149.598023e6_km;
static constexpr double EarthEccentricity = 0.0167086;
static constexpr double EarthMeanAnomaly = 358.617_deg;
static constexpr double EarthPeriHelArg = 114.207_deg;
static constexpr double EarthLongitudeOfAscendingNode = -11.26064_deg;
static constexpr double EarthMeanLongitude = 100.46435_deg;

static constexpr double MarsAphelion = 249.2e6_km;
static constexpr double MarsPerihelion = 206.7e6_km;
static constexpr double MarsSemimajorAxis = 227.9392e6_km;
static constexpr double MarsInclination = 1.85061_deg;
static constexpr double MarsEccentricity = 0.0934;
static constexpr double MarsPeriHelArg = 286.502_deg;
static constexpr double MarsLongitudeOfAscendingNode = 49.57854_deg;
static constexpr double MarsMeanLongitude = 355.45332_deg;

static constexpr TimePoint J2000 = sys_days(2000y/January/1);

inline double daysFromSeconds(double seconds)
{
	return seconds / (24 * 3600);
}

class CircularOrbit
{
public:
	CircularOrbit() = default;
	CircularOrbit(double radius, double mainBodyMass, double orbiterMass = 0)
		: m_radius(radius)
	{
		m_mu = G * (mainBodyMass + orbiterMass);
	}

	double radius() const
	{
		return m_radius;
	}

	// Linear speed of the orbiting body
	double velocity() const
	{
		return sqrt(m_mu / m_radius);
	}

	// Time to complete a full orbit
	double period() const
	{
		return TwoPi * m_radius * sqrt(m_radius / m_mu);
	}

	double gravitationalConstant() const { return m_mu; }

	// Expects numSegments+1 capacity in the x and y arrays
	void plot(float* x, float* y, int numSegments)
	{
		for (int i = 0; i < numSegments; ++i)
		{
			auto argument = TwoPi * i / numSegments;
			x[i] = m_radius * cos(argument);
			y[i] = m_radius * sin(argument);
		}

		// Close the orbit
		x[numSegments] = x[0];
		y[numSegments] = y[0];
	}

private:
	double m_mu; // Gravitational constant
	double m_radius;
};

// For now, it assumes all orbits lay within the ecliptic plane
class ConicOrbit
{
public:
	ConicOrbit() = default;
	ConicOrbit(
		double focalBodyGravitationalParam,
		double periapsis,
		double apoapsis,
		double inclination,
		double argumentOfPeriapsis,
		double longitudeOfAscendingNode,
		double meanLongitudeAtEpoch=0)
		: m_periapsis(periapsis)
		, m_apoapsis(apoapsis)
		, m_argumentOfPeriapsis(argumentOfPeriapsis)
		, m_longitudeOfAscendingNode(longitudeOfAscendingNode)
		, m_meanLongitudeAtEpoch(meanLongitudeAtEpoch)
		, m_mu(focalBodyGravitationalParam)
	{
		m_eccentricity = (m_apoapsis - m_periapsis) / (m_apoapsis + m_periapsis);
		m_p = m_periapsis * (1 + m_eccentricity);
		m_meanAnomalyAtEpoch = meanLongitudeAtEpoch - longitudeOfAscendingNode - argumentOfPeriapsis;
	}

	ConicOrbit(const ConicOrbit&) = default;
	ConicOrbit& operator=(const ConicOrbit&) = default;

	double radius(double anomaly) const
	{
		return m_p / (1 + m_eccentricity * cos(anomaly - m_argumentOfPeriapsis));
	}

	double speed(double anomaly) const
	{
		const auto a = semiMajorAxis();
		const auto r = radius(anomaly);
		return sqrt(2 * m_mu / r - m_mu / a);
	}

	double perihelionVelocity() const
	{
		return sqrt(2 * m_mu * (1 / m_periapsis - 1 / (m_periapsis + m_apoapsis)));
	}

	double aphelionVelocity() const
	{
		return sqrt(2 * m_mu * (1 / m_apoapsis - 1 / (m_periapsis + m_apoapsis)));
	}

	constexpr double semiMajorAxis() const
	{
		return 0.5 * (m_apoapsis + m_periapsis);
	}

	double period() const
	{
		const auto a = semiMajorAxis();
		return TwoPi * a * sqrt(a / m_mu);
	}

	constexpr double eccentricity() const
	{
		return m_eccentricity;
	}

	constexpr static double meanRadius(double perihelion, double eccentricity)
	{
		return perihelion * (1 + eccentricity);
	}

	math::Vec2d position(double argument) const
	{
		auto r = radius(argument);
		return { cos(argument) * r, sin(argument) * r };
	}

	// Expects numSegments+1 capacity in the x and y arrays
	void plot(float* x, float* y, int numSegments, float tmin = 0, float tmax = 1) const
	{
		assert(isElliptical()); // Plotting open trajectories is not supported.

		for (int i = 0; i < numSegments+1; ++i)
		{
			auto argument = (TwoPi * i / numSegments) * tmax;
			auto r = radius(argument);
			x[i] = r * cos(argument);
			y[i] = r * sin(argument);
		}
	}

	constexpr bool isElliptical() const { return m_eccentricity < 1; }
	constexpr bool isParabolical() const { return m_eccentricity == 1; }
	constexpr bool isHyperbolical() const { return m_eccentricity > 1; }

	double TrueAnomalyFromMeanAnomaly(double meanAnomaly)
	{
		const double timeSincePeriapsis = meanAnomaly * period() / TwoPi;
	}

	double TrueAnomalyFromMeanLongitude(double meanLongitude, double longitudeOfPeriapsis)
	{
		const double meanLongitudeSinceAscendingNode = meanLongitude - longitudeOfPeriapsis;
		const double meanAnomalySinceAscendingNode = meanLongitudeSinceAscendingNode; // "Time" since last periapsis at epoch
	}

	double MeanAnomaly(TimePoint time) const
	{
		assert(time >= J2000);

		// Time since epoch
		const auto timeSinceEpoch = duration_cast<duration<double, seconds::period>>(time - J2000).count();
		const auto numOrbits = timeSinceEpoch / period() + m_meanAnomalyAtEpoch / TwoPi + 1; // Make sure we're positive

		// Time since the start of last orbit
		const auto meanAnomaly = TwoPi * (numOrbits - std::floor(numOrbits));

		return meanAnomaly;
	}

	double TrueAnomaly(TimePoint time) const
	{
		const auto meanAnomaly = MeanAnomaly(time);

		return TrueAnomalyFromMeanAnomaly(meanAnomaly);
	}

	double TrueAnomalyFromMeanAnomaly(double M) const
	{
		assert(M >= -Pi && M <= TwoPi);

		// Center anomaly range around 0 to leverage symmetry
		if (M >= Pi)
			M -= TwoPi;

		// Using a fourier expansion (should only be used with small eccentricities).
		const double e = m_eccentricity;
		const double e2 = e * e;
		const double e3 = e2 * e;
		return M + (2 * e + e3 / 4) * sin(M) + 5 * e2 / 4 * sin(2 * M) + 13 * e3 / 12 * sin(3 * M);

		// Note: If higher precision is needed for highly elliptic orbits, we can either implement
		// the newton-raphson method or generate a lookup table of eccentric anomalies for binary search.
	}

private:
	double m_periapsis = 1;
	double m_apoapsis = 1;
	double m_argumentOfPeriapsis = 0;
	double m_longitudeOfAscendingNode = 0;
	double m_meanLongitudeAtEpoch = 0;

	// Precomputed
	double m_meanAnomalyAtEpoch = 0;

	// Constants
	double m_mu = 1;
	double m_eccentricity = 1; // Orbital eccentricity
	double m_p = 1; // Orbital parameter
};

using EllipticalOrbit = ConicOrbit;
using ParabolicalOrbit = ConicOrbit;
using HyperbolicalOrbit = ConicOrbit;

static const EllipticalOrbit EarthOrbit(
	G*(SolarMass + EarthMass),
	EarthPerihelion, EarthAphelion,
	0.0_deg,
	EarthPeriHelArg,
	EarthLongitudeOfAscendingNode,
	EarthMeanLongitude);

static const EllipticalOrbit MarsOrbit(
	G* (SolarMass + MarsMass),
	MarsPerihelion, MarsAphelion,
	MarsInclination,
	MarsPeriHelArg,
	MarsLongitudeOfAscendingNode,
	MarsMeanLongitude);

inline double sphereOfInfluenceRadius(float orbiterMass, float perihelion, float aphelion)
{
	const auto semimajorAxis = perihelion + aphelion;
	return semimajorAxis * pow(orbiterMass / SolarMass, 2 / 5.0);
}