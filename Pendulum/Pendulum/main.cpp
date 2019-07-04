#include <iostream>
#include <fstream>
#include <cassert>

#include <Eigen/Eigen>

template<class T>
constexpr size_t simulationSteps(T duration, T dt)
{
	size_t n = size_t(duration / dt);
	if (n*dt < duration)
		n++;
	return n;
}

template<class T, class X, class Model, class U, class Controller>
void simulate(size_t nSteps, T dt, const X& x0, Model& f, Controller& control, X* x, U*u)
{
	assert(x);
	assert(u);
	x[0] = x0;

	for (size_t i = 0; i < nSteps; ++i)
	{
		auto t = i * dt;
		u[i] = control(x[i], t);
		X dx = f(x[i], u[i]);
		x[i+1] = x[i] + dx * dt; // Forward Euler integration
	}
}

using Vec2 = Eigen::Vector2f;

int main()
{
	constexpr float simDuration = 10.f;
	constexpr float dt = 0.1f;

	constexpr auto nSteps = simulationSteps(simDuration, dt);
	Vec2 X[nSteps + 1];
	float U[nSteps];
	Vec2 x0 = { 1.5f, 0.f };

	constexpr float g = 9.81f;
	constexpr float m = 1.f;
	constexpr float l = 0.4f;
	constexpr float b = 0.05f;

	auto model = [=](auto x, auto u) {
		assert(m > 0);
		auto theta = x(0);
		auto w = x(1);
		return Vec2(w, -g*l*sin(theta) - (b/m) * w + u);
	};
	auto controller = [](auto, auto) {
		return 0.f;
	};

	simulate(
		nSteps, dt, x0,
		model,
		controller,
		X, U);

	// Serialize results
	std::ofstream out("out.csv");
	for (int i = 0; i < nSteps; ++i)
	{
		out << dt * i;
		for (size_t j = 0; j < x0.rows(); ++j)
			out << "," << X[i](j);
		out << "," << U[i];
		out << "\n";
	}
	return 0;
}