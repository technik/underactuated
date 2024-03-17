// Molecular dynamics playground

#include "imgui.h"
#include "implot.h"
#include <cmath>
#include "app.h"
#include <math/vector.h>
#include <math/matrix.h>
#include <numbers>
#include <random>

static constexpr auto Pi = std::numbers::pi_v<double>;
static constexpr auto TwoPi = 2 * std::numbers::pi_v<double>;

using namespace math;

namespace // Unnamed drawing utilities
{
    // Auxiliary drawing
    template<int numSegments>
    static void plotCircle(const char* name, float x0, float y0, float radius)
    {
        static_assert(numSegments > 1);
        float x[numSegments + 1];
        float y[numSegments + 1];
        for (int i = 0; i < numSegments + 1; ++i)
        {
            auto theta = i * TwoPi / numSegments;
            x[i] = radius * cos(theta) + x0;
            y[i] = radius * sin(theta) + y0;
        }
        ImPlot::PlotLine(name, x, y, numSegments + 1);
    }

    static void plotLine(const char* name, const Vec2d& a, const Vec2d& b)
    {
        float x[2] = { float(a.x()), float(b.x()) };
        float y[2] = { float(a.y()), float(b.y()) };

        ImPlot::PlotLine(name, x, y, 2);
    }
}

namespace // Math utils
{
    static constexpr auto g = 9.81;

    static int squirrelNoise(int position, int seed = 0)
    {
        constexpr unsigned int BIT_NOISE1 = 0xB5297A4D;
        constexpr unsigned int BIT_NOISE2 = 0x68E31DA4;
        constexpr unsigned int BIT_NOISE3 = 0x1B56C4E9;

        int mangled = position;
        mangled *= BIT_NOISE1;
        mangled += seed;
        mangled ^= (mangled >> 8);
        mangled *= BIT_NOISE2;
        mangled ^= (mangled << 8);
        mangled *= BIT_NOISE3;
        mangled ^= (mangled >> 8);
        return mangled;
    }

    struct SquirrelRng
    {
        int rand()
        {
            return squirrelNoise(state++);
        }

        float uniform()
        {
            auto i = rand();
            return float(i & ((1 << 24) - 1)) / (1 << 24);
        }

        int state = 0;
    };
}

struct DifferentialCart
{
    struct Params
    {
        double maxWheelVel = 2;
        double axisLen = 0.2;
    } m_params;

    struct State
    {
        double vRight = 0;
        double vLeft = 0;
        Vec2d pos = {};
        double orient = 0;
    } m_state;

    struct Input
    {
        double dvRight = 0;
        double dvLeft = 0;
    };

    Input updateController(SquirrelRng& rng)
    {
        Input action;
        action.dvLeft = rng.uniform() - 0.5;
        action.dvRight = rng.uniform() - 0.5;
        return action;
    }

    void step(double dt, const Input& action)
    {
        // Basic Euler integration
        double meanVel = 0.5 * (m_state.vRight + m_state.vLeft);
        double diffVel = (m_state.vRight - m_state.vLeft) / m_params.axisLen;
        double ct = cos(m_state.orient);
        double st = sin(m_state.orient);
        m_state.pos += (dt * meanVel) * Vec2d(ct, st);
        m_state.orient += dt * diffVel;

        // Apply the action
        m_state.vRight = max(-m_params.maxWheelVel, min(m_params.maxWheelVel, m_state.vRight + action.dvRight));
        m_state.vLeft = max(-m_params.maxWheelVel, min(m_params.maxWheelVel, m_state.vLeft + action.dvLeft));
    }

    void draw()
    {
        float radius = float(m_params.axisLen * 0.5);
        plotCircle<20>("bot", m_state.pos.x(), m_state.pos.y(), radius);
        Vec2d lookAt = m_state.pos + radius * Vec2d(cos(m_state.orient), sin(m_state.orient));
        plotLine("botDir", m_state.pos, lookAt);
    }
};

struct LinearTrack
{
    double length = 10;
    double width = 1;

    void draw()
    {
        const auto hLen = length * 0.5;
        const auto hWid = width * 0.5;

        plotLine("top", Vec2d(-hLen, hWid), Vec2d(hLen, hWid));
        plotLine("bottom", Vec2d(-hLen, -hWid), Vec2d(hLen, -hWid));
    }

    bool isValidPos(const Vec2d& pos) const
    {
        return abs(pos.y()) <= 0.5 * width && pos.x() >= -0.5 * length;
    }

    bool isGoal(const Vec2d& pos) const
    {
        return isValidPos(pos) && pos.x() > 0.5 * length;
    }
};

class CartApp : public App
{
public:
    DifferentialCart cart;
    LinearTrack testTrack;

    CartApp()
    {
        randomizeStart();
        cart.m_state.pos.y() = 0; // Start at the center pos first
    }

    void randomizeStart()
    {
        cart.m_state.pos.x() = -testTrack.length * 0.5;
        cart.m_state.pos.y() = -testTrack.width * 0.5 + testTrack.width * m_rng.uniform();
        cart.m_state.orient = 0;// (-0.5 + m_rng.uniform()) * 3.1415927;
        cart.m_state.vLeft = 0;
        cart.m_state.vRight = 0;
    }

    void update() override
    {
        // Plot params
        if (ImGui::CollapsingHeader("Params"))
        {
            ImGui::InputDouble("Axis Len", &cart.m_params.axisLen);
            ImGui::InputDouble("Max vel", &cart.m_params.maxWheelVel);

            if (ImGui::Button("Generate"))
            {
                cart.m_params.axisLen = 0.1 + 0.4 * m_rng.uniform();
            }
        }

        // Plot state
        if(ImGui::CollapsingHeader("State"))
        {
            ImGui::InputDouble("Left", &cart.m_state.vLeft);
            ImGui::InputDouble("Right", &cart.m_state.vRight);
            if (ImGui::Button("Randomize start"))
            {
                randomizeStart();
            }
        }

        // Run simulation
        ImGui::Checkbox("Run", &m_isRunningSimulation);
        if (m_isRunningSimulation)
        {
            advanceSimulation();
        }

        // Display results
        if(ImGui::Begin("Simulation"))
        {
            float size = float(0.55 * testTrack.length);
            if(ImPlot::BeginPlot("Cart", ImVec2(-1, -1), ImPlotFlags_Equal))
            {
                // Set up rigid axes
                ImPlot::SetupAxis(ImAxis_X1, NULL, ImPlotAxisFlags_AuxDefault);
                ImPlot::SetupAxisLimits(ImAxis_X1, -size, size, ImGuiCond_Always);
                ImPlot::SetupAxis(ImAxis_Y1, NULL, ImPlotAxisFlags_AuxDefault);

                plotState();
            }
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

private:
    bool m_isRunningSimulation = false;
    double m_accumTime = 0;
    double m_stepDt = 0.001;

    void advanceSimulation()
    {
        m_accumTime += 1 / 60.0;
        while (m_accumTime > m_stepDt)
        {
            m_accumTime -= m_stepDt;

            stepSimulation();
        }
    }

    void stepSimulation()
    {
        auto action = cart.updateController(m_rng);
        cart.step(m_stepDt, action);
        if (!testTrack.isValidPos(cart.m_state.pos) || testTrack.isGoal(cart.m_state.pos))
        {
            randomizeStart();
        }
    }
    
    SquirrelRng m_rng;

    void plotState()
    {
        testTrack.draw();
        cart.draw();
    }
};

// Main code
int main(int, char**)
{
    CartApp app;
    if (!app.init())
        return -1;

    // Main loop
    bool done = false;
    while (!done)
    {
        // Poll and handle messages (inputs, window resize, etc.)
        // See the WndProc() function below for our to dispatch events to the Win32 backend.
        MSG msg;
        while (::PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE))
        {
            ::TranslateMessage(&msg);
            ::DispatchMessage(&msg);
            if (msg.message == WM_QUIT)
                done = true;
        }
        if (done)
            break;

        app.beginFrame();
        app.update();
        app.render();
    }

    app.end();

    return 0;
}
