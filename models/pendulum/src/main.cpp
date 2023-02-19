// Molecular dynamics playground

#include "imgui.h"
#include "implot.h"
#include <cmath>
#include "app.h"
#include <math/vector.h>
#include <numbers>
#include <random>

static constexpr auto Pi = std::numbers::pi_v<double>;
static constexpr auto TwoPi = 2 * std::numbers::pi_v<double>;

using namespace math;

class PendulumApp : public App
{
public:
    PendulumApp()
    {
    }

    void update() override
    {
        // Plot params
        if (ImGui::CollapsingHeader("Params"))
        {
            bool inertiaChanged = false;
            inertiaChanged = ImGui::InputDouble("Mass", &m_pendulumParams.m1);
            inertiaChanged |= ImGui::InputDouble("Length", &m_pendulumParams.l1);
            if(inertiaChanged)
            {
                m_pendulumParams.refreshInertia();
            }
            ImGui::InputDouble("Friction", &m_pendulumParams.b1);
            ImGui::InputDouble("Torque Limit", &m_pendulumParams.MaxQ);
            if (ImGui::Button("Generate"))
            {
                m_pendulumParams.l1 = m_rng.uniform() * 10;
                m_pendulumParams.m1 = m_rng.uniform() * 10;
                m_pendulumParams.b1 = m_rng.uniform() * 10;
                m_pendulumParams.refreshInertia();
            }
        }

        // Plot state
        if(ImGui::CollapsingHeader("State"))
        {
            ImGui::InputDouble("Theta", &m_pendulumState.theta);
            ImGui::InputDouble("dTheta", &m_pendulumState.dTheta);
            if (ImGui::Button("Randomize"))
            {
                m_pendulumState.theta = m_rng.uniform() * 2 * 3.1415927;
            }
            if (ImGui::Button("Perturbate"))
            {
                m_pendulumState.dTheta += m_rng.uniform() - 0.5;
            }
        }

        // Plot state
        if (ImGui::CollapsingHeader("Control"))
        {
            bool control = m_control == ControlMode::EnergyPump;
            ImGui::Checkbox("Energy Pump", &control);
            m_control = control ? ControlMode::EnergyPump : ControlMode::Free;

            ImGui::InputDouble("Gain", &m_energyGain);
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
            float size = float(1.1 * m_pendulumParams.l1);
            if(ImPlot::BeginPlot("Pendulum", ImVec2(-1, -1), ImPlotFlags_Equal))
            {
                // Set up rigid axes
                ImPlot::SetupAxis(ImAxis_X1, NULL, ImPlotAxisFlags_AuxDefault);
                ImPlot::SetupAxisLimits(ImAxis_X1, -size, size, ImGuiCond_Always);
                ImPlot::SetupAxis(ImAxis_Y1, NULL, ImPlotAxisFlags_AuxDefault);

                plotPendulum();
            }
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

private:
    struct PendulumParams
    {
        double l1 = 1; // Bar lengths
        double m1 = 1; // Bar masses
        double b1 = 0; // Friction at the joints
        double I1 = 1; // Inertia tensors
        double MaxQ = 0; // Torque limit. 0 means unlimited torque

        void refreshInertia()
        {
            // Inertia concentrated at the end
            I1 = m1 * l1 * l1 / 3;
        }
    } m_pendulumParams;

    struct PendulumState
    {
        double theta = 0;
        double dTheta = 0;
    } m_pendulumState;

    enum class ControlMode
    {
        Free,
        EnergyPump
    };

    ControlMode m_control = ControlMode::Free;

    bool m_isRunningSimulation = false;
    double m_accumTime = 0;
    double m_stepDt = 0.001;
    
    double m_energyGain = 1;

    void advanceSimulation()
    {
        m_accumTime += 1 / 60.0;
        while (m_accumTime > m_stepDt)
        {
            m_accumTime -= m_stepDt;

            stepSimulation();
        }
    }

    double computeControllerInput()
    {
        if (m_control == ControlMode::Free)
        {
            return 0;
        }
        else
        {
            auto& x = m_pendulumState;
            auto& p = m_pendulumParams;
            // Target energy to stay still at the top:
            auto mgl = p.m1 * g * p.l1;
            auto Egoal = mgl;

            // Current energy
            auto T = 0.5 * p.m1 * p.l1 * p.l1 * x.dTheta * x.dTheta;
            auto V = p.m1 * g * p.l1 * -cos(x.theta);
            auto E = T + V;

            // Choose control method
            bool pumpEnergy = false;
            // Find the lowest angle we can stall at with MaxQ
            bool torqueLimited = p.MaxQ != 0 && p.MaxQ < p.m1* p.l1* g;
            if (torqueLimited)
            {
                auto minCos = p.MaxQ / mgl;
                if (-cos(x.theta) <= minCos)
                    pumpEnergy = true;
            }

            if (pumpEnergy)
            {
                auto dE = Egoal - E;

                // Expected damping
                auto tDamp = -p.b1 * x.dTheta;
                auto Ugoal = -tDamp + m_energyGain * dE * x.dTheta;
                auto U = Ugoal;
                if(p.MaxQ > 0)
                    U = std::min(p.MaxQ, std::max(-p.MaxQ, Ugoal));
                return U;
            }
            else
            {
                // Switch to a position error controller
                // or maybe a bang-bang?
                return 0;
            }
        }
    }

    void stepSimulation()
    {
        auto& params = m_pendulumParams;
        auto& x = m_pendulumState;

        auto u = computeControllerInput();

        auto b = m_pendulumParams.b1;
        auto torque = u -params.b1 * x.dTheta - sin(x.theta) * g * params.l1;

        const auto invInertia = m_pendulumParams.I1 > 0 ? (1 / m_pendulumParams.I1) : 0;
        auto ddq = torque * invInertia;

        x.theta += m_stepDt * x.dTheta + 0.5 *ddq * m_stepDt * m_stepDt;
        x.dTheta += ddq * m_stepDt;
    }

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

    struct squirrelRng
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
    } m_rng;

    void plotPendulum()
    {
        plotCircle<20>("Origin", 0, 0, 0.1f);
        double x = m_pendulumParams.l1 * sin(m_pendulumState.theta);
        double y = -m_pendulumParams.l1 * cos(m_pendulumState.theta);
        plotLine("axis", { 0, 0 }, { x, y });
        plotCircle<20>("End point", x, y, 0.1f);
    }

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
};

// Main code
int main(int, char**)
{
    PendulumApp app;
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
