// Molecular dynamics playground

#include "imgui.h"
#include "implot.h"
#include <cmath>
#include "app.h"
#include <math/noise.h>
#include <math/vector.h>
#include <numbers>
#include <random>

static constexpr auto Pi = std::numbers::pi_v<double>;
static constexpr auto TwoPi = 2 * std::numbers::pi_v<double>;

using namespace math;

class AcrobotApp : public App
{
public:
    AcrobotApp()
    {
    }

    void update() override
    {
        // Plot params
        if (ImGui::CollapsingHeader("Params"))
        {
            bool inertiaChanged = false;
            inertiaChanged |= ImGui::InputDouble("Mass 1", &m_params.m1);
            inertiaChanged |= ImGui::InputDouble("Mass 2", &m_params.m2);
            inertiaChanged |= ImGui::InputDouble("Length 1", &m_params.l1);
            inertiaChanged |= ImGui::InputDouble("Length 2", &m_params.l2);
            if(inertiaChanged)
            {
                m_params.refreshInertia();
            }
            ImGui::InputDouble("Friction 1", &m_params.b1);
            ImGui::InputDouble("Friction 2", &m_params.b2);
            ImGui::InputDouble("Torque Limit", &m_params.MaxQ);

            if (ImGui::Button("Generate"))
            {
                m_params.l1 = m_rng.uniform() * 10;
                m_params.l2 = m_rng.uniform() * 10;
                m_params.m1 = m_rng.uniform() * 10;
                m_params.m2 = m_rng.uniform() * 10;
                m_params.b1 = m_rng.uniform() * 10;
                m_params.b2 = m_rng.uniform() * 10;
                m_params.refreshInertia();
            }
        }

        // Plot state
        if(ImGui::CollapsingHeader("State"))
        {
            ImGui::InputDouble("q1", &m_state.q1);
            ImGui::InputDouble("q2", &m_state.q2);
            ImGui::InputDouble("dq1", &m_state.dq1);
            ImGui::InputDouble("dq2", &m_state.dq2);
            if (ImGui::Button("Randomize State"))
            {
                m_state.q1 = m_rng.uniform() * 2 * 3.1415927;
                m_state.q2 = m_rng.uniform() * 2 * 3.1415927;
            }
            if (ImGui::Button("Reset Speed"))
            {
                m_state.dq1 = 0;
                m_state.dq2 = 0;
            }
            if (ImGui::Button("Perturbate"))
            {
                m_state.dq1 += m_rng.uniform() - 0.5;
                m_state.dq2 += m_rng.uniform() - 0.5;
            }
        }

        // Plot state
        if (ImGui::CollapsingHeader("Control"))
        {
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
            float size = float(1.1 * (m_params.l1 + m_params.l2));
            if(ImPlot::BeginPlot("Acrobot", ImVec2(-1, -1), ImPlotFlags_Equal))
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
    struct AcrobotParams
    {
        double l1 = 1, l2 = 1; // Bar lengths
        double m1 = 1, m2 = 1; // Bar masses
        double b1 = 0, b2 = 0; // Friction at the joints
        double I1 = 1, I2 = 1; // Inertia tensors
        double MaxQ = 0; // Torque limit. 0 means unlimited torque

        void refreshInertia()
        {
            // Inertia concentrated at the end
            I1 = m1 * l1 * l1;
            I2 = m2 * l2 * l2;
        }
    } m_params;

    struct AcrobotState
    {
        double q1 = 0, q2 = 0; // positions
        double dq1 = 0, dq2 = 0; // velocities
    } m_state;

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

    auto computeControllerInput()
    {
        return Vec2d{ 0, 0 };
    }

    void stepSimulation()
    {
    }

    static constexpr auto g = 9.81;
    
    LinearCongruentalGenerator m_rng;

    void plotPendulum()
    {
        plotCircle<20>("Origin", 0, 0, 0.1f);
        double x1 = m_params.l1 * sin(m_state.q1);
        double y1 = -m_params.l1 * cos(m_state.q1);
        double x2 = x1 + m_params.l2 * sin(m_state.q1 + m_state.q2);
        double y2 = y1 - m_params.l2 * cos(m_state.q1 + m_state.q2);
        plotLine("l1", { 0, 0 }, { x1, y1 });
        plotLine("l2", { x1, y1 }, { x2, y2 });
        plotCircle<20>("End point", x2, y2, 0.1f);
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
    AcrobotApp app;
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
