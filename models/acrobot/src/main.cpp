// Molecular dynamics playground

#include "imgui.h"
#include "implot.h"
#include <cmath>
#include "app.h"
#include <math/noise.h>
#include <math/vector.h>
#include <math/matrix.h>
#include <numbers>
#include <random>

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
            inertiaChanged |= ImGui::InputDouble("Mass 1", &m_acrobot.p.m1);
            inertiaChanged |= ImGui::InputDouble("Mass 2", &m_acrobot.p.m2);
            inertiaChanged |= ImGui::InputDouble("Length 1", &m_acrobot.p.l1);
            inertiaChanged |= ImGui::InputDouble("Length 2", &m_acrobot.p.l2);
            if(inertiaChanged)
            {
                m_acrobot.p.refreshInertia();
            }
            ImGui::InputDouble("Friction 1", &m_acrobot.p.b1);
            ImGui::InputDouble("Friction 2", &m_acrobot.p.b2);
            ImGui::InputDouble("Torque Limit", &m_acrobot.p.MaxQ);

            if (ImGui::Button("Generate"))
            {
                m_acrobot.p.l1 = m_rng.uniform() * 10;
                m_acrobot.p.l2 = m_rng.uniform() * 10;
                m_acrobot.p.m1 = m_rng.uniform() * 10;
                m_acrobot.p.m2 = m_rng.uniform() * 10;
                m_acrobot.p.b1 = m_rng.uniform() * 10;
                m_acrobot.p.b2 = m_rng.uniform() * 10;
                m_acrobot.p.refreshInertia();
            }
        }

        // Plot state
        if(ImGui::CollapsingHeader("State"))
        {
            ImGui::InputDouble("q1", &m_acrobot.x.q1);
            ImGui::InputDouble("q2", &m_acrobot.x.q2);
            ImGui::InputDouble("dq1", &m_acrobot.x.dq1);
            ImGui::InputDouble("dq2", &m_acrobot.x.dq2);
            if (ImGui::Button("Randomize State"))
            {
                m_acrobot.x.q1 = m_rng.uniform() * 2 * 3.1415927;
                m_acrobot.x.q2 = m_rng.uniform() * 2 * 3.1415927;
            }
            if (ImGui::Button("Reset Speed"))
            {
                m_acrobot.x.dq1 = 0;
                m_acrobot.x.dq2 = 0;
            }
            if (ImGui::Button("Perturbate"))
            {
                m_acrobot.x.dq1 += m_rng.uniform() - 0.5;
                m_acrobot.x.dq2 += m_rng.uniform() - 0.5;
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
            float size = float(1.1 * (m_acrobot.p.l1 + m_acrobot.p.l2));
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
    struct Acrobot
    {
        struct Params
        {
            double l1 = 1, l2 = 1; // Bar lengths
            double m1 = 1, m2 = 1; // Bar masses
            double b1 = 0, b2 = 0; // Friction at the joints
            double I1 = 1, I2 = 1; // Inertia tensors
            double MaxQ = 0; // Torque limit. 0 means unlimited torque

            void refreshInertia()
            {
                // Inertia concentrated at the end
                I1 = m1 * l1 * l1 / 12;
                I2 = m2 * l2 * l2 / 12;
            }
        } p;

        struct State
        {
            double q1 = 0, q2 = 0; // positions
            double dq1 = 0, dq2 = 0; // velocities
        } x;

        static double kineticEnergy(
            double m1, double m2,
            double l1, double l2,
            double I1, double I2,
            double q1, double q2,
            double dq1, double dq2
        )
        {
            auto T1 = 0.5 * I1 * pow(q1, 2);
            auto T2 =
                (m2 * pow(l1, 2) + I2 + 2 * m2 * l1 * l2 * cos(q2)) * pow(dq1, 2) / 2
                + I2 * pow(dq2, 2) / 2
                + (I2 + m2 * l1 * l2 * cos(q2)) * dq1 * dq2;

            return T1+T2;
        }

        static double PotentialEnergy(
            double m1, double m2,
            double l1, double l2,
            double q1, double q2
        )
        {
            auto q1_q2 = q1 + q2;
            auto cq1 = cos(q1);
            auto cq2 = cos(q1_q2);
            return -m1 * g * l1 * cq1 - m2 * g * (l1 * cq1 + l2 * cq2);
        }

        // Manipulator equations
        Mat22d M(Vec2d q)
        {
            return Mat22d(
                p.I1 + p.I2 + p.m2 * pow(p.l1, 2) + 2 * p.m2 * p.l1 * p.l2 * cos(q[2]),
                p.I2 + p.m2 * p.l1 * p.l2 * cos(q[2]),
                p.I2 + p.m2*p.l1*p.l2*cos(q[2]),
                p.I2
            );
        }

    } m_acrobot;

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
        double x1 = m_acrobot.p.l1 * sin(m_acrobot.x.q1);
        double y1 = -m_acrobot.p.l1 * cos(m_acrobot.x.q1);
        double x2 = x1 + m_acrobot.p.l2 * sin(m_acrobot.x.q1 + m_acrobot.x.q2);
        double y2 = y1 - m_acrobot.p.l2 * cos(m_acrobot.x.q1 + m_acrobot.x.q2);
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
