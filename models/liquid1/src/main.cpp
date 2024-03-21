// Molecular dynamics playground

#include "imgui.h"
#include "implot.h"
#include "plot.h"
#include <cmath>
#include "app.h"
#include <math/vector.h>
#include <math/matrix.h>
#include <random>

#include "Agent.h"
#include "policy.h"

#include <libs/eigen/Eigen/Core>

using namespace math;

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
    MLPPolicy policy;
    MLPPolicy bestPolicy;
    // LinearPolicy policy;
    // LinearPolicy bestPolicy;
    float weightAmplitude = 1;

    enum class SimState
    {
        Running,
        Training,
        Stopped
    } m_simState = SimState::Stopped;

    CartApp()
    {
        randomizeStart();
        bestPolicy.randomizeWeights(m_rng, weightAmplitude);
        policy = bestPolicy;
        cart.m_state.pos.y() = 0; // Start at the center pos first
    }

    void randomizeStart()
    {
        cart.m_state.pos.x() = -testTrack.length * 0.5;
        cart.m_state.pos.y() = -testTrack.width * 0.5 + testTrack.width * m_rng.uniform();
        cart.m_state.orient = (-0.5 + m_rng.uniform()) * 3.1415927;
        cart.m_state.vLeft = 0;
        cart.m_state.vRight = 0;
    }

    void update() override
    {
        drawUI();

        // Run simulation
        if (m_simState == SimState::Running)
        {
            advanceSimulation();
        }
        if (m_simState == SimState::Training)
        {
            // Do one epoch per step
            if (m_maxEpoch == 0 || m_curEpoch < m_maxEpoch)
            {
                policy.randomizeWeights(m_rng, weightAmplitude);
                double totalScore = 0;
                for (int i = 0; i < m_iterationsPerEpoch; ++i)
                {
                    randomizeStart();
                    bool alive = true;
                    while (alive)
                    {
                        alive = stepSimulation();
                    }

                    totalScore += m_lastScore;
                }
                if (totalScore > m_bestScore)
                {
                    m_bestScore = totalScore;
                    bestPolicy = policy;
                }
                ++m_curEpoch;
            }
            else
            {
                policy = bestPolicy;
                m_simState = SimState::Running;
            }
        }

        DrawSimulationState();
        policy.DrawActivations();
    }

    void DrawSimulationState()
    {
        // Draw simulation state
        if (ImGui::Begin("Simulation"))
        {
            float size = float(0.55 * testTrack.length);
            if (ImPlot::BeginPlot("Cart", ImVec2(-1, -1), ImPlotFlags_Equal))
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

    void drawUI()
    {
        if (ImGui::Button("Train"))
        {
            m_simState = SimState::Training;
            m_curEpoch = 0;
        }
        if (ImGui::Button("Play"))
        {
            m_simState = SimState::Running;
            policy = bestPolicy;
        }
        if (ImGui::Button("Stop"))
        {
            m_simState = SimState::Stopped;
        }
        if (ImGui::Button("Reset training"))
        {
            m_bestScore = 0;
            bestPolicy.randomizeWeights(m_rng, weightAmplitude);
        }
        // Plot params
        if (ImGui::CollapsingHeader("Params"))
        {
            ImGui::InputDouble("Axis Len", &cart.m_params.axisLen);
            ImGui::InputDouble("Max vel", &cart.m_params.maxWheelVel);
            ImGui::InputFloat("Amplitude", &weightAmplitude);

            if (ImGui::Button("Generate"))
            {
                cart.m_params.axisLen = 0.1 + 0.4 * m_rng.uniform();
            }
            if (ImGui::Button("Randomize Policy"))
            {
                policy.randomizeWeights(m_rng, weightAmplitude);
                randomizeStart();
            }
        }

        if (ImGui::CollapsingHeader("Training"))
        {
            ImGui::InputInt("training epochs", &m_maxEpoch);
            ImGui::InputInt("iterations/epoch ", &m_iterationsPerEpoch);
            ImGui::Text("Epoch: %d", m_curEpoch);
            ImGui::Text("Last Score: %f", (float)m_lastScore);
            ImGui::Text("Best Score: %f", (float)m_bestScore / m_iterationsPerEpoch);
        }

        // Plot state
        if(ImGui::CollapsingHeader("State"))
        {
            ImGui::InputDouble("Left", &cart.m_state.vLeft);
            ImGui::InputDouble("Right", &cart.m_state.vRight);
            ImGui::InputDouble("Angle", &cart.m_state.orient);
            if (ImGui::Button("Randomize start"))
            {
                randomizeStart();
            }
        }
    }

private:
    double m_accumTime = 0;
    double m_stepDt = 0.04;
    double m_runTime = 0;
    double m_timeOut = 10;
    double m_lastScore = 0;
    double m_bestScore = 0;
    int m_curEpoch = 0;
    int m_maxEpoch = 1000;
    int m_iterationsPerEpoch = 40;

    void advanceSimulation()
    {
        m_accumTime += 1 / 60.0;
        while (m_accumTime > m_stepDt)
        {
            m_accumTime -= m_stepDt;

            bool stillAlive = stepSimulation();
        }
    }

    // returns whether the simulation is ongoing
    bool stepSimulation()
    {
        m_runTime += m_stepDt;
        auto action = policy.computeAction(m_rng, cart, testTrack);
        cart.step(m_stepDt, action);
        bool dead = !testTrack.isValidPos(cart.m_state.pos);
        bool win = testTrack.isGoal(cart.m_state.pos);
        bool timeOut = m_runTime > m_timeOut;
        if (timeOut || dead || win)
        {
            m_lastScore = testTrack.length * 0.5 + cart.m_state.pos.x();
            randomizeStart();
            m_runTime = 0;
            return false;
        }

        return true;
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
