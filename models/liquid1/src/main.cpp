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
#include <optional>

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

struct Simulation
{
    DifferentialCart cart;
    LinearTrack testTrack;

    void randomizeStart(SquirrelRng& rng)
    {
        cart.m_state.pos.x() = -testTrack.length * 0.5;
        cart.m_state.pos.y() = -testTrack.width * 0.5 + testTrack.width * rng.uniform();
        cart.m_state.orient = (-0.5 + rng.uniform()) * 3.1415927;
        cart.m_state.vLeft = 0;
        cart.m_state.vRight = 0;
    }

    void DrawSimState()
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

                testTrack.draw();
                cart.draw();
            }
            ImPlot::EndPlot();
        }
        ImGui::End();
    }
};

class CartApp : public App
{
public:
    using Policy = MLPPolicy;

    Simulation m_sim;
    Policy m_bestPolicy;
    // LinearPolicy policy;
    // LinearPolicy bestPolicy;
    float weightAmplitude = 1;
    float gradientStep = 0.01;
    float learnStep = 0.1;

    enum class SimState
    {
        Running,
        RandomExplore,
        SGD,
        Stopped
    } m_simState = SimState::Stopped;

    CartApp()
    {
        m_sim.randomizeStart(m_rng);
        m_bestPolicy.randomizeWeights(m_rng, weightAmplitude);
    }

    void update() override
    {
        drawUI();

        // Run simulation
        if (m_simState == SimState::Running)
        {
            advanceSimulation();
        }
        if (m_simState == SimState::RandomExplore)
        {
            // Generate a fixed training set
            
            // Do one epoch per step
            if (m_curEpoch < m_maxRandomEpoch)
            {
                // Generate a random gradient
                auto policy = m_bestPolicy;
                policy.randomizeWeights(m_rng, weightAmplitude);

                // Evaluate a batch
                float totalScore = EvaluateBatch(policy);

                // Update the policy
                if (totalScore > m_bestScore)
                {
                    m_bestScore = totalScore;
                    m_bestPolicy = policy;
                }
                ++m_curEpoch;
            }
            else
            {
                m_curEpoch = 0;
                m_simState = SimState::SGD;
            }
        }
        else if (m_simState == SimState::SGD)
        {
            // Do one epoch per step
            if (m_maxSGDEpoch == 0 || m_curEpoch < m_maxSGDEpoch)
            {
                // Generate a random gradient
                auto delta = m_bestPolicy.generateVariation(m_rng, gradientStep);
                auto policy = m_bestPolicy;
                policy.applyVariation(delta, 1.f);

                // Evaluate a batch
                float totalScore = EvaluateBatch(policy);

                if (m_useSGD)
                {
                    float dE = totalScore - m_bestScore;
                    // Apply gradient scaled correction
                    policy = m_bestPolicy;
                    policy.applyVariation(delta, learnStep * dE);

                    // Re-evaluate
                    totalScore = EvaluateBatch(policy);
                }

                // Update the policy
                if (totalScore > m_bestScore)
                {
                    m_bestScore = totalScore;
                    m_bestPolicy = policy;
                }

                ++m_curEpoch;
            }
            else
            {
                m_simState = SimState::Running;
            }
        }

        m_sim.DrawSimState();
        m_bestPolicy.DrawActivations();
    }

    float EvaluateBatch(MLPPolicy& policy)
    {
        double totalScore = 0;
        for (int i = 0; i < m_iterationsPerEpoch; ++i)
        {
            m_sim.randomizeStart(m_rng);
            bool alive = true;
            while (alive)
            {
                auto res = stepSimulation(policy);
                if (res.has_value())
                {
                    totalScore += res.value();
                    break;
                }
            }
        }
        return totalScore;
    }

    void drawUI()
    {
        if (ImGui::Button("Train"))
        {
            m_simState = SimState::RandomExplore;
            m_curEpoch = 0;
        }
        if (ImGui::Button("Play"))
        {
            m_simState = SimState::Running;
        }
        if (ImGui::Button("Stop"))
        {
            m_simState = SimState::Stopped;
        }
        if (ImGui::Button("Reset training"))
        {
            m_bestScore = 0;
            m_bestPolicy.randomizeWeights(m_rng, weightAmplitude);
        }
        // Plot params
        if (ImGui::CollapsingHeader("Params"))
        {
            ImGui::InputDouble("Axis Len", &m_sim.cart.m_params.axisLen);
            ImGui::InputDouble("Max vel", &m_sim.cart.m_params.maxWheelVel);
            ImGui::InputFloat("Amplitude", &weightAmplitude);
            ImGui::InputFloat("SGD diff step", &gradientStep);
            ImGui::InputFloat("SGD learn step", &learnStep);

            if (ImGui::Button("Generate"))
            {
                m_sim.cart.m_params.axisLen = 0.1 + 0.4 * m_rng.uniform();
            }
        }

        if (ImGui::CollapsingHeader("Training"))
        {
            ImGui::InputInt("explore epochs", &m_maxRandomEpoch);
            ImGui::InputInt("descent epochs", &m_maxSGDEpoch);
            ImGui::InputInt("iterations/epoch ", &m_iterationsPerEpoch);
            ImGui::Checkbox("Use SGD", &m_useSGD);
            ImGui::Text("Epoch: %d", m_curEpoch);
            ImGui::Text("Last Score: %f", (float)m_lastScore);
            ImGui::Text("Best Score: %f", (float)m_bestScore / m_iterationsPerEpoch);
        }
    }

private:
    double m_accumTime = 0;
    double m_stepDt = 0.01;
    double m_runTime = 0;
    double m_timeOut = 10;
    double m_lastScore = 0;
    double m_bestScore = 0;
    bool m_useSGD = true;
    int m_curEpoch = 0;
    int m_maxRandomEpoch = 2000;
    int m_maxSGDEpoch = 2000;
    int m_iterationsPerEpoch = 100;

    void advanceSimulation()
    {
        m_accumTime += 1 / 60.0;
        while (m_accumTime > m_stepDt)
        {
            m_accumTime -= m_stepDt;

            auto res = stepSimulation(m_bestPolicy);
            if (res.has_value())
            {
                m_lastScore = res.value();
                break;
            }
        }
    }

    // returns whether the simulation is ongoing
    std::optional<float> stepSimulation(Policy& policy)
    {
        m_runTime += m_stepDt;
        auto action = policy.computeAction(m_rng, m_sim.cart, m_sim.testTrack);
        m_sim.cart.step(m_stepDt, action);
        bool dead = !m_sim.testTrack.isValidPos(m_sim.cart.m_state.pos);
        bool win = m_sim.testTrack.isGoal(m_sim.cart.m_state.pos);
        bool timeOut = m_runTime > m_timeOut;
        if (timeOut || dead || win)
        {
            float score = m_sim.testTrack.length * 0.5 + m_sim.cart.m_state.pos.x();
            if (win)
                score += m_timeOut - m_runTime;
            m_sim.randomizeStart(m_rng);
            m_runTime = 0;
            return score;
        }

        return {};
    }
    
    SquirrelRng m_rng;
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
