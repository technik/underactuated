// Molecular dynamics playground

#include "imgui.h"
#include "implot.h"
#include <cmath>
#include "app.h"
#include <math/vector.h>
#include <math/matrix.h>
#include <numbers>
#include <random>

#include <libs/eigen/Eigen/Core>

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

    void step(double dt, const Input& action)
    {
        // Basic Euler integration
        double meanVel = 0.5 * (m_state.vRight + m_state.vLeft);
        double diffVel = (m_state.vRight - m_state.vLeft) / m_params.axisLen;
        double ct = cos(m_state.orient);
        double st = sin(m_state.orient);
        m_state.pos += (dt * meanVel) * Vec2d(ct, st);
        m_state.orient += dt * diffVel;
        if (m_state.orient < -TwoPi)
            m_state.orient += TwoPi;
        if (m_state.orient > TwoPi)
            m_state.orient -= TwoPi;

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

struct CartPolicy
{
    using Action = DifferentialCart::Input;
    virtual Action computeAction(SquirrelRng& rng, const DifferentialCart& agent, LinearTrack& track) = 0;
};

struct RandomPolicy : CartPolicy
{
    Action computeAction(SquirrelRng& rng, const DifferentialCart& agent, LinearTrack& track) override
    {
        Action action;
        action.dvLeft = rng.uniform() - 0.5;
        action.dvRight = rng.uniform() - 0.5;
        return action;
    }
};

struct LinearPolicy : CartPolicy
{
    Eigen::Matrix<float, 2, 6> weights;
    Eigen::Vector<float, 6> inputVector;

    void randomizeWeights(SquirrelRng& rng)
    {
        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < 6; ++j)
            {
                weights(i, j) = rng.uniform() * 2 - 1;
            }
        }
    }

    Action computeAction(SquirrelRng& rng, const DifferentialCart& agent, LinearTrack& track) override
    {
        // Computen input vector from agent state
        auto& state = agent.m_state;
        inputVector[0] = state.orient;
        inputVector[1] = state.pos.x();
        inputVector[2] = state.pos.y();
        inputVector[3] = state.vLeft;
        inputVector[4] = state.vRight;
        inputVector[5] = 1; // Bias

        // Apply the single neuron layer
        Eigen::Vector<float, 2> activations = weights * inputVector;

        Action action;
        action.dvLeft = activations[0];
        action.dvRight = activations[1];
        return action;
    }
};

class CartApp : public App
{
public:
    DifferentialCart cart;
    LinearTrack testTrack;
    //RandomPolicy policy;
    LinearPolicy policy;
    LinearPolicy bestPolicy;

    enum class SimState
    {
        Running,
        Training,
        Stopped
    } m_simState = SimState::Stopped;

    CartApp()
    {
        randomizeStart();
        bestPolicy.randomizeWeights(m_rng);
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
                policy.randomizeWeights(m_rng);
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
            randomizeStart();
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
            bestPolicy.randomizeWeights(m_rng);
        }
        // Plot params
        if (ImGui::CollapsingHeader("Params"))
        {
            ImGui::InputDouble("Axis Len", &cart.m_params.axisLen);
            ImGui::InputDouble("Max vel", &cart.m_params.maxWheelVel);

            if (ImGui::Button("Generate"))
            {
                cart.m_params.axisLen = 0.1 + 0.4 * m_rng.uniform();
            }
            if (ImGui::Button("Randomize Policy"))
            {
                policy.randomizeWeights(m_rng);
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
    double m_stepDt = 0.001;
    double m_runTime = 0;
    double m_timeOut = 10;
    double m_lastScore = 0;
    double m_bestScore = 0;
    int m_curEpoch = 0;
    int m_maxEpoch = 20;
    int m_iterationsPerEpoch = 10;

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
