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
    double length = 10; // deprecated
    double width = 1;
    float radius = 10;

    std::vector<float> m_insideLineX;
    std::vector<float> m_insideLineY;
    std::vector<float> m_outsideLineX;
    std::vector<float> m_outsideLineY;

    std::vector<float> m_sectorCumLen;

    void Init()
    {
        // vertices:
        constexpr auto numVtx = 5;

        // Init track corners
        Vec2d v[numVtx];
        for (size_t i = 0; i < numVtx; ++i)
        {
            v[i] = Vec2d(
                radius * cos(-math::TwoPi * i / numVtx),
                radius * sin(-math::TwoPi * i / numVtx));
        }

        // Init track sectors
        m_sectors.resize(numVtx-1);
        for (size_t i = 0; i < numVtx-1; ++i)
        {
            m_sectors[i] = Segment(v[i], v[i+1]);
        }

        // Cache drawing lines
        m_insideLineX.resize(numVtx);
        m_insideLineY.resize(numVtx);
        m_outsideLineX.resize(numVtx);
        m_outsideLineY.resize(numVtx);
        for (size_t i = 0; i < numVtx; ++i)
        {
            Vec2d n;
            if (i == 0)
            {
                n = 0.5 * width * m_sectors[0].m_normal;
            }
            else if (i == numVtx - 1)
            {
                n = 0.5 * width * m_sectors.back().m_normal;
            }
            else
            {
                n = 0.5 * width * normalize(m_sectors[i-1].m_normal + m_sectors[i].m_normal);
            }
            auto inside = v[i] - n;
            auto outside = v[i] + n;
            m_insideLineX[i] = inside.x();
            m_insideLineY[i] = inside.y();
            m_outsideLineX[i] = outside.x();
            m_outsideLineY[i] = outside.y();
        }

        // cache sector cummulative length
        float accumPath = 0;
        m_sectorCumLen.resize(m_sectors.size());
        for (size_t i = 0; i < numVtx - 1; ++i)
        {
            m_sectorCumLen[i] = accumPath;
            accumPath += m_sectors[i].m_len;
        }
    }

    bool m_plotOpen = false;

    void BeginPlot()
    {
        // Draw simulation state
        if (ImGui::Begin("SimTrack"))
        {
            float size = radius + width;
            m_plotOpen = ImPlot::BeginPlot("Track", ImVec2(-1, -1), ImPlotFlags_Equal);
            if(m_plotOpen)
            {
                // Set up rigid axes
                ImPlot::SetupAxis(ImAxis_X1, NULL, ImPlotAxisFlags_AuxDefault);
                ImPlot::SetupAxisLimits(ImAxis_X1, -size, size, ImGuiCond_Always);
                ImPlot::SetupAxis(ImAxis_Y1, NULL, ImPlotAxisFlags_AuxDefault);

                // Draw track limits
                ImPlot::PlotLine("inside", m_insideLineX.data(), m_insideLineY.data(), m_insideLineX.size());
                ImPlot::PlotLine("outside", m_outsideLineX.data(), m_outsideLineY.data(), m_outsideLineX.size());
            }
        }
    }

    void EndPlot()
    {
        if (m_plotOpen)
        {
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

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

    struct Segment
    {
        Segment() = default;
        Segment(Vec2d s, Vec2d e)
            : m_start(s), m_end(e)
        {
            m_dir = normalize(m_end - m_start);
            m_normal = Vec2d(-m_dir.y(), m_dir.x());
            m_len = dot(m_end - m_start, m_dir);
        }
        Vec2d m_start, m_end;
        Vec2d m_dir, m_normal;
        double m_len;

        float closestPointDistance(const Vec2d& pos, Vec2d& outP)
        {
            auto t = dot(pos - m_start, m_dir);
            outP = (t <= 0) ? m_start : ((t >= m_len) ? m_end : (m_start + t * m_dir));
            return (outP - pos).norm();
        }

        struct ProjectedState
        {
            float x, y;
            float cosT, sinT;
        };

        ProjectedState project(const Vec2d pos, const Vec2d dir)
        {
            ProjectedState result;
            auto relPos = pos - m_start;
            result.x = dot(relPos, m_dir);
            result.y = dot(relPos, m_normal);
            result.cosT = dot(dir, m_dir);
            result.sinT = dot(dir, m_normal);

            return result;
        }
    };

    size_t closestSegment(const Vec2d& pos)
    {
        Vec2d cp;
        float minDistance = std::numeric_limits<float>::max();
        size_t segmentId = -1;
        for (size_t i = 0; i < m_sectors.size(); ++i)
        {
            float t = m_sectors[0].closestPointDistance(pos, cp);
            if (t < minDistance)
            {
                minDistance = t;
                segmentId = i;
            }
        }
        return segmentId;
    };

    std::vector<Segment> m_sectors;
};

struct Simulation
{
    DifferentialCart cart;
    LinearTrack testTrack;

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

        testTrack.BeginPlot();
        testTrack.EndPlot();
    }
};

class CartApp : public App
{
public:
    using Policy = MLPPolicy;
    using Agent = DifferentialCart;

    Simulation m_sim;
    Policy m_bestPolicy;
    // LinearPolicy policy;
    // LinearPolicy bestPolicy;
    float weightAmplitude = 1;
    float gradientStep = 0.01;
    float learnStep = 0.1;

    int m_trainSetSize = 2000;
    int m_validationSize = 100;

    std::vector<Agent::State> m_trainSet;
    std::vector<Agent::State> m_validationSet;

    enum class SimState
    {
        Running,
        RandomExplore,
        SGD,
        Stopped
    } m_simState = SimState::Stopped;

    CartApp()
    {
        m_bestPolicy.randomizeWeights(m_rng, weightAmplitude);
        m_sim.testTrack.Init();
    }

    auto GenerateTestCases(SquirrelRng& rng, size_t numCases, bool anyStartPos)
    {
        auto minX = -0.5 * m_sim.testTrack.length;
        auto maxX = anyStartPos ? 0.5 * m_sim.testTrack.length : minX;
        std::vector<Agent::State> testSet(numCases);
        int numStartCases = numCases / 2; // Dedicate some cases to the start
        int i = 0;
        for (auto& t : testSet)
        {
            t.randomize(minX, i > numStartCases ? maxX : minX, m_sim.testTrack.width, m_rng);
            i++;
        }
        return testSet;
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
                    float dE = (totalScore - m_bestScore) / gradientStep;
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
        for (auto& t : m_trainSet)
        {
            m_sim.cart.m_state = t;
            for (;;)
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
            if (m_trainSet.size() != m_iterationsPerEpoch)
            {
                m_trainSet = GenerateTestCases(m_rng, m_iterationsPerEpoch, true);
                m_validationSet = GenerateTestCases(m_rng, m_iterationsPerEpoch, false);
            }
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
                auto startPos = -m_sim.testTrack.length * 0.5f;
                m_sim.cart.m_state.randomize(startPos, startPos, m_sim.testTrack.width, m_rng);
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
