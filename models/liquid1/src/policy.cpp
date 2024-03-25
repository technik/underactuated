
#include "policy.h"
#include "track.h"

#include "imgui.h"
#include "implot.h"
#include "plot.h"
#include <math/vector.h>
#include <math/matrix.h>
#include <math/noise.h>

using namespace math;

namespace
{
    ImVec4 mapColor(float intensity)
    {
        float r = min(1.f, max(0.f, -intensity));
        float g = min(1.f, max(0.f, intensity));
        float b = 0;
        float a = min(1.f, abs(intensity));
        return ImVec4(r, g, b, a);
    }
}

auto RandomPolicy::computeAction(SquirrelRng& rng, const DifferentialCart& agent, LinearTrack& track) -> Action
{
    Action action;
    action.dvLeft = rng.uniform() - 0.5;
    action.dvRight = rng.uniform() - 0.5;
    return action;
}

void LinearPolicy::randomizeWeights(SquirrelRng& rng, float amplitude)
{
    for (int i = 0; i < kNumOutputs; ++i)
    {
        for (int j = 0; j < kNumInputs; ++j)
        {
            weights(i, j) = amplitude * (rng.uniform() * 2 - 1);
        }
    }
}

auto LinearPolicy::computeAction(SquirrelRng& rng, const DifferentialCart& agent, LinearTrack& track) -> Action
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

void LinearPolicy::DrawActivations()
{
    if (ImGui::Begin("Activations"))
    {
        float size = float(kNumOutputs + 1);
        if (ImPlot::BeginPlot("Cart", ImVec2(-1, -1), ImPlotFlags_Equal | ImPlotFlags_NoLegend))
        {
            ImPlot::SetupAxis(ImAxis_X1, NULL, ImPlotAxisFlags_AuxDefault);
            ImPlot::SetupAxisLimits(ImAxis_X1, -size, size, ImGuiCond_Always);
            ImPlot::SetupAxis(ImAxis_Y1, NULL, ImPlotAxisFlags_AuxDefault);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -size, size, ImGuiCond_Always);
            for (int i = 0; i < kNumOutputs; ++i)
            {
                float outH = 1 - i;
                for (int j = 0; j < kNumInputs; ++j)
                {
                    float inH = int(kNumInputs / 2 - j);
                    float activation = weights(i, j) * inputVector[j];
                    float r = min(1.f, max(0.f, -activation));
                    float g = min(1.f, max(0.f, 1.f - abs(activation)));
                    float b = min(1.f, max(0.f, activation));
                    ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(r, g, b, 1));
                    std::stringstream label;
                    label << "synapse" << i << "," << j;
                    plotLine(label.str().c_str(), Vec2d(-1, inH), Vec2d(1, outH));
                    ImPlot::PopStyleColor();
                }

            }
        }
        ImPlot::EndPlot();
    }
    ImGui::End();
}

void MLPPolicy::randomizeWeights(SquirrelRng& rng, float amplitude)
{
    m_network.randomize(rng);
}

auto MLPPolicy::generateVariation(math::SquirrelRng& rng, float variationStep) const -> Matrix
{
    return m_network.randomDelta(rng, variationStep);
}

void MLPPolicy::applyVariation(const Matrix& delta)
{
    m_network.step(delta);
}

auto MLPPolicy::computeAction(SquirrelRng& rng, const DifferentialCart& agent, LinearTrack& track) -> Action
{
    // Computen input vector from agent state
    // Project cart state
    auto& state = agent.m_state;
    auto dir = Vec2d(cos(state.orient), sin(state.orient));
    auto p = track.project(state.pos, dir);
    inputVector[0] = p.x;
    inputVector[1] = p.y;
    inputVector[2] = p.cosT;
    inputVector[3] = p.sinT;

    auto activations = m_network.forward(inputVector);

    Action action;
    action.dvLeft = activations(0);
    action.dvRight = activations(1);
    return action;
}

void MLPPolicy::DrawActivations()
{
    if (ImGui::Begin("Activations"))
    {
        float size = kHiddenSize/2 + 0.5f;
        if (ImPlot::BeginPlot("Cart", ImVec2(-1, -1), ImPlotFlags_Equal | ImPlotFlags_NoLegend))
        {
            const float nodeRadius = 0.25f;
            ImPlot::SetupAxis(ImAxis_X1, NULL, ImPlotAxisFlags_AuxDefault);
            ImPlot::SetupAxisLimits(ImAxis_X1, -3, 3, ImGuiCond_Always);
            ImPlot::SetupAxis(ImAxis_Y1, NULL, ImPlotAxisFlags_AuxDefault);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -size, size, ImGuiCond_Always);

            for (int i = 0; i < kHiddenSize; ++i)
            {
                float outH = kHiddenSize/2 - 0.5 - i;
                for (int j = 0; j < kNumInputs; ++j)
                {
                    float inH = kNumInputs / 2 - 0.5 - j;
                    ImPlot::PushStyleColor(ImPlotCol_Line, mapColor(m_network.m_layers[0].W(i, j)));
                    std::stringstream label;
                    label << "synapse0_i_" << i << "," << j;
                    plotLine(label.str().c_str(), Vec2d(-2, inH), Vec2d(-1, outH));
                    ImPlot::PopStyleColor();
                }

                std::stringstream label;
                label << "activ0_i" << i;
                float activation = m_network.m_activationCache[1](i);
                ImPlot::PushStyleColor(ImPlotCol_Line, mapColor(activation));
                plotCircle<8>(label.str().c_str(), -1.f, outH, nodeRadius);
                ImPlot::PopStyleColor();
            }

            for (int i = 0; i < kHiddenSize; ++i)
            {
                float outH = kHiddenSize / 2 - 0.5 - i;
                for (int j = 0; j < kHiddenSize; ++j)
                {
                    float inH = kHiddenSize / 2 - 0.5 - j;
                    ImPlot::PushStyleColor(ImPlotCol_Line, mapColor(m_network.m_layers[1].W(i, j)));
                    std::stringstream label;
                    label << "synapse_h_" << i << "," << j;
                    plotLine(label.str().c_str(), Vec2d(-1, inH), Vec2d(0, outH));
                    ImPlot::PopStyleColor();
                }

                std::stringstream label;
                label << "activH_i" << i;
                float activation = m_network.m_activationCache[2](i);
                ImPlot::PushStyleColor(ImPlotCol_Line, mapColor(activation));
                plotCircle<8>(label.str().c_str(), 0.f, outH, nodeRadius);
                ImPlot::PopStyleColor();
            }

            for (int i = 0; i < kNumOutputs; ++i)
            {
                float outH = 2 * (0.5 - i);
                for (int j = 0; j < kHiddenSize; ++j)
                {
                    float inH = kHiddenSize / 2 - 0.5 - j;
                    ImPlot::PushStyleColor(ImPlotCol_Line, mapColor(m_network.m_layers[2].W(i, j)));
                    std::stringstream label;
                    label << "synapse_o_" << i << "," << j;
                    plotLine(label.str().c_str(), Vec2d(0, inH), Vec2d(1, outH));
                    ImPlot::PopStyleColor();
                }

                std::stringstream label;
                label << "activH_o" << i;
                float activation = m_network.m_activationCache[3](i);
                ImPlot::PushStyleColor(ImPlotCol_Line, mapColor(activation));
                plotCircle<8>(label.str().c_str(), 1.f, outH, nodeRadius);
                ImPlot::PopStyleColor();
            }
        }
        ImPlot::EndPlot();
    }
    ImGui::End();
}