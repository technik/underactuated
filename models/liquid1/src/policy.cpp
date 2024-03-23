
#include "policy.h"

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
    // Input layer
    for (int i = 0; i < kHiddenSize; ++i)
    {
        for (int j = 0; j < kNumInputs+1; ++j)
        {
            inputWeights(i, j) = amplitude * (rng.uniform() * 2 - 1);
        }
    }

    // Hidden layer
    for (int i = 0; i < kHiddenSize; ++i)
    {
        for (int j = 0; j < kHiddenSize+1; ++j)
        {
            hiddenWeights(i, j) = amplitude * (rng.uniform() * 2 - 1);
        }
    }

    // Output layer
    for (int i = 0; i < kNumOutputs; ++i)
    {
        for (int j = 0; j < kHiddenSize+1; ++j)
        {
            outputWeights(i, j) = amplitude * (rng.uniform() * 2 - 1);
        }
    }
}

MLPPolicy MLPPolicy::generateVariation(math::SquirrelRng& rng, float variationStep) const
{
    MLPPolicy variation = *this;
    // Input layer
    for (int i = 0; i < kHiddenSize; ++i)
    {
        for (int j = 0; j < kNumInputs + 1; ++j)
        {
            variation.inputWeights(i, j) = variationStep * (rng.uniform() * 2 - 1);
        }
    }

    // Hidden layer
    for (int i = 0; i < kHiddenSize; ++i)
    {
        for (int j = 0; j < kHiddenSize + 1; ++j)
        {
            variation.hiddenWeights(i, j) = variationStep * (rng.uniform() * 2 - 1);
        }
    }

    // Output layer
    for (int i = 0; i < kNumOutputs; ++i)
    {
        for (int j = 0; j < kHiddenSize + 1; ++j)
        {
            variation.outputWeights(i, j) = variationStep * (rng.uniform() * 2 - 1);
        }
    }

    return variation;
}

void MLPPolicy::applyVariation(const MLPPolicy& delta, float scale)
{
    // Input layer
    for (int i = 0; i < kHiddenSize; ++i)
    {
        for (int j = 0; j < kNumInputs + 1; ++j)
        {
            inputWeights(i, j) += scale * delta.inputWeights(i, j);
        }
    }

    // Hidden layer
    for (int i = 0; i < kHiddenSize; ++i)
    {
        for (int j = 0; j < kHiddenSize + 1; ++j)
        {
            hiddenWeights(i, j) += scale * delta.hiddenWeights(i, j);
        }
    }

    // Output layer
    for (int i = 0; i < kNumOutputs; ++i)
    {
        for (int j = 0; j < kHiddenSize + 1; ++j)
        {
            outputWeights(i, j) += scale * delta.outputWeights(i, j);
        }
    }
}

auto MLPPolicy::computeAction(SquirrelRng& rng, const DifferentialCart& agent, LinearTrack& track) -> Action
{
    // Computen input vector from agent state
    auto& state = agent.m_state;
    inputVector[0] = state.orient;
    inputVector[1] = state.pos.x();
    inputVector[2] = state.pos.y();
    inputVector[3] = state.vLeft;
    inputVector[4] = state.vRight;
    inputVector[5] = cos(state.orient);
    inputVector[6] = sin(state.orient);
    inputVector[7] = 1; // Bias

    // Input layer
    inputActivations.block<kHiddenSize,1>(0,0) = inputWeights * inputVector;
    // Relu
    for (int i = 0; i < kHiddenSize; ++i)
    {
        inputActivations(i) = max(0.f, inputActivations(i));
    }
    inputActivations[kHiddenSize] = 1; // Implicit bias term

    // Hidden layer
    hiddenActivations.block<kHiddenSize, 1>(0, 0) = hiddenWeights * inputActivations;
    // Relu
    for (int i = 0; i < kHiddenSize; ++i)
    {
        hiddenActivations(i) = max(0.f, hiddenActivations(i));
    }
    hiddenActivations[kHiddenSize] = 1; // Implicit bias term

    // Hidden layer
    outputActivations = outputWeights * hiddenActivations;

    Action action;
    action.dvLeft = outputActivations[0];
    action.dvRight = outputActivations[1];
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
                    ImPlot::PushStyleColor(ImPlotCol_Line, mapColor(inputWeights(i, j)));
                    std::stringstream label;
                    label << "synapse_i_" << i << "," << j;
                    plotLine(label.str().c_str(), Vec2d(-2, inH), Vec2d(-1, outH));
                    ImPlot::PopStyleColor();
                    label << "c";
                    float activation = inputWeights(i, j) * inputVector[j] + inputWeights(i, kNumInputs);
                    ImPlot::PushStyleColor(ImPlotCol_Line, mapColor(activation));
                    plotCircle<8>(label.str().c_str(), -1.f, outH, nodeRadius);
                    ImPlot::PopStyleColor();
                }

            }

            for (int i = 0; i < kHiddenSize; ++i)
            {
                float outH = kHiddenSize / 2 - 0.5 - i;
                for (int j = 0; j < kHiddenSize; ++j)
                {
                    float inH = kHiddenSize / 2 - 0.5 - j;
                    ImPlot::PushStyleColor(ImPlotCol_Line, mapColor(hiddenWeights(i, j)));
                    std::stringstream label;
                    label << "synapse_h_" << i << "," << j;
                    plotLine(label.str().c_str(), Vec2d(-1, inH), Vec2d(0, outH));
                    ImPlot::PopStyleColor();
                    label << "c";
                    float activation = hiddenWeights(i, j) * inputActivations[j] + hiddenWeights(i, kHiddenSize);
                    ImPlot::PushStyleColor(ImPlotCol_Line, mapColor(activation));
                    plotCircle<8>(label.str().c_str(), 0.f, outH, nodeRadius);
                    ImPlot::PopStyleColor();
                }

            }

            for (int i = 0; i < kNumOutputs; ++i)
            {
                float outH = 2 * (0.5 - i);
                for (int j = 0; j < kHiddenSize; ++j)
                {
                    float inH = kHiddenSize / 2 - 0.5 - j;
                    ImPlot::PushStyleColor(ImPlotCol_Line, mapColor(outputWeights(i, j)));
                    std::stringstream label;
                    label << "synapse_o_" << i << "," << j;
                    plotLine(label.str().c_str(), Vec2d(0, inH), Vec2d(1, outH));
                    ImPlot::PopStyleColor();
                    label << "c";
                    float activation = outputWeights(i, j) * hiddenActivations[j] + outputWeights(i, kHiddenSize);
                    ImPlot::PushStyleColor(ImPlotCol_Line, mapColor(activation));
                    plotCircle<8>(label.str().c_str(), 1.f, outH, nodeRadius);
                    ImPlot::PopStyleColor();
                }

            }
        }
        ImPlot::EndPlot();
    }
    ImGui::End();
}