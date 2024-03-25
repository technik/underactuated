#pragma once

#include "Agent.h"
#include <math/noise.h>

#include <Eigen/Core>
#include <ml/nn/FullyConnectedNN.h>

struct LinearTrack;

struct CartPolicy
{
    using Action = DifferentialCart::Input;
    virtual Action computeAction(math::SquirrelRng& rng, const DifferentialCart& agent, LinearTrack& track) = 0;
};

struct RandomPolicy : CartPolicy
{
    Action computeAction(math::SquirrelRng& rng, const DifferentialCart& agent, LinearTrack& track) override;
};

struct LinearPolicy : CartPolicy
{
    static inline constexpr size_t kNumOutputs = 2;
    static inline constexpr size_t kNumInputs = 6; // 1 + state vector size
    Eigen::Matrix<float, kNumOutputs, kNumInputs> weights;
    Eigen::Vector<float, kNumInputs> inputVector;

    void randomizeWeights(math::SquirrelRng& rng, float amplitude);

    Action computeAction(math::SquirrelRng& rng, const DifferentialCart& agent, LinearTrack& track) override;

    void DrawActivations();
};

// Multilayer perceptron
struct MLPPolicy : CartPolicy
{
    static inline constexpr size_t kNumOutputs = 2;
    static inline constexpr size_t kNumInputs = 7;
    static inline constexpr size_t kHiddenSize = 16;
    
    nn::FullyConnectedNN m_network;
    using Matrix = nn::FullyConnectedNN::Matrix;

    Eigen::Vector<float, kNumInputs> inputVector;

    MLPPolicy()
    {
        m_network = nn::FullyConnectedNN(1, kNumInputs, kHiddenSize, kNumOutputs);
    }

    void randomizeWeights(math::SquirrelRng& rng, float amplitude);
    Matrix generateVariation(math::SquirrelRng& rng, float variationStep) const;
    void applyVariation(const Matrix& delta);

    Action computeAction(math::SquirrelRng& rng, const DifferentialCart& agent, LinearTrack& track) override;

    void DrawActivations();

};