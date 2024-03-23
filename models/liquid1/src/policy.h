#pragma once

#include "Agent.h"
#include <math/noise.h>

#include <libs/eigen/Eigen/Core>

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
    // All the +1 below are to make room for an implicit bias term
    Eigen::Matrix<float, kHiddenSize, kNumInputs+1> inputWeights;
    Eigen::Matrix<float, kHiddenSize, kHiddenSize+1> hiddenWeights;
    Eigen::Matrix<float, kNumOutputs, kHiddenSize+1> outputWeights;
    Eigen::Vector<float, kNumInputs+1> inputVector;
    Eigen::Vector<float, kHiddenSize+1> inputActivations;
    Eigen::Vector<float, kHiddenSize+1> hiddenActivations;
    Eigen::Vector<float, kNumOutputs> outputActivations;

    void randomizeWeights(math::SquirrelRng& rng, float amplitude);
    MLPPolicy generateVariation(math::SquirrelRng& rng, float variationStep) const;
    void applyVariation(const MLPPolicy& delta, float scale);

    Action computeAction(math::SquirrelRng& rng, const DifferentialCart& agent, LinearTrack& track) override;

    void DrawActivations();

};