#pragma once

#include "layer.h"
#include <math/noise.h>

namespace nn
{
    class FullyConnectedNN
    {
    public:
        using Scalar = float;
        using Layer = FullyConnectedLayer;
        using Matrix = Layer::Matrix;
        using ActivationFn = ReLu;

        std::vector<Layer> m_layers;
        ActivationFn m_activationFn;

        FullyConnectedNN() = default;
        FullyConnectedNN(size_t numHiddenLayers, size_t inputWidth, size_t hiddenLayerWidth, size_t outputWidth)
        {
            // Init weight layers.
            m_layers.resize(numHiddenLayers + 2);
            m_layers[0] = Layer(hiddenLayerWidth, inputWidth);
            for (size_t i = 0; i < numHiddenLayers; ++i)
            {
                m_layers[i + 1] = Layer(hiddenLayerWidth, hiddenLayerWidth);
            }
            m_layers.back() = Layer(outputWidth, hiddenLayerWidth);

            // Prealloc activation cache
            m_activationCache.resize(3 + numHiddenLayers);
            m_activationCache[0] = Matrix::Ones(inputWidth, 1);
            for (size_t i = 0; i < numHiddenLayers+1; ++i)
            {
                m_activationCache[i + 1] = Matrix::Ones(hiddenLayerWidth, 1);
            }
            m_activationCache.back() = Matrix::Ones(outputWidth, 1);
        }

        const Matrix& forward(const Matrix& x)
        {
            m_activationCache[0].topRows(x.rows()) = x;
            for (size_t i = 0; i < m_layers.size() - 1; ++i)
            {
                // Convolution
                m_activationCache[i + 1] = m_layers[i].forward(m_activationCache[i]);

                // Activation
                m_activationCache[i + 1] = m_activationFn.forward(m_activationCache[i + 1]);
            }

            // final layer, with no activation function
            m_activationCache.back() = m_layers.back().forward(m_activationCache[m_layers.size()-1]);
            return m_activationCache.back();
        }
        
        void randomize(math::SquirrelRng& rng)
        {
            for (auto& layer : m_layers)
                layer.randomize(rng);
        }
        
        Matrix randomDelta(math::SquirrelRng& rng, float stepSize) const
        {
            size_t gradRows = 0;
            size_t gradCols = 0;
            for (auto& layer : m_layers)
            {
                gradRows += layer.gradRows();
                gradCols = std::max(gradCols, layer.gradCols());
            }

            Matrix delta = Matrix::Zero(gradRows, gradCols);

            auto row0 = 0;
            for (auto& layer : m_layers)
            {
                delta.block(row0, 0, layer.gradRows(), layer.gradCols()) = layer.randomDelta(rng, stepSize);
                row0 += layer.gradRows();
            }

            return delta;
        }
        
        void step(Matrix delta)
        {
            auto row0 = 0;
            for (auto& layer : m_layers)
            {
                layer.step(delta.block(row0, 0, layer.gradRows(), layer.gradCols()));
                row0 += layer.gradRows();
            }
        }

        std::vector<Matrix> m_activationCache;
    };
} // namespace nn