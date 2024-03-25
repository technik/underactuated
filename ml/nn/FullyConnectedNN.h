#pragma once

#include "layer.h"

namespace nn
{
    template<class S>
    class FullyConnectedNN
    {
    public:
        using Scalar = S;
        using Layer = FullyConnectedLayer<Scalar>;
        using Matrix = Layer::Matrix;
        using ActivationFn = ReLu<Scalar>;

        std::vector<Layer> m_layers;
        ActivationFn m_activationFn;

        FullyConnectedNN() = default;
        FullyConnectedNN(size_t numHiddenLayers, size_t inputWidth, size_t hiddenLayerWidth, size_t outputWidth)
        {
            // Init weight layers.
            // Each layer's input size is extended by one to accomodate a weight
            m_layers.resize(numHiddenLayers + 2);
            m_layers[i] = Matrix(hiddenLayerWidth, inputWidth + 1);
            for (size_t i = 0; i < numHiddenLayers; ++i)
            {
                layers[i + 1] = Matrix(hiddenLayerWidth, hiddenLayerWidth + 1);
            }
            layers.back() = Matrix(outputWidth, hiddenLayerWidth + 1);

            // Prealloc activation cache
            m_activationCache.resize(2 + numHiddenLayers);
            m_activationCache[0] = Matrix::Ones(inputWidth + 1, 1);
            for (size_t i = 0; i < numHiddenLayers; ++i)
            {
                m_activationCache[i + 1] = Matrix::Ones(hiddenLayerWidth + 1, 1);
            }
        }

        Matrix forward(const Matrix& x)
        {
            m_activationCache[0].TopRows(x.rows()) = x;
            for (size_t i = 0; i < m_layers.size() - 1; ++i)
            {
                // Convolution
                m_activationCache[i + 1].TopRows(hiddenLayerWidth) = m_layers[i] * m_activationCache[i];

                // Activation
                m_activationCache[i + 1] = m_activationFn.forward(m_activationCache[i + 1]);
            }

            // final layer, with no activation function
            return m_layers.back() * m_activationCache.back();
        }

    private:
        std::vector<Matrix> m_activationCache;
    };
}