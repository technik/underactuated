#pragma once

#include <Eigen/Core>
#include <math/noise.h>
#include <cassert>

namespace nn
{
	// Fully connected layer
	struct FullyConnectedLayer
	{
		// Local types
		using Scalar = float;
		using Matrix = Eigen::Matrix<Scalar, -1, -1>;

		size_t m, n;
		Matrix W;

		FullyConnectedLayer() = default;
		FullyConnectedLayer(size_t _m, size_t _n) // Init as an (M x N+1) matrix
		{
			m = _m;
			n = _n;
			W = Matrix(m, n+1); // n+1 to store a bias term
		}

		// The size of dW
		size_t gradRows() const { return m; }
		size_t gradCols() const { return n + 1; }

		// Forward evaluation
		Matrix forward(const Matrix& x)
		{
			assert(W.cols() == x.rows() + 1);
			m_xWithBias = Matrix::Ones(n + 1, 1);
			m_xWithBias.topRows(n) = x;
			return W * m_xWithBias;
		}

		// Back propagation
		Matrix d_dX(const Matrix& xWithBias)
		{
			return W.transpose();
		}

		Matrix d_dW(const Matrix& xWithBias)
		{
			return m_xWithBias.transpose();
		}

		void randomize(math::SquirrelRng& rng)
		{
			auto norm = 1.f / W.cols();
			for(auto i = 0; i < W.rows(); i++)
			{
				for (auto j = 0; j < W.cols(); ++j)
				{
					W(i, j) = rng.uniform(-norm, norm);
				}
			}
		}

		Matrix randomDelta(math::SquirrelRng& rng, float stepSize) const
		{
			Matrix delta = Matrix(m, n+1);

			for (auto i = 0; i < W.rows(); i++)
			{
				for (auto j = 0; j < W.cols(); ++j)
				{
					delta(i, j) = rng.uniform(-stepSize, stepSize);
				}
			}

			return delta;
		}

		void step(const Matrix& dir)
		{
			W += dir;
		}

	private:
		Matrix m_xWithBias;
	};

	struct ReLu
	{
		// Local types
		using Scalar = float;
		using Matrix = Eigen::Matrix<Scalar, -1, -1>;

		Matrix forward(const Matrix& x)
		{
			return x.cwiseMax(Scalar(0));
		}

		Matrix d_dW(const Matrix& x, const Matrix& dz) const
		{
			return Matrix(0, 0); // Fixed function. No weights
		}

		Matrix d_dX(const Matrix& x) const
		{
			Matrix res(x.rows(), x.cols());

			for (int i = 0; i < x.rows(); ++i)
			{
				for (int j = 0; j < x.cols(); ++j)
				{
					res(i, j) = (x(i, j) >= 0) ? Scalar(1) : Scalar(0);
				}
			}

			return res;
		}
	};
}