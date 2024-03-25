#pragma once

#include <Eigen/Eigen>
#include <math/noise.h>

namespace nn
{
	template<class S>
	struct Layer
	{
		// Local types
		using Scalar = S;
		using Matrix = Eigen::Matrix<Scalar, -1, -1>;

		// The size of dW
		virtual size_t gradRows() const = 0;
		virtual size_t gradCols() const = 0;

		// Forward evaluation
		virtual Matrix forward(const Matrix& x) = 0;

		// Back propagation
		virtual Matrix d_dW(const Matrix& x) const = 0;
		virtual Matrix d_dX(const Matrix& x) const = 0;

		virtual void randomize(math::SquirrelRng& rng) = 0;
		virtual void step(Scalar stepSize, const Matrix& dir) = 0;
	};

	// Fully connected layer
	template<class S>
	struct FullyConnectedLayer final : Layer<S>
	{
		size_t m, n;
		Matrix W;

		FullyConnectedLayer() = default;
		FullyConnectedLayer(size_t _m, size_t _n) // Init as an (M x N) matrix
		{
			m = _m;
			n = _n;
			W = Eigen::Matrix(m, n+1); // n+1 to store a bias term
		}

		// The size of dW
		size_t gradRows() const override { return m; }
		size_t gradCols() const override { return n + 1; }

		// Forward evaluation
		Matrix forward(const Matrix& x)
		{
			assert(W.cols() == x.rows() + 1);
			m_xWithBias = Matrix::Ones(x.rows() + 1, x.cols());
			m_X.topRows(x.rows()) = x;
			return W * m_xWithBias;
		}

		// Back propagation
		Matrix d_dX(const Matrix& x) override
		{
			return W.transpose();
		}

		Matrix d_dW(const Matrix& x) override
		{
			return m_xWithBias.transpose();
		}

		void randomize(math::SquirrelRng& rng) override
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

		void step(Scalar stepSize, const Matrix& dir) override
		{
			W += stepSize * dir;
		}

	private:
		Matrix m_xWithBias;
	};

	template<class S>
	struct ReLu final : Layer<S>
	{
		size_t gradRows() const override { return 0; }
		size_t gradCols() const override { return 0; }

		Matrix forward(const Matrix& x) override {
			return x.cwiseMax(Scalar(0));
		}

		Matrix d_dW(const Matrix& x, const Matrix& dz) const override {
			return Matrix(0, 0); // Fixed function. No weights
		}

		Matrix d_dX(const Matrix& x) const {
			Matrix res(x.rows(), x.cols());
			for (int i = 0; i < x.rows(); ++i) {
				for (int j = 0; j < x.cols(); ++j) {
					res(i, j) = x(i, j) == fwd(i, j) ? Scalar(1) : Scalar(0);
				}
			}
			return res;
		}

		void randomize() override {}
		void step(Scalar, const Matrix&) override {}
	};
}