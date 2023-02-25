//-------------------------------------------------------------------------------------------------
// Toy path tracer
//--------------------------------------------------------------------------------------------------
// Copyright 2018 Carmelo J Fdez-Aguera
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software
// and associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or
// substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#pragma once

#include <cassert>
#include <initializer_list>
#include "aabb.h"
#include "vector.h"

#include <DirectXMath.h>

namespace math
{
	template<class T>
	struct Mat22
	{
		using Column = Vec2<T>;

		Mat22() = default;
		Mat22(T a00, T a01, T a10, T a11)
			: m{{a00,a10}, {a01,a11}}
		{}

		auto det() const
		{
			return m[0][0] * m[1][1] - m[1][0] * m[0][1];
		}

		Mat22 operator+(const Mat22& b) const
		{
			return Mat22(
				m[0][0] + b.m[0][0], m[0][1] + b.m[0][1],
				m[1][0] + b.m[1][0], m[1][1] + b.m[1][1]
			);
		}

		Mat22 operator-(const Mat22& b) const
		{
			return Mat22(
				m[0][0] + b.m[0][0], m[0][1] + b.m[0][1],
				m[1][0] + b.m[1][0], m[1][1] + b.m[1][1]
			);
		}

		Vec2<T> operator*(const Vec2<T>& v) const
		{
			Vec2<T> result;
			result[0] = m[0][0] * v[0] + m[1][0] * v[1];
			result[1] = m[0][0] * v[0] + m[1][0] * v[1];
			return result;
		}

		double operator()(size_t row, size_t col) const
		{
			return m[col][row];
		}

		double& operator()(size_t row, size_t col)
		{
			return m[col][row];
		}
	private:
		Column m[2];
	};

	template<class T>
	Mat22<T> operator*(const Mat22<T>& a, const Mat22<T>& b)
	{
		Mat22<T> result;
		result(0, 0) = a(0, 0) * b(0, 0) + a(0, 1) * b(1, 0);
		result(0, 1) = a(0, 0) * b(0, 1) + a(0, 1) * b(1, 1);
		result(1, 0) = a(1, 0) * b(0, 0) + a(1, 1) * b(1, 0);
		result(1, 1) = a(1, 0) * b(0, 1) + a(1, 1) * b(1, 1);
		return result;
	}

	template<class T>
	Vec2<T> operator*(const Vec2<T>& v, const Mat22<T>& m)
	{
		Vec2<T> result(
			v[0]*m(0,0) + v[1]*m(0,1),
			v[0]*m(1,0) + v[1]*m(1,1)
		);
		return result;
	}

	using Mat22d = Mat22<double>;

	class alignas(4 * sizeof(float)) Matrix34f
	{
	public:
		Matrix34f() = default;
		Matrix34f(std::initializer_list<float> il)
		{
			assert(il.size() == 12);

			auto iter = il.begin();
			for (size_t i = 0; i < il.size(); ++i)
				m[i] = *iter++;
		}

		Matrix34f(float x)
		{
			for(auto i = 0; i < 12; ++i)
				m[i] = x;
		}

		Matrix34f inverse() const;

		static Matrix34f identity()
		{
			Matrix34f x;
			for(int i = 0; i < 3; ++i)
				for(int j = 0; j < 4; ++j)
					x(i,j) = float(i==j?1:0);
			return x;
		}

		Vec3f& position() { return reinterpret_cast<Vec3f&>((*this)(0,3)); }
		Vec3f position() const {
			return col<3>();
		}

		Matrix34f operator*(const Matrix34f& b) const
		{
			Matrix34f res;
			for(int i = 0; i < 3; ++i)
			{
				for(int j = 0; j < 4; ++j)
				{
					res(i,j) =
						(*this)(i,0)*b(0,j) +
						(*this)(i,1)*b(1,j) +
						(*this)(i,2)*b(2,j);
				}
				res(i,3) += (*this)(i,3);
			}
			return res;
		}

        AABB operator*(const AABB& b) const
        {
            Vec3f origin = b.origin();
            Vec3f halfSize = b.max() - origin; // Positive by definition

            Vec3f ex = abs(col<0>() * halfSize.x());
            Vec3f ey = abs(col<1>() * halfSize.y());
            Vec3f ez = abs(col<2>() * halfSize.z());

            Vec3f extent = ex + ey + ez;

            origin = transformPos(origin);
            auto newMax = origin + extent;
            auto newMin = origin - extent;

            return AABB(newMin, newMax);
        }

		Vec3f transformPos(const Vec3f& v) const
		{
			Vec3f res;
			for(int i = 0; i < 3; ++i)
			{
				res[i] =
					(*this)(i,0)*v[0] +
					(*this)(i,1)*v[1] +
					(*this)(i,2)*v[2] +
					(*this)(i,3);
			}
			return res;
		}

        template<int i>
        const Vec3f& col() const
        {
            static_assert(i < 4);
            return reinterpret_cast<const Vec3f&>(m[3 * i]);
        }

        template<int i>
        Vec3f& col()
        {
            static_assert(i < 4);
            return reinterpret_cast<Vec3f&>(m[3 * i]);
        }

		Vec3f transformDir(const Vec3f& v) const
		{
			Vec3f res;
			for(int i = 0; i < 3; ++i)
			{
				res[i] =
					(*this)(i,0)*v[0] +
					(*this)(i,1)*v[1] +
					(*this)(i,2)*v[2];
			}
			return res;
		}

		float& operator()(int i, int j)
		{
			return m[3*j+i];
		}

		float operator()(int i, int j) const
		{
			return m[3*j+i];
		}

	private:
		// Column major storage
		float m[12];
	};

	class alignas(16 * sizeof(float)) Matrix44f
	{
	public:
		Matrix44f() = default;
		Matrix44f(std::initializer_list<float> il)
		{
			assert(il.size() == 16);

			auto iter = il.begin();
			for (size_t i = 0; i < il.size(); ++i)
				m[i] = *iter++;
		}
		Matrix44f(const Matrix44f&) = default;

		// Assuming an implicit (0,0,0,1) 4th row
		explicit Matrix44f(const Matrix34f& x)
		{
			*this = Matrix44f::identity();
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 4; ++j)
					(*this)(i, j) = x(i, j);
		}

		explicit Matrix44f(const std::array<float,16>& colMajorArray)
		{
			memcpy(m, colMajorArray.data(), sizeof(Matrix44f));
		}

		explicit Matrix44f(const DirectX::XMMATRIX& rowMajorMtx)
		{
			auto& colMajor = reinterpret_cast<DirectX::XMMATRIX&>(*this);
			colMajor = DirectX::XMMatrixTranspose(rowMajorMtx);
		}

		explicit operator DirectX::XMMATRIX() const
		{
			auto& colMajor = reinterpret_cast<const DirectX::XMMATRIX&>(*this);
			// Col-Major to Row-Major
			return DirectX::XMMatrixTranspose(colMajor);
		}

		static auto lowSolve(const Matrix44f& L, const Vec4f& y)
		{
			Vec4f x;
			for(int i = 0; i < 4; ++i)
			{
				assert(std::abs(L(i,i)) > 1e-4f);
				float accum = 0.f;
				for(int j = 0; j < i; ++j)
					accum += L(i,j)*x[j];
				x[i] = (y[i] - accum)/L(i,i);
			}
			return x;
		}

		static auto upSolve(const Matrix44f& U, const Vec4f& y)
		{
			Vec4f x;
			for(int i = 3; i >= 0; --i)
			{
				assert(std::abs(U(i,i)) > 1e-5f);
				float accum = 0.f;
				for(int j = i+1; j < 4; ++j)
					accum += U(i,j)*x[j];
				x[i] = (y[i] - accum)/U(i,i);
			}
			return x;
		}

		Matrix44f inverse() const
		{
			Matrix44f inv;
			Matrix44f L, U;
			std::array<int,4> P;
			factorizationLU(L,U,P);
			Matrix44f pb = Matrix44f(0.f);
			for(int i = 0; i < 4; ++i)
			{
				pb(i,P[i]) = 1.f;
			}
			for(int j = 0; j < 4; ++j)
			{
				Vec4f b(pb(0,j),pb(1,j),pb(2,j),pb(3,j));
				auto y = lowSolve(L,b);
				auto x = upSolve(U,y);
				for(auto i = 0; i < 4; ++i)
					inv(i,j) = x[i];
			}
			return inv;
		}

		bool operator== (const Matrix44f& x) const
		{
			for(int i = 0; i < 3; ++i)
				for(int j = 0; j < 4; ++j)
					if((*this)(i,j) != x(i,j)) return false;
			return true;
		}

		static Matrix44f identity()
		{
			Matrix44f x;
			for(int i = 0; i < 4; ++i)
				for(int j = 0; j < 4; ++j)
					x(i,j) = float(i==j?1:0);
			return x;
		}

		Matrix44f operator*(const Matrix44f& b) const
		{
			Matrix44f res;
			for(int i = 0; i < 4; ++i)
			{
				for(int j = 0; j < 4; ++j)
				{
					res(i,j) =
						(*this)(i,0)*b(0,j) +
						(*this)(i,1)*b(1,j) +
						(*this)(i,2)*b(2,j) +
						(*this)(i,3)*b(3,j);
				}
			}
			return res;
		}

		Vec4f operator*(const Vec4f& b) const
		{
			Vec4f res;
			for(int i = 0; i < 4; ++i)
			{
				res[i] =
					(*this)(i,0)*b[0] +
					(*this)(i,1)*b[1] +
					(*this)(i,2)*b[2] +
					(*this)(i,3)*b[3];
			}
			return res;
		}

		float& operator()(int i, int j)
		{
			return m[4*j+i];
		}

		float operator()(int i, int j) const
		{
			return m[4*j+i];
		}

		float element(int i, int j) const
		{
			return m[4*j+i];
		}

		// Apply gauss elimination with partial pivoting
		void factorizationLU(Matrix44f& L, Matrix44f& U, std::array<int, 4>& P) const;

	private:
		// Column major storage
		float m[16];
	};

	inline Matrix44f transpose(const Matrix44f x)
	{
		Matrix44f result;
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				result(i,j) = x(j,i);
			}
		}
		return result;
	}

	inline Matrix34f Matrix34f::inverse() const
	{
		Matrix34f inv;
		auto x = Matrix44f(*this);
		auto xi = x.inverse();
		for(int j = 0; j < 4; ++j)
			for(int i = 0; i < 3; ++i)
				inv(i,j) = xi(i,j);
		return inv;
	}

	using Mat34f = Matrix34f;
	using Mat44f = Matrix44f;

} // namespace math
