/*
===============================================================================
 Matrix & Vector Math Convention
===============================================================================
 - Matrices are stored in row-major order.
 - Vectors are treated as row vectors (i.e., transformations are applied as: vec * Mat).
 - Matrix multiplication is done in left-to-right order: M * V * P.
 - Shader-side multiplication must match this: gl_Position = pos * u_mvp.
 - To send matrices to OpenGL (which expects column-major layout), transpose the matrix before uploading:
	 glUniformMatrix4fv(..., GL_FALSE, matrix.transpose().data());
===============================================================================
*/
#pragma once
#define ENGINE_MATH_SIMD // enable SIMD math
#ifndef ENGINE_MATH_SIMD
#if defined(__AVX2__)
#define ENGINE_MATH_SIMD
#endif
#endif

#if defined(ENGINE_MATH_SIMD) && !defined(__AVX2__)
#error "ERROR::PREPROCESSOR::MATH::DEFINED::ENGINE_MATH_SIMD::BUT_AVX_NOT_SUPPORTED!"
#endif

#include <stdexcept> 
#include <immintrin.h>
#include <cmath> 

namespace engine::math
{
	// foward declartion of Vec4
	struct alignas(16) Vec4f;

	struct alignas(16) Vec3f
	{
		union
		{
			struct { float x, y, z, _pad; }; // Add padding for alignment
			struct { float r, g, b, _pad2; };
			struct { float _a, _b, _c, _pad3; };
			__m128 simd;
		};

		Vec3f() : simd(_mm_setzero_ps()) {}
		Vec3f(float x, float y, float z) : simd(_mm_setr_ps(x, y, z, 0.0f)) {}
		Vec3f(__m128 simdValue) : simd(simdValue) {}

		Vec3f(const Vec4f& vec4); // constructor pre-declaration
		// Mutable access
		inline float& operator[](int idx)
		{
			switch (idx)
			{
			case 0: return x;
			case 1: return y;
			case 2: return z;
			default: throw std::out_of_range("ERROR::ENGINE::MATH::VEC3F::INDEX_OUT_OF_RANGE");
			}
		}

		inline const float& operator[](int idx) const
		{
			switch (idx)
			{
			case 0: return x;
			case 1: return y;
			case 2: return z;
			default: throw std::out_of_range("ERROR::ENGINE::MATH::VEC3F::INDEX_OUT_OF_RANGE");
			}
		}

		// simd access
		inline __m128 simdValue() const { return simd; }
		inline void setSimd(__m128 val) { simd = val; }

		inline bool operator==(const Vec3f& other) const
		{
#ifndef ENGINE_MATH_SIMD
			return x == other.x && y == other.y && z == other.z;
#else
			// Use epsilon comparison for floating point
			const float epsilon = 1e-6f;
			__m128 diff = _mm_sub_ps(simd, other.simd);
			__m128 abs_diff = _mm_andnot_ps(_mm_set1_ps(-0.0f), diff);
			__m128 eps = _mm_set1_ps(epsilon);
			__m128 cmp = _mm_cmple_ps(abs_diff, eps);
			return (_mm_movemask_ps(cmp) & 0x7) == 0x7;
#endif
		}

		inline bool operator<(const Vec3f& other) const
		{
			if (x != other.x) return x < other.x;
			if (y != other.y) return y < other.y;
			return z < other.z;
		}

		inline float dot(const Vec3f& other) const
		{
#ifndef ENGINE_MATH_SIMD
			return x * other.x + y * other.y + z * other.z;
#else
			__m128 mul = _mm_mul_ps(simd, other.simd);
			__m128 sum1 = _mm_hadd_ps(mul, mul);
			__m128 sum2 = _mm_hadd_ps(sum1, sum1);
			return _mm_cvtss_f32(sum2);
#endif
		}

		inline Vec3f cross(const Vec3f& other) const
		{
#ifndef ENGINE_MATH_SIMD
			return Vec3f(
				y * other.z - z * other.y,
				z * other.x - x * other.z,
				x * other.y - y * other.x
			);
#else
			// SIMD cross product
			__m128 a_yzx = _mm_shuffle_ps(simd, simd, _MM_SHUFFLE(3, 0, 2, 1));
			__m128 b_yzx = _mm_shuffle_ps(other.simd, other.simd, _MM_SHUFFLE(3, 0, 2, 1));
			__m128 a_zxy = _mm_shuffle_ps(simd, simd, _MM_SHUFFLE(3, 1, 0, 2));
			__m128 b_zxy = _mm_shuffle_ps(other.simd, other.simd, _MM_SHUFFLE(3, 1, 0, 2));

			return Vec3f(_mm_sub_ps(_mm_mul_ps(a_yzx, b_zxy), _mm_mul_ps(a_zxy, b_yzx)));
#endif
		}


		Vec3f xyz()
		{
			return{x,y,z};
		}
	};

#ifndef ENGINE_MATH_SIMD
	inline Vec3f operator-(const Vec3f& a, const Vec3f& b)
	{
		return Vec3f(a.x - b.x, a.y - b.y, a.z - b.z);
	}

	inline Vec3f operator+(const Vec3f& a, const Vec3f& b)
	{
		return Vec3f(a.x + b.x, a.y + b.y, a.z + b.z);
	}

	inline Vec3f operator*(const Vec3f& a, float scalar)
	{
		return Vec3f(a.x * scalar, a.y * scalar, a.z * scalar);
	}

	inline Vec3f operator*(float scalar, const Vec3f& a)
	{
		return a * scalar;
	}
#else
	inline Vec3f operator+(const Vec3f& a, const Vec3f& b)
	{
		return Vec3f(_mm_add_ps(a.simdValue(), b.simdValue()));
	}

	inline Vec3f operator-(const Vec3f& a, const Vec3f& b)
	{
		return Vec3f(_mm_sub_ps(a.simdValue(), b.simdValue()));
	}

	inline Vec3f operator*(const Vec3f& a, float scalar)
	{
		__m128 s = _mm_set1_ps(scalar);
		return Vec3f(_mm_mul_ps(a.simdValue(), s));
	}

	inline Vec3f operator*(float scalar, const Vec3f& a)
	{
		return a * scalar;
	}

	// Standalone dot product function
	inline float dot(const Vec3f& a, const Vec3f& b)
	{
#ifndef ENGINE_MATH_SIMD
		return a.x * b.x + a.y * b.y + a.z * b.z;
#else
		__m128 mul = _mm_mul_ps(a.simdValue(), b.simdValue());
		__m128 sum1 = _mm_hadd_ps(mul, mul);
		__m128 sum2 = _mm_hadd_ps(sum1, sum1);
		return _mm_cvtss_f32(sum2);
#endif
	}

	// Standalone length function
	inline float length(const Vec3f& a)
	{
		return sqrt(dot(a, a));
	}

	// Standalone normalize function
	inline Vec3f normalize(const Vec3f& a)
	{
		float len = length(a);
		if (len < 1e-6f) return Vec3f(0, 0, 0);
		return a * (1.0f / len);
	}

	// Standalone cross product function
	inline Vec3f cross(const Vec3f& a, const Vec3f& b)
	{
#ifndef ENGINE_MATH_SIMD
		return Vec3f(
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x
		);
#else
		__m128 a_yzx = _mm_shuffle_ps(a.simdValue(), a.simdValue(), _MM_SHUFFLE(3, 0, 2, 1));
		__m128 b_yzx = _mm_shuffle_ps(b.simdValue(), b.simdValue(), _MM_SHUFFLE(3, 0, 2, 1));
		__m128 a_zxy = _mm_shuffle_ps(a.simdValue(), a.simdValue(), _MM_SHUFFLE(3, 1, 0, 2));
		__m128 b_zxy = _mm_shuffle_ps(b.simdValue(), b.simdValue(), _MM_SHUFFLE(3, 1, 0, 2));

		return Vec3f(_mm_sub_ps(_mm_mul_ps(a_yzx, b_zxy), _mm_mul_ps(a_zxy, b_yzx)));
#endif
	}
#endif


	struct Vec3fHash
	{
		std::size_t operator()(const Vec3f& v) const noexcept
		{
#ifdef ENGINE_MATH_SIMD
			/* Re-interpret the three float lanes as 32-bit integers.           */
			__m128i bits = _mm_castps_si128(v.simd);    // [x y z _pad]
			uint32_t x = _mm_extract_epi32(bits, 0);
			uint32_t y = _mm_extract_epi32(bits, 1);
			uint32_t z = _mm_extract_epi32(bits, 2);
#else
			uint32_t x; std::memcpy(&x, &v.x, 4);
			uint32_t y; std::memcpy(&y, &v.y, 4);
			uint32_t z; std::memcpy(&z, &v.z, 4);
#endif
			auto mix32 = [](uint32_t h) -> uint32_t
				{
					h += 0x9E3779B9u;					// 1. add golden-ratio step Knuth’s multiplicative constant
					h = (h ^ (h >> 16)) * 0x85EBCA6Bu;	// 2. mix high-/low-half and multiply by first odd constant One of the “magic” odd numbers
					h = (h ^ (h >> 13)) * 0xC2B2AE35u;	// 3. mix again, multiply by second odd constant The second MurmurHash 3 constant
					return h ^ (h >> 16);				// 4. final avalanche
				};

			uint32_t h = mix32(x);
			h = mix32(h ^ y);
			h = mix32(h ^ z);
			return static_cast<std::size_t>(h);
		}
	};


	struct alignas(16) Vec4f
	{
		union
		{
			struct { float x, y, z, w; };
			struct { float r, g, b, a; };
			struct { float _a, _b, _c, _d;};
			__m128 simd;
		};

		Vec4f() : simd(_mm_setzero_ps()) {}
		Vec4f(float x, float y, float z, float w) : simd(_mm_setr_ps(x, y, z, w)) {}
		Vec4f(__m128 simdValue) : simd(simdValue) {}

		Vec4f(Vec3f vec3, float w_val) 
		{
			x = vec3.x;
			y = vec3.y;
			z = vec3.z;
			this->w = w_val; 
		}

		Vec4f(Vec3f vec3)
		{
			x = vec3.x;
			y = vec3.y;
			z = vec3.z;
			this->w = 1.0f; 
		}

		// Mutable access
		inline float& operator[](int idx)
		{
			switch (idx)
			{
			case 0: return x;
			case 1: return y;
			case 2: return z;
			case 3: return w;
			default: throw std::out_of_range("ERROR::ENGINE::MATH::VEC4F::INDEX_OUT_OF_RANGE");
			}
		}

		inline const float& operator[](int idx) const
		{
			switch (idx)
			{
			case 0: return x;
			case 1: return y;
			case 2: return z;
			case 3: return w;
			default: throw std::out_of_range("ERROR::ENGINE::MATH::VEC4F::INDEX_OUT_OF_RANGE");
			}
		}

		// simd access
		inline __m128 simdValue() const { return simd; }
		inline void setSimd(__m128 val) { simd = val; }

		inline bool operator==(const Vec4f& other) const
		{
#ifndef ENGINE_MATH_SIMD
			return x == other.x && y == other.y && z == other.z && w == other.w;
#else
			// Use epsilon comparison for floating point
			const float epsilon = 1e-6f;
			__m128 diff = _mm_sub_ps(simd, other.simd);
			__m128 abs_diff = _mm_andnot_ps(_mm_set1_ps(-0.0f), diff);
			__m128 eps = _mm_set1_ps(epsilon);
			__m128 cmp = _mm_cmple_ps(abs_diff, eps);
			return _mm_movemask_ps(cmp) == 0xF;
#endif
		}

		inline bool operator<(const Vec4f& other) const
		{
			if (x != other.x) return x < other.x;
			if (y != other.y) return y < other.y;
			if (z != other.z) return z < other.z;
			return w < other.w;
		}

		inline float dot(const Vec4f& other) const
		{
#ifndef ENGINE_MATH_SIMD
			return x * other.x + y * other.y + z * other.z + w * other.w;
#else
			__m128 mul = _mm_mul_ps(simd, other.simd);
			__m128 sum1 = _mm_hadd_ps(mul, mul);
			__m128 sum2 = _mm_hadd_ps(sum1, sum1);
			return _mm_cvtss_f32(sum2);
#endif
		}

		// Cross product treating Vec4f as homogeneous 3D vector (ignoring w component)
		inline Vec4f cross(const Vec4f& other) const
		{
#ifndef ENGINE_MATH_SIMD
			return Vec4f(
				y * other.z - z * other.y,
				z * other.x - x * other.z,
				x * other.y - y * other.x,
				0.0f  // w component set to 0 for direction vector
			);
#else
			// SIMD cross product (same as Vec3f but with w=0)
			__m128 a_yzx = _mm_shuffle_ps(simd, simd, _MM_SHUFFLE(3, 0, 2, 1));
			__m128 b_yzx = _mm_shuffle_ps(other.simd, other.simd, _MM_SHUFFLE(3, 0, 2, 1));
			__m128 a_zxy = _mm_shuffle_ps(simd, simd, _MM_SHUFFLE(3, 1, 0, 2));
			__m128 b_zxy = _mm_shuffle_ps(other.simd, other.simd, _MM_SHUFFLE(3, 1, 0, 2));

			__m128 result = _mm_sub_ps(_mm_mul_ps(a_yzx, b_zxy), _mm_mul_ps(a_zxy, b_yzx));
			// Set w component to 0
			result = _mm_insert_ps(result, _mm_setzero_ps(), 0x30);
			return Vec4f(result);
#endif
		}

		Vec4f xyzw()
		{
			return{ x,y,z,w };
		}

		Vec3f xyz()
		{
			return{ x,y,z};
		}
	};

#ifndef ENGINE_MATH_SIMD
	inline Vec4f operator+(const Vec4f& a, const Vec4f& b)
	{
		return Vec4f(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
	}

	inline Vec4f operator-(const Vec4f& a, const Vec4f& b)
	{
		return Vec4f(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
	}

	inline Vec4f operator*(const Vec4f& a, float scalar)
	{
		return Vec4f(a.x * scalar, a.y * scalar, a.z * scalar, a.w * scalar);
	}

	inline Vec4f operator*(float scalar, const Vec4f& a)
	{
		return a * scalar;
	}

	//Vec4f / scalar
	inline Vec4f operator/(const Vec4f& a, float s)
	{
		const float inv = 1.0f / s;
		return Vec4f(a.x * inv, a.y * inv, a.z * inv, a.w * inv);
	}

	//scalar / Vec4f
	inline Vec4f operator/(float s, const Vec4f& a)
	{
		return Vec4f(s / a.x, s / a.y, s / a.z, s / a.w);
	}


	inline Vec4f operator*(const Vec4f& v, const Mat4f& M)
	{
		Vec4f r;
		r.x = v.x * M[0][0] + v.y * M[1][0] + v.z * M[2][0] + v.w * M[3][0];
		r.y = v.x * M[0][1] + v.y * M[1][1] + v.z * M[2][1] + v.w * M[3][1];
		r.z = v.x * M[0][2] + v.y * M[1][2] + v.z * M[2][2] + v.w * M[3][2];
		r.w = v.x * M[0][3] + v.y * M[1][3] + v.z * M[2][3] + v.w * M[3][3];
		return r;
	}

#else
	inline Vec4f operator+(const Vec4f& a, const Vec4f& b)
	{
		return Vec4f(_mm_add_ps(a.simdValue(), b.simdValue()));
	}

	inline Vec4f operator-(const Vec4f& a, const Vec4f& b)
	{
		return Vec4f(_mm_sub_ps(a.simdValue(), b.simdValue()));
	}

	inline Vec4f operator*(const Vec4f& a, float scalar)
	{
		__m128 s = _mm_set1_ps(scalar);
		return Vec4f(_mm_mul_ps(a.simdValue(), s));
	}


	inline Vec4f operator*(float scalar, const Vec4f& a)
	{
		return a * scalar;
	}



	inline Vec4f operator/(const Vec4f& a, float s)
	{
		/* broadcast 1/s and multiply – prefer _mm_set1_ps for load latency */
		__m128 inv = _mm_set1_ps(1.0f / s);
		return Vec4f(_mm_mul_ps(a.simdValue(), inv));
	}

	inline Vec4f operator/(float s, const Vec4f& a)
	{
		__m128 num = _mm_set1_ps(s);
		/* reciprocal approximation + Newton-Raphson for accuracy */
		__m128 rcp = _mm_rcp_ps(a.simdValue());
		// one NR iteration : rcp = rcp * (2 - a*rcp)
		rcp = _mm_mul_ps(rcp,
			_mm_sub_ps(_mm_set1_ps(2.0f),
				_mm_mul_ps(a.simdValue(), rcp)));
		return Vec4f(_mm_mul_ps(num, rcp));
	}

	// Standalone dot product function
	inline float dot(const Vec4f& a, const Vec4f& b)
	{
#ifndef ENGINE_MATH_SIMD
		return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
#else
		__m128 mul = _mm_mul_ps(a.simdValue(), b.simdValue());
		__m128 sum1 = _mm_hadd_ps(mul, mul);
		__m128 sum2 = _mm_hadd_ps(sum1, sum1);
		return _mm_cvtss_f32(sum2);
#endif
	}

	// Standalone length function (single vector)
	inline float length(const Vec4f& a)
	{
		return sqrt(dot(a, a));
	}

	// Standalone normalize function (single vector)
	inline Vec4f normalize(const Vec4f& a)
	{
		float len = length(a);
		if (len < 1e-6f) return Vec4f(0, 0, 0, 0);
		return a * (1.0f / len);
	}

	// Standalone cross product function
	inline Vec4f cross(const Vec4f& a, const Vec4f& b)
	{
#ifndef ENGINE_MATH_SIMD
		return Vec4f(
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x,
			0.0f
		);
#else
		__m128 a_yzx = _mm_shuffle_ps(a.simdValue(), a.simdValue(), _MM_SHUFFLE(3, 0, 2, 1));
		__m128 b_yzx = _mm_shuffle_ps(b.simdValue(), b.simdValue(), _MM_SHUFFLE(3, 0, 2, 1));
		__m128 a_zxy = _mm_shuffle_ps(a.simdValue(), a.simdValue(), _MM_SHUFFLE(3, 1, 0, 2));
		__m128 b_zxy = _mm_shuffle_ps(b.simdValue(), b.simdValue(), _MM_SHUFFLE(3, 1, 0, 2));

		__m128 result = _mm_sub_ps(_mm_mul_ps(a_yzx, b_zxy), _mm_mul_ps(a_zxy, b_yzx));
		result = _mm_insert_ps(result, _mm_setzero_ps(), 0x30);
		return Vec4f(result);
#endif
	}
#endif

	struct Vec4fHash
	{
		std::size_t operator()(const Vec4f& v) const noexcept
		{
			/* ---- 1. extract the raw 32-bit float patterns ---- */
			uint32_t wBits, xBits, yBits, zBits;

#ifdef ENGINE_MATH_SIMD
			__m128i bits = _mm_castps_si128(v.simd);   // treat lanes as int32
			xBits = _mm_extract_epi32(bits, 0);
			yBits = _mm_extract_epi32(bits, 1);
			zBits = _mm_extract_epi32(bits, 2);
			wBits = _mm_extract_epi32(bits, 3);
#else
			std::memcpy(&xBits, &v.x, 4);
			std::memcpy(&yBits, &v.y, 4);
			std::memcpy(&zBits, &v.z, 4);
			std::memcpy(&wBits, &v.w, 4);
#endif

			/* ---- 2. SplitMix32 mixer (same as in Vec3fHash) ---- */
			auto mix32 = [](uint32_t h) -> uint32_t
				{
					h += 0x9E3779B9u;                 // golden-ratio increment
					h = (h ^ (h >> 16)) * 0x85EBCA6Bu;
					h = (h ^ (h >> 13)) * 0xC2B2AE35u;
					return h ^ (h >> 16);
				};

			/* ---- 3. combine the four lanes ---- */
			uint32_t h = mix32(xBits);
			h = mix32(h ^ yBits);
			h = mix32(h ^ zBits);
			h = mix32(h ^ wBits);

			return static_cast<std::size_t>(h);
		}
	};

	// Normalize a plane (nx, ny, nz, d) so ||n|| = 1, scaling d accordingly.
	// If the normal is near zero, returns {0,0,0,0}.
	inline engine::math::Vec4f normalizePlane(const engine::math::Vec4f& p)
	{
#ifndef ENGINE_MATH_SIMD
		const float nx = p.x, ny = p.y, nz = p.z, d = p.w;
		const float len2 = nx * nx + ny * ny + nz * nz;
		if (len2 <= 1e-20f) return { 0.0f, 0.0f, 0.0f, 0.0f };
		const float invLen = 1.0f / std::sqrt(len2);
		return { nx * invLen, ny * invLen, nz * invLen, d * invLen };
#else
		const __m128 v = _mm_setr_ps(p.x, p.y, p.z, p.w);
		const __m128 n = _mm_setr_ps(p.x, p.y, p.z, 0.0f);

		// len2 = dot3(n, n)
#if defined(__SSE4_1__) || defined(__AVX2__)
		const __m128 len2v = _mm_dp_ps(n, n, 0x71);                 // sum xyz -> x
		const float   len2 = _mm_cvtss_f32(len2v);
		if (len2 <= 1e-20f) return { 0.0f, 0.0f, 0.0f, 0.0f };
		__m128 len2b = _mm_shuffle_ps(len2v, len2v, _MM_SHUFFLE(0, 0, 0, 0)); // broadcast
#else
		__m128 m = _mm_mul_ps(n, n);
		__m128 sh1 = _mm_shuffle_ps(m, m, _MM_SHUFFLE(3, 0, 2, 1));
		__m128 s = _mm_add_ps(m, sh1);
		sh1 = _mm_shuffle_ps(s, s, _MM_SHUFFLE(3, 0, 2, 1));
		s = _mm_add_ps(s, sh1);
		const float len2 = _mm_cvtss_f32(s);
		if (len2 <= 1e-20f) return { 0.0f, 0.0f, 0.0f, 0.0f };
		__m128 len2b = _mm_shuffle_ps(s, s, _MM_SHUFFLE(0, 0, 0, 0));  // broadcast
#endif

		// invLen ? rsqrt(len2) with one Newton–Raphson refinement
		__m128 inv = _mm_rsqrt_ps(len2b);
#if defined(__FMA__) || defined(__AVX2__)
		const __m128 half = _mm_set1_ps(0.5f);
		const __m128 thrh = _mm_set1_ps(1.5f);
		const __m128 inv2 = _mm_mul_ps(inv, inv);
		const __m128 term = _mm_sub_ps(thrh, _mm_mul_ps(half, _mm_mul_ps(len2b, inv2)));
		inv = _mm_mul_ps(inv, term);
#else
		const __m128 half = _mm_set1_ps(0.5f);
		const __m128 thrh = _mm_set1_ps(1.5f);
		const __m128 inv2 = _mm_mul_ps(inv, inv);
		const __m128 term = _mm_sub_ps(thrh, _mm_mul_ps(half, _mm_mul_ps(len2b, inv2)));
		inv = _mm_mul_ps(inv, term);
#endif

		// Scale [nx, ny, nz, d] by invLen
		const __m128 invScalar = _mm_shuffle_ps(inv, inv, _MM_SHUFFLE(0, 0, 0, 0));
		const __m128 out = _mm_mul_ps(v, invScalar);

		alignas(16) float t[4];
		_mm_store_ps(t, out);
		return { t[0], t[1], t[2], t[3] };
#endif
	}


	// ===  helpers for the degeneracy/flip guards =======================

	inline float dot3_xyz(const engine::math::Vec4f& a, const engine::math::Vec4f& b)
	{
#ifndef ENGINE_MATH_SIMD
		return a.x * b.x + a.y * b.y + a.z * b.z;
#else
		const __m128 va = _mm_setr_ps(a.x, a.y, a.z, 0.0f);
		const __m128 vb = _mm_setr_ps(b.x, b.y, b.z, 0.0f);
#if defined(__SSE4_1__) || defined(__AVX2__)
		return _mm_cvtss_f32(_mm_dp_ps(va, vb, 0x71)); // sum xyz -> x
#else
		__m128 m = _mm_mul_ps(va, vb);
		__m128 sh1 = _mm_shuffle_ps(m, m, _MM_SHUFFLE(3, 0, 2, 1));
		__m128 s = _mm_add_ps(m, sh1);
		sh1 = _mm_shuffle_ps(s, s, _MM_SHUFFLE(3, 0, 2, 1));
		s = _mm_add_ps(s, sh1);
		return _mm_cvtss_f32(s);
#endif
#endif
	}

	inline engine::math::Vec4f cross3_xyz(const engine::math::Vec4f& a, const engine::math::Vec4f& b)
	{
#ifndef ENGINE_MATH_SIMD
		return { a.y * b.z - a.z * b.y,
				 a.z * b.x - a.x * b.z,
				 a.x * b.y - a.y * b.x,
				 0.0f };
#else
		const __m128 va = _mm_setr_ps(a.x, a.y, a.z, 0.0f);
		const __m128 vb = _mm_setr_ps(b.x, b.y, b.z, 0.0f);

		const __m128 a_yzx = _mm_shuffle_ps(va, va, _MM_SHUFFLE(3, 0, 2, 1));
		const __m128 b_zxy = _mm_shuffle_ps(vb, vb, _MM_SHUFFLE(3, 1, 0, 2));
		const __m128 a_zxy = _mm_shuffle_ps(va, va, _MM_SHUFFLE(3, 1, 0, 2));
		const __m128 b_yzx = _mm_shuffle_ps(vb, vb, _MM_SHUFFLE(3, 0, 2, 1));

#if defined(__FMA__) || defined(__AVX2__)
		__m128 cr = _mm_fmsub_ps(a_yzx, b_zxy, _mm_mul_ps(a_zxy, b_yzx));
#else
		__m128 cr = _mm_sub_ps(_mm_mul_ps(a_yzx, b_zxy), _mm_mul_ps(a_zxy, b_yzx));
#endif

		alignas(16) float t[4];
		_mm_store_ps(t, cr);
		return { t[0], t[1], t[2], 0.0f };
#endif
	}

	// Squared length of the 3D difference (|p-q|^2)
	inline float len2_xyz(const engine::math::Vec4f& p, const engine::math::Vec4f& q)
	{
#ifndef ENGINE_MATH_SIMD
		const float dx = p.x - q.x, dy = p.y - q.y, dz = p.z - q.z;
		return dx * dx + dy * dy + dz * dz;
#else
		const __m128 vp = _mm_setr_ps(p.x, p.y, p.z, 0.0f);
		const __m128 vq = _mm_setr_ps(q.x, q.y, q.z, 0.0f);
		const __m128 d = _mm_sub_ps(vp, vq);
#if defined(__SSE4_1__) || defined(__AVX2__)
		return _mm_cvtss_f32(_mm_dp_ps(d, d, 0x71));
#else
		__m128 m = _mm_mul_ps(d, d);
		__m128 sh1 = _mm_shuffle_ps(m, m, _MM_SHUFFLE(3, 0, 2, 1));
		__m128 s = _mm_add_ps(m, sh1);
		sh1 = _mm_shuffle_ps(s, s, _MM_SHUFFLE(3, 0, 2, 1));
		s = _mm_add_ps(s, sh1);
		return _mm_cvtss_f32(s);
#endif
#endif
	}

	// (2*area)^2 for triangle (a,b,c) = |cross(b-a, c-a)|^2
	inline float area2_sq(const engine::math::Vec4f& a,
		const engine::math::Vec4f& b,
		const engine::math::Vec4f& c)
	{
#ifndef ENGINE_MATH_SIMD
		auto ab = b - a, ac = c - a;
		auto cr = cross3_xyz(ab, ac);
		return cr.x * cr.x + cr.y * cr.y + cr.z * cr.z;
#else
		const __m128 va = _mm_setr_ps(a.x, a.y, a.z, 0.0f);
		const __m128 vb = _mm_setr_ps(b.x, b.y, b.z, 0.0f);
		const __m128 vc = _mm_setr_ps(c.x, c.y, c.z, 0.0f);

		const __m128 ab = _mm_sub_ps(vb, va);
		const __m128 ac = _mm_sub_ps(vc, va);

		// cross(ab, ac)
		const __m128 ab_yzx = _mm_shuffle_ps(ab, ab, _MM_SHUFFLE(3, 0, 2, 1));
		const __m128 ac_zxy = _mm_shuffle_ps(ac, ac, _MM_SHUFFLE(3, 1, 0, 2));
		const __m128 ab_zxy = _mm_shuffle_ps(ab, ab, _MM_SHUFFLE(3, 1, 0, 2));
		const __m128 ac_yzx = _mm_shuffle_ps(ac, ac, _MM_SHUFFLE(3, 0, 2, 1));
#if defined(__FMA__) || defined(__AVX2__)
		const __m128 cr = _mm_fmsub_ps(ab_yzx, ac_zxy, _mm_mul_ps(ab_zxy, ac_yzx));
#else
		const __m128 cr = _mm_sub_ps(_mm_mul_ps(ab_yzx, ac_zxy), _mm_mul_ps(ab_zxy, ac_yzx));
#endif

		// dot3(cr, cr)
#if defined(__SSE4_1__) || defined(__AVX2__)
		return _mm_cvtss_f32(_mm_dp_ps(cr, cr, 0x71));
#else
		__m128 m = _mm_mul_ps(cr, cr);
		__m128 sh1 = _mm_shuffle_ps(m, m, _MM_SHUFFLE(3, 0, 2, 1));
		__m128 s = _mm_add_ps(m, sh1);
		sh1 = _mm_shuffle_ps(s, s, _MM_SHUFFLE(3, 0, 2, 1));
		s = _mm_add_ps(s, sh1);
		return _mm_cvtss_f32(s);
#endif
#endif
	}

	// Unit normal of triangle (a,b,c); returns {nx,ny,nz,0}. If degenerate -> {0,0,0,0}
	inline engine::math::Vec4f unitNormal_xyz(const engine::math::Vec4f& a,
		const engine::math::Vec4f& b,
		const engine::math::Vec4f& c)
	{
#ifndef ENGINE_MATH_SIMD
		auto ab = b - a, ac = c - a;
		auto n = cross3_xyz(ab, ac);
		const float L2 = n.x * n.x + n.y * n.y + n.z * n.z;
		if (L2 <= 1e-20f) return { 0,0,0,0 };
		const float invL = 1.0f / std::sqrt(L2);
		return { n.x * invL, n.y * invL, n.z * invL, 0.0f };
#else
		const __m128 va = _mm_setr_ps(a.x, a.y, a.z, 0.0f);
		const __m128 vb = _mm_setr_ps(b.x, b.y, b.z, 0.0f);
		const __m128 vc = _mm_setr_ps(c.x, c.y, c.z, 0.0f);

		const __m128 ab = _mm_sub_ps(vb, va);
		const __m128 ac = _mm_sub_ps(vc, va);

		// cross(ab, ac)
		const __m128 ab_yzx = _mm_shuffle_ps(ab, ab, _MM_SHUFFLE(3, 0, 2, 1));
		const __m128 ac_zxy = _mm_shuffle_ps(ac, ac, _MM_SHUFFLE(3, 1, 0, 2));
		const __m128 ab_zxy = _mm_shuffle_ps(ab, ab, _MM_SHUFFLE(3, 1, 0, 2));
		const __m128 ac_yzx = _mm_shuffle_ps(ac, ac, _MM_SHUFFLE(3, 0, 2, 1));
#if defined(__FMA__) || defined(__AVX2__)
		__m128 n = _mm_fmsub_ps(ab_yzx, ac_zxy, _mm_mul_ps(ab_zxy, ac_yzx));
#else
		__m128 n = _mm_sub_ps(_mm_mul_ps(ab_yzx, ac_zxy), _mm_mul_ps(ab_zxy, ac_yzx));
#endif

		// L2 = dot3(n,n)
#if defined(__SSE4_1__) || defined(__AVX2__)
		__m128 L2v = _mm_dp_ps(n, n, 0x71);
#else
		__m128 m = _mm_mul_ps(n, n);
		__m128 sh1 = _mm_shuffle_ps(m, m, _MM_SHUFFLE(3, 0, 2, 1));
		__m128 s = _mm_add_ps(m, sh1);
		sh1 = _mm_shuffle_ps(s, s, _MM_SHUFFLE(3, 0, 2, 1));
		__m128 L2v = _mm_add_ps(s, sh1);
#endif

		const float L2 = _mm_cvtss_f32(L2v);
		if (L2 <= 1e-20f) return { 0,0,0,0 };

		// invL = rsqrt(L2) with one Newton–Raphson refinement
		__m128 L2b = _mm_shuffle_ps(L2v, L2v, _MM_SHUFFLE(0, 0, 0, 0)); // broadcast
		__m128 inv = _mm_rsqrt_ps(L2b);
		// inv = inv * (1.5 - 0.5*L2*inv*inv)
#if defined(__FMA__) || defined(__AVX2__)
		const __m128 half = _mm_set1_ps(0.5f);
		const __m128 thrh = _mm_set1_ps(1.5f);

		__m128 inv2 = _mm_mul_ps(inv, inv);
		__m128 term = _mm_fnmadd_ps(_mm_mul_ps(half, L2b), inv2, thrh); // 1.5 - 0.5*L2*inv^2
		inv = _mm_mul_ps(inv, term);

#else
		const __m128 half = _mm_set1_ps(0.5f);
		const __m128 thrh = _mm_set1_ps(1.5f);
		const __m128 inv2 = _mm_mul_ps(inv, inv);
		const __m128 term = _mm_sub_ps(thrh, _mm_mul_ps(half, _mm_mul_ps(L2b, inv2)));
		inv = _mm_mul_ps(inv, term);
#endif

		// n *= inv (only xyz)
		n = _mm_mul_ps(n, inv);

		alignas(16) float t[4];
		_mm_store_ps(t, n);
		return { t[0], t[1], t[2], 0.0f };
#endif
	}

	// Vec3 contructor definition 
	inline Vec3f::Vec3f(const Vec4f& vec4): x(vec4.x), y(vec4.y), z(vec4.z), _pad(vec4.w){}

	struct alignas(16) Mat3f
	{
		Vec3f row[3];

		Mat3f()                                       // identity
			: row{ 
				{1,0,0}, 
				{0,1,0}, 
				{0,0,1} 
			}{}

		Mat3f(float d)                                // diag(d)
			: row{ 
				{d,0,0}, 
				{0,d,0}, 
				{0,0,d} 
			} {}

		Mat3f(const Vec3f& r0, const Vec3f& r1, const Vec3f& r2)
			: row{ r0,r1,r2 } {
		}
		/* default copy / assignment fine */

		/* ---------- element access ------------------------------------------ */
		Vec3f& operator[](int i) { return row[i]; }
		const Vec3f& operator[](int i) const { return row[i]; }

		float* data() { return reinterpret_cast<float*>(row); }
		const float* data() const { return reinterpret_cast<const float*>(row); }

		Mat3f transpose() const
		{
#ifndef ENGINE_MATH_SIMD
			return Mat3f({ row[0].x,row[1].x,row[2].x },
				{ row[0].y,row[1].y,row[2].y },
				{ row[0].z,row[1].z,row[2].z });
#else
			__m128 a = _mm_setr_ps(row[0].x, row[0].y, row[0].z, 0.0f); // [x0 y0 z0 0]
			__m128 b = _mm_setr_ps(row[1].x, row[1].y, row[1].z, 0.0f); // [x1 y1 z1 0]
			__m128 c = _mm_setr_ps(row[2].x, row[2].y, row[2].z, 0.0f); // [x2 y2 z2 0]
			__m128 d = _mm_setzero_ps();                                 // [0  0  0  0]
			_MM_TRANSPOSE4_PS(a, b, c, d); // now: a=[x0 x1 x2 0], b=[y0 y1 y2 0], c=[z0 z1 z2 0]

			alignas(16) float ax[4], bx[4], cx[4];
			_mm_store_ps(ax, a);
			_mm_store_ps(bx, b);
			_mm_store_ps(cx, c);
			return Mat3f(Vec3f{ ax[0], ax[1], ax[2] },
				Vec3f{ bx[0], bx[1], bx[2] },
				Vec3f{ cx[0], cx[1], cx[2] });
#endif
		}
		/* ---------- matrix × matrix ---------------------------------------- */
		Mat3f operator*(const Mat3f& b) const
		{
			Mat3f out;
#ifndef ENGINE_MATH_SIMD
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j) {
					out[i][j] = 0.f;
					for (int k = 0; k < 3; ++k) out[i][j] += row[i][k] * b[k][j];
				}
#else
			for (int i = 0; i < 3; ++i) {
				__m128 acc = _mm_setzero_ps();
				for (int k = 0; k < 3; ++k) {
					__m128 a = _mm_set1_ps(row[i][k]);
					__m128 br = _mm_set_ps(0, b[k].z, b[k].y, b[k].x);
					acc = _mm_add_ps(acc, _mm_mul_ps(a, br));
				}
				out[i] = Vec3f{ acc.m128_f32[0],acc.m128_f32[1],acc.m128_f32[2] };
			}
#endif
			return out;
		}

		/* ---------- matrix × vec ------------------------------------------- */
		Vec3f operator*(const Vec3f& v) const
		{
#ifndef ENGINE_MATH_SIMD
			return {
				row[0].dot(v),
				row[1].dot(v),
				row[2].dot(v) };
#else
			__m128 vx = _mm_set1_ps(v.x),
				vy = _mm_set1_ps(v.y),
				vz = _mm_set1_ps(v.z);
			__m128 res =
				_mm_add_ps(_mm_add_ps(_mm_mul_ps(vx, row[0].simd),
					_mm_mul_ps(vy, row[1].simd)),
					_mm_mul_ps(vz, row[2].simd));
			return Vec3f(res);
#endif
		}
	};

	/* ---------- free helpers (SIMD + scalar) ------------------------------ */
	inline Mat3f operator+(const Mat3f& a, const Mat3f& b) {
		Mat3f o; for (int i = 0; i < 3; ++i) o[i] = a[i] + b[i]; return o;
	}
	inline Mat3f operator-(const Mat3f& a, const Mat3f& b) {
		Mat3f o; for (int i = 0; i < 3; ++i) o[i] = a[i] - b[i]; return o;
	}
	inline Mat3f operator*(const Mat3f& m, float s) {
#ifdef ENGINE_MATH_SIMD
		__m128 S = _mm_set1_ps(s); Mat3f o;
		for (int i = 0; i < 3; ++i) o[i] = Vec3f(_mm_mul_ps(m[i].simd, S));
		return o;
#else
		Mat3f o; for (int i = 0; i < 3; ++i) o[i] = m[i] * s; return o;
#endif
	}


	inline float det(const Mat3f& mat)
	{
#ifndef ENGINE_MATH_SIMD
		// Scalar 3x3 determinant via triple product: det = a · (b × c)
		const float ax = mat[0][0], ay = mat[0][1], az = mat[0][2];
		const float bx = mat[1][0], by = mat[1][1], bz = mat[1][2];
		const float cx = mat[2][0], cy = mat[2][1], cz = mat[2][2];

		const float cx_x = by * cz - bz * cy;
		const float cx_y = bz * cx - bx * cz;
		const float cx_z = bx * cy - by * cx;

		return ax * cx_x + ay * cx_y + az * cx_z;

#else
		// SIMD (SSE2+, optional FMA). det = a · (b × c)
		const __m128 a = _mm_setr_ps(mat[0][0], mat[0][1], mat[0][2], 0.0f); // [ax ay az 0]
		const __m128 b = _mm_setr_ps(mat[1][0], mat[1][1], mat[1][2], 0.0f); // [bx by bz 0]
		const __m128 c = _mm_setr_ps(mat[2][0], mat[2][1], mat[2][2], 0.0f); // [cx cy cz 0]

		// b × c
		const __m128 b_yzx = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1)); // [by bz bx 0]
		const __m128 c_zxy = _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 1, 0, 2)); // [cz cx cy 0]
		const __m128 b_zxy = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2)); // [bz bx by 0]
		const __m128 c_yzx = _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 0, 2, 1)); // [cy cz cx 0]

		__m128 cross = _mm_sub_ps(_mm_mul_ps(b_yzx, c_zxy), _mm_mul_ps(b_zxy, c_yzx)); // [cx_x cx_y cx_z 0]

		// det = dot3(a, cross)
		__m128 prod = _mm_mul_ps(a, cross);
		// horizontal sum of x+y+z (w is 0)
		__m128 shuf = _mm_shuffle_ps(prod, prod, _MM_SHUFFLE(3, 0, 2, 1));
		__m128 sum = _mm_add_ps(prod, shuf);
		shuf = _mm_shuffle_ps(sum, sum, _MM_SHUFFLE(3, 0, 2, 1));
		sum = _mm_add_ps(sum, shuf);

		return _mm_cvtss_f32(sum);
#endif
	}

	// 3x3 Matrix of Minors: Min[r][c] = det( A without row r and column c )
// Implemented via cross products: for 3x3, cofactor rows are
//   Cof0 = cross(r1, r2), Cof1 = cross(r2, r0), Cof2 = cross(r0, r1)
// And Min = sign * Cof with checkerboard signs removed/applied back.
	inline Mat3f matrixOfMinors(const Mat3f& A)
	{
#ifndef ENGINE_MATH_SIMD
		const float a = A.row[0].x, b = A.row[0].y, c = A.row[0].z;
		const float d = A.row[1].x, e = A.row[1].y, f = A.row[1].z;
		const float g = A.row[2].x, h = A.row[2].y, i = A.row[2].z;

		// Cofactor rows via cross products
		// Cof0 = cross(r1, r2)
		const float Cof00 = e * i - f * h;
		const float Cof01 = f * g - d * i;
		const float Cof02 = d * h - e * g;

		// Cof1 = cross(r2, r0)
		const float Cof10 = h * c - i * b;
		const float Cof11 = i * a - g * c;
		const float Cof12 = g * b - h * a;

		// Cof2 = cross(r0, r1)
		const float Cof20 = b * f - c * e;
		const float Cof21 = c * d - a * f;
		const float Cof22 = a * e - b * d;

		// Minors = checkerboard * Cofactors  (since Cof = sign * Min  => Min = sign * Cof)
		Mat3f Min;
		Min.row[0] = { +Cof00, -Cof01, +Cof02 }; // (+ - +)
		Min.row[1] = { -Cof10, +Cof11, -Cof12 }; // (- + -)
		Min.row[2] = { +Cof20, -Cof21, +Cof22 }; // (+ - +)
		return Min;

#else
		// Load rows as [x y z 0]
		const __m128 r0 = _mm_setr_ps(A.row[0].x, A.row[0].y, A.row[0].z, 0.0f);
		const __m128 r1 = _mm_setr_ps(A.row[1].x, A.row[1].y, A.row[1].z, 0.0f);
		const __m128 r2 = _mm_setr_ps(A.row[2].x, A.row[2].y, A.row[2].z, 0.0f);

		auto cross3 = [](__m128 a, __m128 b) -> __m128 {
			const __m128 a_yzx = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
			const __m128 b_zxy = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2));
			const __m128 a_zxy = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2));
			const __m128 b_yzx = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1));
#if defined(__FMA__) || defined(__AVX2__)
			return _mm_fmsub_ps(a_yzx, b_zxy, _mm_mul_ps(a_zxy, b_yzx));
#else
			return _mm_sub_ps(_mm_mul_ps(a_yzx, b_zxy), _mm_mul_ps(a_zxy, b_yzx));
#endif
		};

		// Cofactor rows (not transposed)
		const __m128 Cof0 = cross3(r1, r2);
		const __m128 Cof1 = cross3(r2, r0);
		const __m128 Cof2 = cross3(r0, r1);

		// Minors = checkerboard * Cofactors
		const __m128 s0 = _mm_setr_ps(1.0f, -1.0f, 1.0f, 0.0f); // row 0 signs
		const __m128 s1 = _mm_setr_ps(-1.0f, 1.0f, -1.0f, 0.0f); // row 1 signs
		const __m128 s2 = _mm_setr_ps(1.0f, -1.0f, 1.0f, 0.0f); // row 2 signs

		__m128 m0 = _mm_mul_ps(Cof0, s0);
		__m128 m1 = _mm_mul_ps(Cof1, s1);
		__m128 m2 = _mm_mul_ps(Cof2, s2);

		alignas(16) float rM0[4], rM1[4], rM2[4];
		_mm_store_ps(rM0, m0);
		_mm_store_ps(rM1, m1);
		_mm_store_ps(rM2, m2);

		Mat3f Min;
		Min.row[0] = { rM0[0], rM0[1], rM0[2] };
		Min.row[1] = { rM1[0], rM1[1], rM1[2] };
		Min.row[2] = { rM2[0], rM2[1], rM2[2] };
		return Min;
#endif
	}

	// 3x3 Cofactor from Minors: Cof[r][c] = ((r+c)%2 ? -1 : +1) * Min[r][c]
	inline Mat3f cofactorFromMinors(const Mat3f& Min)
	{
#ifndef ENGINE_MATH_SIMD
		Mat3f Cof;
		Cof.row[0] = { +Min.row[0].x, -Min.row[0].y, +Min.row[0].z };
		Cof.row[1] = { -Min.row[1].x, +Min.row[1].y, -Min.row[1].z };
		Cof.row[2] = { +Min.row[2].x, -Min.row[2].y, +Min.row[2].z };
		return Cof;
#else
		const __m128 s0 = _mm_setr_ps(1.0f, -1.0f, 1.0f, 0.0f);
		const __m128 s1 = _mm_setr_ps(-1.0f, 1.0f, -1.0f, 0.0f);
		const __m128 s2 = _mm_setr_ps(1.0f, -1.0f, 1.0f, 0.0f);

		const __m128 r0 = _mm_setr_ps(Min.row[0].x, Min.row[0].y, Min.row[0].z, 0.0f);
		const __m128 r1 = _mm_setr_ps(Min.row[1].x, Min.row[1].y, Min.row[1].z, 0.0f);
		const __m128 r2 = _mm_setr_ps(Min.row[2].x, Min.row[2].y, Min.row[2].z, 0.0f);

		__m128 c0 = _mm_mul_ps(r0, s0);
		__m128 c1 = _mm_mul_ps(r1, s1);
		__m128 c2 = _mm_mul_ps(r2, s2);

		alignas(16) float rc0[4], rc1[4], rc2[4];
		_mm_store_ps(rc0, c0);
		_mm_store_ps(rc1, c1);
		_mm_store_ps(rc2, c2);

		Mat3f Cof;
		Cof.row[0] = { rc0[0], rc0[1], rc0[2] };
		Cof.row[1] = { rc1[0], rc1[1], rc1[2] };
		Cof.row[2] = { rc2[0], rc2[1], rc2[2] };
		return Cof;
#endif
	}



	struct alignas(16) Mat4f
	{
		Vec4f row[4];

		Mat4f()
		{
			row[0] = Vec4f(1.0f, 0, 0, 0);
			row[1] = Vec4f(0, 1.0f, 0, 0);
			row[2] = Vec4f(0, 0, 1.0f, 0);
			row[3] = Vec4f(0, 0, 0, 1.0f);
		}

		// Constructor for custom matrix
		Mat4f(const Vec4f& row0, const Vec4f& row1, const Vec4f& row2, const Vec4f& row3)
		{
			row[0] = row0;
			row[1] = row1;
			row[2] = row2;
			row[3] = row3;
		}

		Mat4f(float Q)
		{
			row[0] = Vec4f(Q, 0, 0, 0);
			row[1] = Vec4f(0, Q, 0, 0);
			row[2] = Vec4f(0, 0, Q, 0);
			row[3] = Vec4f(0, 0, 0, Q);
		}


		// Copy constructor
		Mat4f(const Mat4f& other)
		{
			// Perform a deep copy of the SIMD registers
			row[0] = other.row[0];
			row[1] = other.row[1];
			row[2] = other.row[2];
			row[3] = other.row[3];
		}

		// Copy assignment operator
		Mat4f& operator=(const Mat4f& other)
		{
			if (this != &other) // Protect against self-assignment
			{
				row[0] = other.row[0];
				row[1] = other.row[1];
				row[2] = other.row[2];
				row[3] = other.row[3];
			}
			return *this;
		}

		inline const Vec4f& operator[](int idxrow) const
		{
			switch (idxrow)
			{
			case 0: return row[0];
			case 1: return row[1];
			case 2: return row[2];
			case 3: return row[3];
			default: throw std::out_of_range("ERROR::ENGINE::MATH::MAT4F::INDEX_OUT_OF_RANGE");
			}
		}

		// Mutable access
		inline Vec4f& operator[](int idxrow)
		{
			switch (idxrow)
			{
			case 0: return row[0];
			case 1: return row[1];
			case 2: return row[2];
			case 3: return row[3];
			default: throw std::out_of_range("ERROR::ENGINE::MATH::MAT4F::INDEX_OUT_OF_RANGE");
			}
		}

		// Non-const version (for modification)
		inline float* data() {
			return reinterpret_cast<float*>(&row[0]);
		}

		// Const version (for read-only access)
		inline const float* data() const {
			return reinterpret_cast<const float*>(&row[0]);
		}



		Mat4f transpose() const {
			Mat4f result;
			for (int i = 0; i < 4; ++i)
				for (int j = 0; j < 4; ++j)
					result[i][j] = (*this)[j][i];  // Transpose rows and columns
			return result;
		}

		Mat4f operator*(const Mat4f& other) const
		{
			Mat4f result;
#ifndef ENGINE_MATH_SIMD
			// Standard matrix multiplication: result[i][j] = sum(this[i][k] * other[k][j])
			for (int i = 0; i < 4; ++i) {
				for (int j = 0; j < 4; ++j) {
					result[i][j] = 0.0f;
					for (int k = 0; k < 4; ++k) {
						result[i][j] += mat4f[i][k] * other[k][j];
					}
				}
			}
#else
			for (int i = 0; i < 4; ++i) {
				// For each row i of the result matrix
				__m128 result_row = _mm_setzero_ps();

				for (int k = 0; k < 4; ++k) {
					// Broadcast mat4f[i][k] to all elements of a __m128
					__m128 a_ik = _mm_set1_ps(row[i][k]);
					// Multiply by row k of other matrix
					__m128 b_k = other[k].simdValue();
					// Add to result
					result_row = _mm_add_ps(result_row, _mm_mul_ps(a_ik, b_k));
				}

				result[i] = Vec4f(result_row);
			}
#endif

			return result;
		}

		Vec4f operator*(const Vec4f& vec) const
		{
#ifndef ENGINE_MATH_SIMD
			return Vec4f(
				vec.x * mat4f[0].x + vec.y * mat4f[1].x + vec.z * mat4f[2].x + vec.w * mat4f[3].x,
				vec.x * mat4f[0].y + vec.y * mat4f[1].y + vec.z * mat4f[2].y + vec.w * mat4f[3].y,
				vec.x * mat4f[0].z + vec.y * mat4f[1].z + vec.z * mat4f[2].z + vec.w * mat4f[3].z,
				vec.x * mat4f[0].w + vec.y * mat4f[1].w + vec.z * mat4f[2].w + vec.w * mat4f[3].w
			);
#else
			// Load vector to SIMD
			__m128 v = vec.simdValue();

			__m128 col0 = _mm_setr_ps(row[0].x, row[1].x, row[2].x, row[3].x);
			__m128 col1 = _mm_setr_ps(row[0].y, row[1].y, row[2].y, row[3].y);
			__m128 col2 = _mm_setr_ps(row[0].z, row[1].z, row[2].z, row[3].z);
			__m128 col3 = _mm_setr_ps(row[0].w, row[1].w, row[2].w, row[3].w);

			__m128 res;
			res = _mm_mul_ps(v, col0);
			float x = res.m128_f32[0] + res.m128_f32[1] + res.m128_f32[2] + res.m128_f32[3];

			res = _mm_mul_ps(v, col1);
			float y = res.m128_f32[0] + res.m128_f32[1] + res.m128_f32[2] + res.m128_f32[3];

			res = _mm_mul_ps(v, col2);
			float z = res.m128_f32[0] + res.m128_f32[1] + res.m128_f32[2] + res.m128_f32[3];

			res = _mm_mul_ps(v, col3);
			float w = res.m128_f32[0] + res.m128_f32[1] + res.m128_f32[2] + res.m128_f32[3];

			return Vec4f(x, y, z, w);
#endif
		}
	};

#ifndef ENGINE_MATH_SIMD
	inline Mat4f operator+(const Mat4f& a, const Mat4f& b)
	{
		Mat4f result;
		for (int i = 0; i < 4; ++i)
			result[i] = a[i] + b[i];
		return result;
	}

	inline Mat4f operator-(const Mat4f& a, const Mat4f& b)
	{
		Mat4f result; 
		for (int i = 0; i < 4; ++i)
			result[i] = a[i] - b[i];
		return result;
	}

	inline Mat4f operator*(const Mat4f& m, float scalar)
	{
		Mat4f result;
		// Standard matrix multiplication: result[i][j] = sum(this[i][k] * other[k][j])
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				result[i][j] = 0.0f;
				for (int k = 0; k < 4; ++k) {
					result[i][j] += mat4f[i][k] * other[k][j];
				}
			}
		}

	}

	inline bool operator==(const Mat4f& a, const Mat4f& b)
	{
		for (int i = 0; i < 4; ++i)
			if (!(a[i] == b[i])) return false;
		return true;
	}
#else
	inline Mat4f operator+(const Mat4f& a, const Mat4f& b)
	{
		Mat4f result;
		for (int i = 0; i < 4; ++i)
			result[i] = Vec4f(_mm_add_ps(a[i].simdValue(), b[i].simdValue()));
		return result;
	}

	inline Mat4f operator-(const Mat4f& a, const Mat4f& b)
	{
		Mat4f result;
		for (int i = 0; i < 4; ++i)
			result[i] = Vec4f(_mm_sub_ps(a[i].simdValue(), b[i].simdValue()));
		return result;
	}

	inline Mat4f operator*(const Mat4f& m, float scalar)
	{
		__m128 s = _mm_set1_ps(scalar);
		Mat4f result;
		for (int i = 0; i < 4; ++i)
			result[i] = Vec4f(_mm_mul_ps(m[i].simdValue(), s));
		return result;
	}

	inline bool operator==(const Mat4f& a, const Mat4f& b)
	{
		const float epsilon = 1e-6f;
		for (int i = 0; i < 4; ++i)
		{
			__m128 diff = _mm_sub_ps(a[i].simdValue(), b[i].simdValue());
			__m128 abs_diff = _mm_andnot_ps(_mm_set1_ps(-0.0f), diff);
			__m128 eps = _mm_set1_ps(epsilon);
			__m128 cmp = _mm_cmple_ps(abs_diff, eps);
			if (_mm_movemask_ps(cmp) != 0xF)
				return false;
		}
		return true;
	}


	inline Vec4f operator*(const Vec4f& v, const Mat4f& M)
	{
		// r = v.x * row0 + v.y * row1 + v.z * row2 + v.w * row3
		__m128 acc = _mm_mul_ps(_mm_set1_ps(v.x), M[0].simd);
		acc = _mm_add_ps(acc, _mm_mul_ps(_mm_set1_ps(v.y), M[1].simd));
		acc = _mm_add_ps(acc, _mm_mul_ps(_mm_set1_ps(v.z), M[2].simd));
		acc = _mm_add_ps(acc, _mm_mul_ps(_mm_set1_ps(v.w), M[3].simd));

		Vec4f out;
		out.simd = acc;
		return out;
	}


	inline float det(const Mat4f& mat)
	{
#ifndef ENGINE_MATH_SIMD
		// Scalar fast path via 2x2 minors (no temporaries, branchless)
		const float a00 = mat[0][0], a01 = mat[0][1], a02 = mat[0][2], a03 = mat[0][3];
		const float a10 = mat[1][0], a11 = mat[1][1], a12 = mat[1][2], a13 = mat[1][3];
		const float a20 = mat[2][0], a21 = mat[2][1], a22 = mat[2][2], a23 = mat[2][3];
		const float a30 = mat[3][0], a31 = mat[3][1], a32 = mat[3][2], a33 = mat[3][3];

		// 2x2 minors made from rows 2&3 (columns as indicated)
		const float M23_01 = a20 * a31 - a21 * a30;
		const float M23_02 = a20 * a32 - a22 * a30;
		const float M23_03 = a20 * a33 - a23 * a30;
		const float M23_12 = a21 * a32 - a22 * a31;
		const float M23_13 = a21 * a33 - a23 * a31;
		const float M23_23 = a22 * a33 - a23 * a32;

		// Cofactors for first row
		const float C00 = a11 * M23_23 - a12 * M23_13 + a13 * M23_12;
		const float C01 = a10 * M23_23 - a12 * M23_03 + a13 * M23_02;
		const float C02 = a10 * M23_13 - a11 * M23_03 + a13 * M23_01;
		const float C03 = a10 * M23_12 - a11 * M23_02 + a12 * M23_01;

		// Determinant (expansion along first row, with signs + - + -)
		return  a00 * C00 - a01 * C01 + a02 * C02 - a03 * C03;

#else
		// SIMD (SSE2+, optional FMA) via 2x2 minors

		const float a00 = mat[0][0], a01 = mat[0][1], a02 = mat[0][2], a03 = mat[0][3];
		const float a10 = mat[1][0], a11 = mat[1][1], a12 = mat[1][2], a13 = mat[1][3];
		const float a20 = mat[2][0], a21 = mat[2][1], a22 = mat[2][2], a23 = mat[2][3];
		const float a30 = mat[3][0], a31 = mat[3][1], a32 = mat[3][2], a33 = mat[3][3];

		// Pack rows 2 and 3
		const __m128 r2 = _mm_set_ps(a23, a22, a21, a20); // [a23 a22 a21 a20]
		const __m128 r3 = _mm_set_ps(a33, a32, a31, a30); // [a33 a32 a31 a30]

		// Helper extracts (x=lane0, y=lane1, z=lane2 of a [w z y x] set_ps layout)
		auto getx = [](__m128 v) -> float { return _mm_cvtss_f32(v); };
		auto gety = [](__m128 v) -> float {
			return _mm_cvtss_f32(_mm_shuffle_ps(v, v, _MM_SHUFFLE(3, 2, 1, 1)));
			};
		auto getz = [](__m128 v) -> float {
			return _mm_cvtss_f32(_mm_shuffle_ps(v, v, _MM_SHUFFLE(3, 2, 1, 2)));
			};

		// V1 = [ M23_01, M23_02, M23_03, 0 ] =
		//       a20 * [a31 a32 a33 0] - a30 * [a21 a22 a23 0]
		const __m128 v31_33 = _mm_set_ps(0.0f, a33, a32, a31);
		const __m128 v21_23 = _mm_set_ps(0.0f, a23, a22, a21);
		const __m128 a20v = _mm_set1_ps(a20);
		const __m128 a30v = _mm_set1_ps(a30);
#if defined(__FMA__) ||  defined(__AVX2__)
		const __m128 V1 = _mm_fmsub_ps(a20v, v31_33, _mm_mul_ps(a30v, v21_23));
#else
		const __m128 V1 = _mm_sub_ps(_mm_mul_ps(a20v, v31_33), _mm_mul_ps(a30v, v21_23));
#endif

		// V2 = [ M23_12, M23_13, 0, 0 ] =
		//       a21 * [a32 a33 0 0] - a31 * [a22 a23 0 0]
		const __m128 v32_33 = _mm_set_ps(0.0f, 0.0f, a33, a32);
		const __m128 v22_23 = _mm_set_ps(0.0f, 0.0f, a23, a22);
		const __m128 a21v = _mm_set1_ps(a21);
		const __m128 a31v = _mm_set1_ps(a31);
#if defined(__FMA__) ||  defined(__AVX2__)
		const __m128 V2 = _mm_fmsub_ps(a21v, v32_33, _mm_mul_ps(a31v, v22_23));
#else
		const __m128 V2 = _mm_sub_ps(_mm_mul_ps(a21v, v32_33), _mm_mul_ps(a31v, v22_23));
#endif

		// M23_23 = a22*a33 - a23*a32 (scalar)
		const float M23_23 = a22 * a33 - a23 * a32;

		// Extract needed minors
		const float M23_01 = getx(V1);
		const float M23_02 = gety(V1);
		const float M23_03 = getz(V1);
		const float M23_12 = getx(V2);
		const float M23_13 = gety(V2);

		// Cofactors (scalars; small dot products)
		const float C00 = a11 * M23_23 - a12 * M23_13 + a13 * M23_12;
		const float C01 = a10 * M23_23 - a12 * M23_03 + a13 * M23_02;
		const float C02 = a10 * M23_13 - a11 * M23_03 + a13 * M23_01;
		const float C03 = a10 * M23_12 - a11 * M23_02 + a12 * M23_01;

		// Determinant
		return  a00 * C00 - a01 * C01 + a02 * C02 - a03 * C03;
#endif
	}

	// Matrix of minors for a 4x4 matrix.
	// Returns Min where Min[r][c] = det( A without row r and column c ).
	// NOTE: This is the unsigned "matrix of minors" (not cofactors). Apply checkerboard signs separately.

	inline Mat4f matrixOfMinors(const Mat4f& A)
	{
#ifndef ENGINE_MATH_SIMD
		const float a00 = A[0][0], a01 = A[0][1], a02 = A[0][2], a03 = A[0][3];
		const float a10 = A[1][0], a11 = A[1][1], a12 = A[1][2], a13 = A[1][3];
		const float a20 = A[2][0], a21 = A[2][1], a22 = A[2][2], a23 = A[2][3];
		const float a30 = A[3][0], a31 = A[3][1], a32 = A[3][2], a33 = A[3][3];

		// Precompute all 2x2 minors for row-pairs we’ll use:
		// Rows (2,3):
		const float M23_01 = a20 * a31 - a21 * a30;
		const float M23_02 = a20 * a32 - a22 * a30;
		const float M23_03 = a20 * a33 - a23 * a30;
		const float M23_12 = a21 * a32 - a22 * a31;
		const float M23_13 = a21 * a33 - a23 * a31;
		const float M23_23 = a22 * a33 - a23 * a32;
		// Rows (1,3):
		const float M13_01 = a10 * a31 - a11 * a30;
		const float M13_02 = a10 * a32 - a12 * a30;
		const float M13_03 = a10 * a33 - a13 * a30;
		const float M13_12 = a11 * a32 - a12 * a31;
		const float M13_13 = a11 * a33 - a13 * a31;
		const float M13_23 = a12 * a33 - a13 * a32;
		// Rows (1,2):
		const float M12_01 = a10 * a21 - a11 * a20;
		const float M12_02 = a10 * a22 - a12 * a20;
		const float M12_03 = a10 * a23 - a13 * a20;
		const float M12_12 = a11 * a22 - a12 * a21;
		const float M12_13 = a11 * a23 - a13 * a21;
		const float M12_23 = a12 * a23 - a13 * a22;

		Mat4f Min;

		// Exclude row 0: use rows (1,2,3), expand along row 1 with pair (2,3)
		Min[0][0] = a11 * M23_23 - a12 * M23_13 + a13 * M23_12; // cols 1,2,3
		Min[0][1] = a10 * M23_23 - a12 * M23_03 + a13 * M23_02; // cols 0,2,3
		Min[0][2] = a10 * M23_13 - a11 * M23_03 + a13 * M23_01; // cols 0,1,3
		Min[0][3] = a10 * M23_12 - a11 * M23_02 + a12 * M23_01; // cols 0,1,2

		// Exclude row 1: use rows (0,2,3), expand along row 0 with pair (2,3)
		Min[1][0] = a01 * M23_23 - a02 * M23_13 + a03 * M23_12; // cols 1,2,3
		Min[1][1] = a00 * M23_23 - a02 * M23_03 + a03 * M23_02; // cols 0,2,3
		Min[1][2] = a00 * M23_13 - a01 * M23_03 + a03 * M23_01; // cols 0,1,3
		Min[1][3] = a00 * M23_12 - a01 * M23_02 + a02 * M23_01; // cols 0,1,2

		// Exclude row 2: use rows (0,1,3), expand along row 0 with pair (1,3)
		Min[2][0] = a01 * M13_23 - a02 * M13_13 + a03 * M13_12; // cols 1,2,3
		Min[2][1] = a00 * M13_23 - a02 * M13_03 + a03 * M13_02; // cols 0,2,3
		Min[2][2] = a00 * M13_13 - a01 * M13_03 + a03 * M13_01; // cols 0,1,3
		Min[2][3] = a00 * M13_12 - a01 * M13_02 + a02 * M13_01; // cols 0,1,2

		// Exclude row 3: use rows (0,1,2), expand along row 0 with pair (1,2)
		Min[3][0] = a01 * M12_23 - a02 * M12_13 + a03 * M12_12; // cols 1,2,3
		Min[3][1] = a00 * M12_23 - a02 * M12_03 + a03 * M12_02; // cols 0,2,3
		Min[3][2] = a00 * M12_13 - a01 * M12_03 + a03 * M12_01; // cols 0,1,3
		Min[3][3] = a00 * M12_12 - a01 * M12_02 + a02 * M12_01; // cols 0,1,2

		return Min;

#else
		const float a00 = A[0][0], a01 = A[0][1], a02 = A[0][2], a03 = A[0][3];
		const float a10 = A[1][0], a11 = A[1][1], a12 = A[1][2], a13 = A[1][3];
		const float a20 = A[2][0], a21 = A[2][1], a22 = A[2][2], a23 = A[2][3];
		const float a30 = A[3][0], a31 = A[3][1], a32 = A[3][2], a33 = A[3][3];

		auto getx = [](__m128 v) -> float { return _mm_cvtss_f32(v); };
		auto gety = [](__m128 v) -> float { return _mm_cvtss_f32(_mm_shuffle_ps(v, v, _MM_SHUFFLE(3, 2, 1, 1))); };
		auto getz = [](__m128 v) -> float { return _mm_cvtss_f32(_mm_shuffle_ps(v, v, _MM_SHUFFLE(3, 2, 1, 2))); };

		// ----- Rows (2,3): build M23_01,02,03 | M23_12,13 | M23_23 -----
		const __m128 v31_33 = _mm_set_ps(0.0f, a33, a32, a31);
		const __m128 v21_23 = _mm_set_ps(0.0f, a23, a22, a21);
		const __m128 a20v = _mm_set1_ps(a20);
		const __m128 a30v = _mm_set1_ps(a30);
#if defined(__FMA__) || defined(__AVX2__)
		const __m128 V23a = _mm_fmsub_ps(a20v, v31_33, _mm_mul_ps(a30v, v21_23)); // [M23_01, M23_02, M23_03, 0]
#else
		const __m128 V23a = _mm_sub_ps(_mm_mul_ps(a20v, v31_33), _mm_mul_ps(a30v, v21_23));
#endif
		const float M23_01 = getx(V23a), M23_02 = gety(V23a), M23_03 = getz(V23a);

		const __m128 v32_33 = _mm_set_ps(0.0f, 0.0f, a33, a32);
		const __m128 v22_23 = _mm_set_ps(0.0f, 0.0f, a23, a22);
		const __m128 a21v = _mm_set1_ps(a21);
		const __m128 a31v = _mm_set1_ps(a31);
#if defined(__FMA__) || defined(__AVX2__)
		const __m128 V23b = _mm_fmsub_ps(a21v, v32_33, _mm_mul_ps(a31v, v22_23)); // [M23_12, M23_13, 0, 0]
#else
		const __m128 V23b = _mm_sub_ps(_mm_mul_ps(a21v, v32_33), _mm_mul_ps(a31v, v22_23));
#endif
		const float M23_12 = getx(V23b), M23_13 = gety(V23b);
		const float M23_23 = a22 * a33 - a23 * a32;

		// ----- Rows (1,3): build M13_01,02,03 | M13_12,13 | M13_23 -----
		const __m128 v31_33b = _mm_set_ps(0.0f, a33, a32, a31);
		const __m128 v11_13 = _mm_set_ps(0.0f, a13, a12, a11);
		const __m128 a10v = _mm_set1_ps(a10);
		const __m128 a30v2 = _mm_set1_ps(a30);
#if defined(__FMA__) || defined(__AVX2__)
		const __m128 V13a = _mm_fmsub_ps(a10v, v31_33b, _mm_mul_ps(a30v2, v11_13)); // [M13_01, M13_02, M13_03, 0]
#else
		const __m128 V13a = _mm_sub_ps(_mm_mul_ps(a10v, v31_33b), _mm_mul_ps(a30v2, v11_13));
#endif
		const float M13_01 = getx(V13a), M13_02 = gety(V13a), M13_03 = getz(V13a);

		const __m128 v32_33b = _mm_set_ps(0.0f, 0.0f, a33, a32);
		const __m128 v12_13 = _mm_set_ps(0.0f, 0.0f, a13, a12);
		const __m128 a11v = _mm_set1_ps(a11);
		const __m128 a31v2 = _mm_set1_ps(a31);
#if defined(__FMA__) || defined(__AVX2__)
		const __m128 V13b = _mm_fmsub_ps(a11v, v32_33b, _mm_mul_ps(a31v2, v12_13)); // [M13_12, M13_13, 0, 0]
#else
		const __m128 V13b = _mm_sub_ps(_mm_mul_ps(a11v, v32_33b), _mm_mul_ps(a31v2, v12_13));
#endif
		const float M13_12 = getx(V13b), M13_13 = gety(V13b);
		const float M13_23 = a12 * a33 - a13 * a32;

		// ----- Rows (1,2): build M12_01,02,03 | M12_12,13 | M12_23 -----
		const __m128 v21_23b = _mm_set_ps(0.0f, a23, a22, a21);
		const __m128 v11_13b = _mm_set_ps(0.0f, a13, a12, a11);
		const __m128 a10v2 = _mm_set1_ps(a10);
		const __m128 a20v2 = _mm_set1_ps(a20);
#if defined(__FMA__) || defined(__AVX2__)
		const __m128 V12a = _mm_fmsub_ps(a10v2, v21_23b, _mm_mul_ps(a20v2, v11_13b)); // [M12_01, M12_02, M12_03, 0]
#else
		const __m128 V12a = _mm_sub_ps(_mm_mul_ps(a10v2, v21_23b), _mm_mul_ps(a20v2, v11_13b));
#endif
		const float M12_01 = getx(V12a), M12_02 = gety(V12a), M12_03 = getz(V12a);

		const __m128 v22_23b = _mm_set_ps(0.0f, 0.0f, a23, a22);
		const __m128 v12_13b = _mm_set_ps(0.0f, 0.0f, a13, a12);
		const __m128 a11v2 = _mm_set1_ps(a11);
		const __m128 a21v2 = _mm_set1_ps(a21);
#if defined(__FMA__) || defined(__AVX2__)
		const __m128 V12b = _mm_fmsub_ps(a11v2, v22_23b, _mm_mul_ps(a21v2, v12_13b)); // [M12_12, M12_13, 0, 0]
#else
		const __m128 V12b = _mm_sub_ps(_mm_mul_ps(a11v2, v22_23b), _mm_mul_ps(a21v2, v12_13b));
#endif
		const float M12_12 = getx(V12b), M12_13 = gety(V12b);
		const float M12_23 = a12 * a23 - a13 * a22;

		Mat4f Min;

		// Row 0 minors (exclude row 0) using pair (2,3) and row 1
		Min[0][0] = a11 * M23_23 - a12 * M23_13 + a13 * M23_12;
		Min[0][1] = a10 * M23_23 - a12 * M23_03 + a13 * M23_02;
		Min[0][2] = a10 * M23_13 - a11 * M23_03 + a13 * M23_01;
		Min[0][3] = a10 * M23_12 - a11 * M23_02 + a12 * M23_01;

		// Row 1 minors (exclude row 1) using pair (2,3) and row 0
		Min[1][0] = a01 * M23_23 - a02 * M23_13 + a03 * M23_12;
		Min[1][1] = a00 * M23_23 - a02 * M23_03 + a03 * M23_02;
		Min[1][2] = a00 * M23_13 - a01 * M23_03 + a03 * M23_01;
		Min[1][3] = a00 * M23_12 - a01 * M23_02 + a02 * M23_01;

		// Row 2 minors (exclude row 2) using pair (1,3) and row 0
		Min[2][0] = a01 * M13_23 - a02 * M13_13 + a03 * M13_12;
		Min[2][1] = a00 * M13_23 - a02 * M13_03 + a03 * M13_02;
		Min[2][2] = a00 * M13_13 - a01 * M13_03 + a03 * M13_01;
		Min[2][3] = a00 * M13_12 - a01 * M13_02 + a02 * M13_01;

		// Row 3 minors (exclude row 3) using pair (1,2) and row 0
		Min[3][0] = a01 * M12_23 - a02 * M12_13 + a03 * M12_12;
		Min[3][1] = a00 * M12_23 - a02 * M12_03 + a03 * M12_02;
		Min[3][2] = a00 * M12_13 - a01 * M12_03 + a03 * M12_01;
		Min[3][3] = a00 * M12_12 - a01 * M12_02 + a02 * M12_01;

		return Min;
#endif
	}

#endif

	// Apply checkerboard signs to a 4x4 matrix of minors to produce the cofactor matrix.
	// Cof[r][c] = ((r+c)%2 ? -1 : +1) * Min[r][c]
	inline Mat4f cofactorFromMinors(const Mat4f& Min)
	{
#ifndef ENGINE_MATH_SIMD
		Mat4f Cof;
		Cof[0][0] = Min[0][0];  Cof[0][1] = -Min[0][1]; Cof[0][2] = Min[0][2];  Cof[0][3] = -Min[0][3];
		Cof[1][0] = -Min[1][0];  Cof[1][1] = Min[1][1]; Cof[1][2] = -Min[1][2];  Cof[1][3] = Min[1][3];
		Cof[2][0] = Min[2][0];  Cof[2][1] = -Min[2][1]; Cof[2][2] = Min[2][2];  Cof[2][3] = -Min[2][3];
		Cof[3][0] = -Min[3][0];  Cof[3][1] = Min[3][1]; Cof[3][2] = -Min[3][2];  Cof[3][3] = Min[3][3];
		return Cof;
#else
		Mat4f Cof;

		const __m128 s0 = _mm_setr_ps(1.0f, -1.0f, 1.0f, -1.0f); // row 0 & 2
		const __m128 s1 = _mm_setr_ps(-1.0f, 1.0f, -1.0f, 1.0f); // row 1 & 3

		// row 0
		{
			const __m128 r = _mm_setr_ps(Min[0][0], Min[0][1], Min[0][2], Min[0][3]);
			alignas(16) float tmp[4];
			_mm_store_ps(tmp, _mm_mul_ps(r, s0));
			Cof[0][0] = tmp[0]; Cof[0][1] = tmp[1]; Cof[0][2] = tmp[2]; Cof[0][3] = tmp[3];
		}
		// row 1
		{
			const __m128 r = _mm_setr_ps(Min[1][0], Min[1][1], Min[1][2], Min[1][3]);
			alignas(16) float tmp[4];
			_mm_store_ps(tmp, _mm_mul_ps(r, s1));
			Cof[1][0] = tmp[0]; Cof[1][1] = tmp[1]; Cof[1][2] = tmp[2]; Cof[1][3] = tmp[3];
		}
		// row 2
		{
			const __m128 r = _mm_setr_ps(Min[2][0], Min[2][1], Min[2][2], Min[2][3]);
			alignas(16) float tmp[4];
			_mm_store_ps(tmp, _mm_mul_ps(r, s0));
			Cof[2][0] = tmp[0]; Cof[2][1] = tmp[1]; Cof[2][2] = tmp[2]; Cof[2][3] = tmp[3];
		}
		// row 3
		{
			const __m128 r = _mm_setr_ps(Min[3][0], Min[3][1], Min[3][2], Min[3][3]);
			alignas(16) float tmp[4];
			_mm_store_ps(tmp, _mm_mul_ps(r, s1));
			Cof[3][0] = tmp[0]; Cof[3][1] = tmp[1]; Cof[3][2] = tmp[2]; Cof[3][3] = tmp[3];
		}

		return Cof;
#endif
	}

	inline Mat4f inverse(const Mat4f& mat)
	{
		Mat4f res(0);

		float detA = det(mat);

		// matrix of minors
		auto m = matrixOfMinors(mat);

		// coffactor
		auto cof = cofactorFromMinors(m);
		
		auto adj = cof.transpose();

		for (int i = 0; i < 4; i++)
		{
			res[i] = adj[i] / detA;
		}
		return res;
	}

	inline Mat3f inverse(const Mat3f& A)
	{
#ifndef ENGINE_MATH_SIMD
		const engine::math::Vec3f r0 = A.row[0];
		const engine::math::Vec3f r1 = A.row[1];
		const engine::math::Vec3f r2 = A.row[2];

		const engine::math::Vec3f c0 = cross(r1, r2);
		const engine::math::Vec3f c1 = cross(r2, r0);
		const engine::math::Vec3f c2 = cross(r0, r1);

		const float det = r0.dot(c0);          // r0 · (r1×r2)
		const float invDet = 1.0f / det;

		// inv rows are the transposed cof-rows (i.e., columns = c0,c1,c2)
		return Mat3f(
			{ c0.x * invDet, c1.x * invDet, c2.x * invDet },
			{ c0.y * invDet, c1.y * invDet, c2.y * invDet },
			{ c0.z * invDet, c1.z * invDet, c2.z * invDet }
		);
#else
		// Pack rows as [x y z 0]
		const __m128 r0 = _mm_setr_ps(A.row[0].x, A.row[0].y, A.row[0].z, 0.0f);
		const __m128 r1 = _mm_setr_ps(A.row[1].x, A.row[1].y, A.row[1].z, 0.0f);
		const __m128 r2 = _mm_setr_ps(A.row[2].x, A.row[2].y, A.row[2].z, 0.0f);

		auto cross3 = [](__m128 a, __m128 b) {
			const __m128 a_yzx = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1));
			const __m128 b_zxy = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2));
			const __m128 a_zxy = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2));
			const __m128 b_yzx = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1));
#if defined(__FMA__) || defined(__AVX2__)
			return _mm_fmsub_ps(a_yzx, b_zxy, _mm_mul_ps(a_zxy, b_yzx));
#else
			return _mm_sub_ps(_mm_mul_ps(a_yzx, b_zxy), _mm_mul_ps(a_zxy, b_yzx));
#endif
			};
		auto dot3 = [](__m128 a, __m128 b) -> float {
#if defined(__SSE4_1__) || defined(__AVX__)
			return _mm_cvtss_f32(_mm_dp_ps(a, b, 0x71));   // sum xyz -> x
#else
			__m128 m = _mm_mul_ps(a, b);
			__m128 sh = _mm_shuffle_ps(m, m, _MM_SHUFFLE(3, 0, 2, 1));
			__m128 s = _mm_add_ps(m, sh);
			sh = _mm_shuffle_ps(s, s, _MM_SHUFFLE(3, 0, 2, 1));
			s = _mm_add_ps(s, sh);
			return _mm_cvtss_f32(s);
#endif
			};

		// Cofactor rows
		const __m128 c0 = cross3(r1, r2);
		const __m128 c1 = cross3(r2, r0);
		const __m128 c2 = cross3(r0, r1);

		const float det = dot3(r0, c0);                 // r0 · (r1×r2)
		const __m128 invDet = _mm_set1_ps(1.0f / det);  // broadcast

		// Scale cof-rows by 1/det
		const __m128 cc0 = _mm_mul_ps(c0, invDet);
		const __m128 cc1 = _mm_mul_ps(c1, invDet);
		const __m128 cc2 = _mm_mul_ps(c2, invDet);

		// inv rows are transpose of cc0,cc1,cc2 (we only need xyz)
		alignas(16) float a0[4], a1[4], a2[4];
		_mm_store_ps(a0, cc0);
		_mm_store_ps(a1, cc1);
		_mm_store_ps(a2, cc2);

		return Mat3f(
			{ a0[0], a1[0], a2[0] },
			{ a0[1], a1[1], a2[1] },
			{ a0[2], a1[2], a2[2] }
		);
#endif
	}


	// ========== Matrix Transformation Functions ==========


	constexpr float PI = 3.14159265358979323846f;
	inline float degToRad(float degrees) {
		return degrees * (PI / 180.0f);
	}
	inline float radToDeg(float radians) {
		return radians * (180.0f / PI);
	}

	inline Mat4f perspective(float fovDeg, float aspect, float zn, float zf)
	{
		if (fovDeg <= 0 || fovDeg >= 180) throw std::invalid_argument("FOV in (0,180)");
		if (aspect <= 0)                  throw std::invalid_argument("Aspect > 0");
		if (zn <= 0 || zf <= 0)           throw std::invalid_argument("zn,zf > 0");
		if (zn >= zf)                     throw std::invalid_argument("zn < zf");
		const float f = 1.0f / std::tan(degToRad(fovDeg) * 0.5f);

		Mat4f m(0.0f);
		m[0][0] = f / aspect;
		m[1][1] = f;

		m[2][3] = -1.0f;
		const float a = zf / (zn - zf);
		const float b = (zn * zf) / (zn - zf);

		m[2][2] = a;   // note: negative
		m[3][2] = b;   // note: negative
		return m;
	}


	inline Mat4f translate(const Mat4f& m, const Vec3f& v_translation_xyz)
	{
		Mat4f Result(m);
		Result[3][0] += v_translation_xyz.x;
		Result[3][1] += v_translation_xyz.y;
		Result[3][2] += v_translation_xyz.z;
		return Result;
	}

	inline Mat4f lookAt(const Vec3f& eye, const Vec3f& center, const Vec3f& upWorld)
	{
		const Vec3f f = normalize(center - eye);         // forward (towards -Z in view space)
		const Vec3f r = normalize(cross(f, upWorld));    // right
		const Vec3f u = normalize(cross(r, f));          // up

		Mat4f V(1.0f);
		// basis in ROWS for row-vector math:
		V[0][0] = r.x; V[0][1] = r.y; V[0][2] = r.z; V[0][3] = 0;
		V[1][0] = u.x; V[1][1] = u.y; V[1][2] = u.z; V[1][3] = 0;
		V[2][0] = -f.x; V[2][1] = -f.y; V[2][2] = -f.z; V[2][3] = 0;

		// translation in LAST ROW for row-vector math:
		V[3][0] = -dot(r, eye);
		V[3][1] = -dot(u, eye);
		V[3][2] = dot(f, eye);
		V[3][3] = 1.0f;

		return V;
	}

	// Row-vector correction to turn Vulkan-clip into OpenGL-clip
	inline Mat4f vk_to_gl_correction_row()
	{
		Mat4f C(1.0f);
		// x' = x
		// y' = y
		// z' = 2*z - w
		// w' = w
		C[2][2] = 2.0f;
		C[3][2] = -1.0f;
		return C;
	}


};