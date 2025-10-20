#pragma once
#include <engine/math/math.h>


namespace engine::mesh
{

	struct AABB
	{
		engine::math::Vec3f min{}, max{};
		void reset()
		{
			min = { FLT_MAX,FLT_MAX,FLT_MAX };
			max = { -FLT_MAX, -FLT_MAX, -FLT_MAX };
		}

		engine::math::Vec4f getCenter() const
		{
			engine::math::Vec3f res = (min + max) * 0.5f;
			return { res.x,res.y ,res.z ,1.0f};
		}

		engine::math::Vec4f getHalfExtent() const
		{
			engine::math::Vec3f res = (max - min) * 0.5f;
			return { res.x,res.y ,res.z ,0.0f };
		}

		void expand(const engine::math::Vec3f& point)
		{
#ifdef ENGINE_MATH_SIMD
			min.simd = _mm_min_ps(min.simd, point.simd);
			max.simd = _mm_max_ps(max.simd, point.simd);
#else
			min.x = std::min(min.x, point.x);
			min.y = std::min(min.y, point.y);
			min.z = std::min(min.z, point.z);

			max.x = std::max(max.x, point.x);
			max.y = std::max(max.y, point.y);
			max.z = std::max(max.z, point.z);
#endif // ENGINE_MATH_SIMD


		}
	};



};