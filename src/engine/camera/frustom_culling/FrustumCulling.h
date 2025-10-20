#pragma once
#include <engine/utils/utils.h>

namespace engine::frustum
{
	struct Frustom
	{

	private:
		// Order: left, right, bottom, top, near, far
		engine::utils::array<engine::math::Vec4f, 6> planes; // (nx, ny, nz, d)

		const engine::math::Vec4f clipPlanes[6] = {
		engine::math::Vec4f(+1,  0,  0, +1), // left
		engine::math::Vec4f(-1,  0,  0, +1), // right
		engine::math::Vec4f(0, +1,  0, +1), // bottom
		engine::math::Vec4f(0, -1,  0, +1), // top
		engine::math::Vec4f(0,  0, +1,  0), // near  (z >= 0)
		engine::math::Vec4f(0,  0, -1, +1)  // far   (z <= w)
		};

	public:

		inline engine::math::Vec4f col(const engine::math::Mat4f& m, int j) {
			// assuming m[r][c] access
			return { m[0][j], m[1][j], m[2][j], m[3][j] };
		}

		engine::utils::array<engine::math::Vec4f, 6> getPlanes(const engine::math::Mat4f& vpMat)
		{
			using namespace engine::math;

			const Vec4f c0 = col(vpMat, 0);
			const Vec4f c1 = col(vpMat, 1);
			const Vec4f c2 = col(vpMat, 2);
			const Vec4f c3 = col(vpMat, 3);

			planes[0] = normalizePlane(c0 + c3);   // left
			planes[1] = normalizePlane((c0 * -1.0f) + c3);  // right
			planes[2] = normalizePlane(c1 + c3);   // bottom
			planes[3] = normalizePlane((c1 * -1.0f) + c3);  // top
			planes[4] = normalizePlane(c2);        // near  (Vulkan)
			planes[5] = normalizePlane((c2 * -1.0f) + c3);  // far   (Vulkan)

			return planes;
		}


	};

	struct FrustumSoA
	{
		alignas(32) float nx[8], ny[8], nz[8], d[8]; // pad to 8 lanes for AVX
		int count = 6;
	};

	inline FrustumSoA makeFrustumSoA(const engine::utils::array<engine::math::Vec4f, 6>& planes)
	{
		FrustumSoA F{};
		for (int i = 0; i < 6; ++i) {
			F.nx[i] = planes[i].x;
			F.ny[i] = planes[i].y;
			F.nz[i] = planes[i].z;
			F.d[i] = planes[i].w;   // plane: n·x + d >= 0 is inside
		}
		// pad remaining lanes to keep AVX happy
		for (int i = 6; i < 8; ++i) {
			F.nx[i] = F.ny[i] = F.nz[i] = 0.0f;
			F.d[i] = -1e30f; // very negative so it never rejects
		}
		F.count = 6;
		return F;
	}
}

