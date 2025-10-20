#include "FrustumCuller.h"

FrustumCuller::FrustumCuller()
{

}

FrustumCuller::~FrustumCuller()
{
}

void FrustumCuller::setFrustomPlanes(engine::utils::array<engine::math::Vec4f, 6>& frustom_WorldSpace)
{
	_frustom_WorldSpace = frustom_WorldSpace;
}

// Planes are (nx,ny,nz,d) in world space (ideally normalized).
bool FrustumCuller::intersectsAABB(const engine::math::Vec4f& centerWorld,
    const engine::math::Vec4f& halfLocal,   // only x/y/z used
    const engine::math::Mat4f& model)
{
    using namespace engine::math;

    // World-space center as Vec3f
    const Vec3f C{ centerWorld.x, centerWorld.y, centerWorld.z };

    // World-space orientation/scale of the local axes = rows of model
    const Vec3f ax{ model[0][0], model[0][1], model[0][2] }; // local +X in world
    const Vec3f ay{ model[1][0], model[1][1], model[1][2] }; // local +Y in world
    const Vec3f az{ model[2][0], model[2][1], model[2][2] }; // local +Z in world

    const Vec3f H{ halfLocal.x, halfLocal.y, halfLocal.z };  // (hx,hy,hz)

    for (int i = 0; i < 6; ++i)
    {
        const Vec4f& P = _frustom_WorldSpace[i];   // plane (n,d)
        const Vec3f  n{ P.x, P.y, P.z };

        // signed distance from center to plane
        const float dist = dot(n, C) + P.w;

        // projected radius of OBB onto plane normal
        const float r =
            std::fabs(dot(n, ax)) * H.x +
            std::fabs(dot(n, ay)) * H.y +
            std::fabs(dot(n, az)) * H.z;

        if (dist < -r)           // fully outside this plane
            return false;
    }
    return true;                 // intersects or inside
}

bool FrustumCuller::inPlaneCheck(const engine::math::Vec3f& pos, const engine::math::Vec4f& plane)
{
	engine::math::Vec3f normal = { plane.x,plane.y,plane.z };

	return engine::math::dot(normal, pos) + plane.w < 0.0f; // outside if < 0
}

float FrustumCuller::measureDistanceFromClusterCenter(const engine::math::Vec4f& center)
{
	return engine::math::length(center - _camPos);
}

void FrustumCuller::setCamPos(engine::math::Vec4f& camPos)
{
	_camPos = camPos;
}

bool FrustumCuller::intersectsOBB_AVX(
    const engine::math::Vec3f& C,
    const engine::math::Vec3f& ax, 
    const engine::math::Vec3f& ay, 
    const engine::math::Vec3f& az,
    const engine::math::Vec3f& H,
    const engine::frustum::FrustumSoA& F) const
{
#if defined(__AVX__)
    __m256 nx = _mm256_load_ps(F.nx);
    __m256 ny = _mm256_load_ps(F.ny);
    __m256 nz = _mm256_load_ps(F.nz);
    __m256 dd = _mm256_load_ps(F.d);

    // dist = n·C + d
    __m256 dist = _mm256_add_ps(dd,
        _mm256_add_ps(_mm256_mul_ps(nx, _mm256_set1_ps(C.x)),
            _mm256_add_ps(_mm256_mul_ps(ny, _mm256_set1_ps(C.y)),
                _mm256_mul_ps(nz, _mm256_set1_ps(C.z)))));

    // r = |n·ax|*Hx + |n·ay|*Hy + |n·az|*Hz
    auto vabs = [](__m256 v) { return _mm256_andnot_ps(_mm256_set1_ps(-0.0f), v); };

    __m256 dax = _mm256_add_ps(_mm256_mul_ps(nx, _mm256_set1_ps(ax.x)),
        _mm256_add_ps(_mm256_mul_ps(ny, _mm256_set1_ps(ax.y)),
            _mm256_mul_ps(nz, _mm256_set1_ps(ax.z))));
    __m256 day = _mm256_add_ps(_mm256_mul_ps(nx, _mm256_set1_ps(ay.x)),
        _mm256_add_ps(_mm256_mul_ps(ny, _mm256_set1_ps(ay.y)),
            _mm256_mul_ps(nz, _mm256_set1_ps(ay.z))));
    __m256 daz = _mm256_add_ps(_mm256_mul_ps(nx, _mm256_set1_ps(az.x)),
        _mm256_add_ps(_mm256_mul_ps(ny, _mm256_set1_ps(az.y)),
            _mm256_mul_ps(nz, _mm256_set1_ps(az.z))));

    __m256 r = _mm256_add_ps(_mm256_mul_ps(vabs(dax), _mm256_set1_ps(H.x)),
        _mm256_add_ps(_mm256_mul_ps(vabs(day), _mm256_set1_ps(H.y)),
            _mm256_mul_ps(vabs(daz), _mm256_set1_ps(H.z))));

    // compare: dist >= -r  <=>  dist + r >= 0
    __m256 cmp = _mm256_cmp_ps(_mm256_add_ps(dist, r), _mm256_set1_ps(0.0f), _CMP_GE_OQ);

    // Build mask for the first 6 lanes
    int mask = _mm256_movemask_ps(cmp) & 0x3F;
    return mask == 0x3F; // all 6 lanes true => intersects/inside
#else
    // Fallback to scalar
    return intersectsOBB_branchless(C, ax, ay, az, H);
#endif
}