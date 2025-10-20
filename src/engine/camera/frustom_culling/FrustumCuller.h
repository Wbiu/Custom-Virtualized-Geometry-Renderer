#pragma once
#include <engine/clustering/cluster.h>
#include <engine/camera/frustom_culling/FrustumCulling.h>

class FrustumCuller
{
private:

	engine::utils::array<engine::math::Vec4f, 6> _frustom_WorldSpace;
	engine::math::Vec4f _camPos;

public:
	FrustumCuller();
	~FrustumCuller();

	void setFrustomPlanes(engine::utils::array<engine::math::Vec4f, 6>& frustom_WorldSpace);
	void setCamPos(engine::math::Vec4f& camPos);

	bool intersectsOBB_AVX(const engine::math::Vec3f& C, const engine::math::Vec3f& ax, const engine::math::Vec3f& ay, const engine::math::Vec3f& az, const engine::math::Vec3f& H, const engine::frustum::FrustumSoA& F) const;

	bool inPlaneCheck(const engine::math::Vec3f& pos, const engine::math::Vec4f& plane);
	bool intersectsAABB(const engine::math::Vec4f& centerWorld,
		const engine::math::Vec4f& halfLocal,   // only x/y/z used
		const engine::math::Mat4f& model);

	float measureDistanceFromClusterCenter(const engine::math::Vec4f& center);
};