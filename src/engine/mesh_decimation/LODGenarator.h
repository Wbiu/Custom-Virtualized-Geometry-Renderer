#pragma once
#include <engine/clustering/cluster.h>
#include <queue>
#include <algorithm>
#include <fstream>


namespace engine::lod
{
	struct LODGenaratorConf
	{
	
	};

	struct EdgeToCollapseCanidate
	{
		engineID_t edge;
		float error;
		engine::math::Vec4f optimal;
	};

}


struct CollapseResult {
	engineID_t w;           // merged vertex (new or snapped)
	engineID_t u,v;			// old endpoints (for Q update)
	std::vector<engineID_t> incidentEdges;  // edge IDs for (w,x) after rewiring
};

class LODGenarator
{

private:
	
	// variables
	
	const int _MAX_PRIM_CNT_PER_CLUSTER = 2000;
	const int _REDUCTION_RATE_PERCENTAGE = 90; // recomended MAX 90

	std::size_t statsDegeneracy[2] = { 0, 0 };
	std::size_t bucket[11] = { 0 };

	// globel acces to clustered Model
	engine::cluster::RootClusterModel& _rootClustermodel;

	struct RejectStats { size_t noQ = 0, area = 0, flip = 0; } g_rej;

	//std::ofstream logfile;

	// ---------- triangle flip / degeneracy guard var -------------------------
	const float _kAreaRelEps = 1e-3f;          // relative area threshold
	const float _kMaxFlipDeg = 60.0f;             // normal flip
	const float _kCosMaxFlip = std::cos(_kMaxFlipDeg * engine::math::PI / 180.0f);


	// end variables

	// function declaration
	engine::math::Vec4f calculate_primitive_normal(const engine::math::Vec4f& p_pos_0,
		const engine::math::Vec4f& p_pos_1,
		const engine::math::Vec4f& p_pos_2);
	void aggregateClusters_V2(engine::cluster::LODClusterMeshTree& LODClusterMeshTree,engine::cluster::HierarchyLevel prevType, engine::cluster::HierarchyLevel newType);
	// QEM functions
	std::unordered_map<engineID_t, engine::math::Mat4f> computeQEM_Quadrics_V2(engine::cluster::Cluster* cluster, engine::cluster::LODClusterMeshTree& LODClusterMeshTree, engine::cluster::HierarchyLevel level);
	engine::lod::EdgeToCollapseCanidate computeQEM_EdgeCost_V2(std::unordered_map<engineID_t, engine::math::Mat4f>& Q_mxMap,
		engine::mesh::Edge* edge,
		engine::cluster::Cluster* cluster,
		engine::cluster::LODClusterMeshTree& LODClusterMeshTree,
		engine::cluster::HierarchyLevel level);
	CollapseResult collapseEdge_v2(engine::lod::EdgeToCollapseCanidate& canidate,
		engine::cluster::Cluster* clusterPtr,
		engine::cluster::LODClusterMeshTree& LODClusterMeshTree,
		engine::cluster::HierarchyLevel clusterType,
		engine::cluster::HierarchyLevel prevClusterType);
	engine::math::Mat4f computeQ(engineID_t v_id, engine::cluster::LODClusterMeshTree& LODClusterMeshTree, engine::cluster::HierarchyLevel level);
	bool linkCheck(engine::lod::EdgeToCollapseCanidate canidate, engine::cluster::LODClusterMeshTree& LODClusterMeshTree, engine::cluster::HierarchyLevel& level);
	// end function
public:
	LODGenarator(engine::cluster::RootClusterModel& rootClustermodel);
	~LODGenarator();
	void generateLODs(engine::cluster::LODClusterMeshTree& LODClusterMeshTree);
};