#pragma once
#include <engine/clustering/cluster.h>
#include <vector>
#include <stack>

struct SpatialPartitionerConf
{
	unsigned int _MAX_SUBDIVISION_DEPTH = 8;
	unsigned int _DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT = 128;
	unsigned int _MIN_CLUSTER_SIZE_THRESHOLD = _DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT + (_DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT >> 3);
	unsigned int _LEAST_CLUSTER_SIZE = _DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT >> 2;
	unsigned int AABB_CNT = 8;
	engine::cluster::HierarchyLevel prevClusterType;
	engine::cluster::HierarchyLevel newClusterType;
};


struct TriAABBSoA {
	std::vector<float> minX, minY, minZ;
	std::vector<float> maxX, maxY, maxZ;
};


class SpatialPartitioner
{
private:

	// variables 
	engine::cluster::HierarchyLevel _prevClusterType = engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL;
	engine::cluster::HierarchyLevel _newClusterType = engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL;

	std::list<std::unique_ptr<engine::cluster::Cluster>> _clusterArena;

	const unsigned int _DEFAULT_SUBDIVITION_COUNT = 8;

	unsigned int _MAX_SUBDIVISION_DEPTH = 0;

	unsigned int _DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT = 128;

	unsigned int _MIN_CLUSTER_SIZE_THRESHOLD = _DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT + (_DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT >> 3);

	unsigned int _LEAST_CLUSTER_SIZE = _DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT >> 2;

	engine::cluster::ClusterPools& _clusterPools;
	std::unordered_map<engineID_t, std::unordered_set<engine::cluster::Cluster*>> _primitiveToClusterMapGlobal;
	std::unordered_map<engineID_t,engine::cluster::Cluster*> _leafClusters;
	engineID_t _leafClusterID = 0;

	TriAABBSoA _triSoA;                
	std::unordered_map<engineID_t, uint32_t> _primIdToIdx; // primID -> SoA index

	// end variables

	// function declarations


	// intersection functions
	void subdivideAABB(unsigned int& subdivitionCount, engine::mesh::AABB& aabb, std::unordered_map<engineID_t, engine::cluster::Cluster*>& clusters);
	std::vector<engine::mesh::AABB> subdivideAABBUniform(const engine::mesh::AABB& aabb, unsigned int numSubdivisions);
	engine::cluster::Cluster* createNewClusterArena(engineID_t id, engine::mesh::AABB aabb);
	engine::cluster::Cluster* createNewClusterArena(engineID_t id);
	void intersectionValidation_V2(engine::cluster::Cluster* cluster, std::unordered_set<engineID_t>* primitives, std::unordered_map<engineID_t, std::unordered_set<engineID_t>>& primitiveToClusterTrackMap);
	uint32_t triAABBOverlapMask8_ids(const TriAABBSoA& S, const uint32_t idx[8], int count, const engine::math::Vec3f& bMin, const engine::math::Vec3f& bMax);
	void buildGlobalTriAABBSoA(const std::vector<engineID_t>& allPrimIds, const engine::cluster::ClusterPools& pools, engine::cluster::HierarchyLevel level);
	bool triBoxOverlap(const engine::mesh::Primitive* prim, const engine::mesh::AABB& aabb);
	void subdivideCluster(unsigned int aabbCnt, engine::cluster::Cluster* cluster,unsigned int DEPTH_COUNTER);
	engine::cluster::Cluster* pickBestNeighbour(engine::cluster::Cluster* current, const std::vector<engineID_t>& part, const std::unordered_map<engineID_t, std::unordered_set<engine::cluster::Cluster*>>& p2c);
	engine::cluster::Cluster* pickBestNeighbour(engine::cluster::Cluster* current, const std::unordered_set<engineID_t>& part, const std::unordered_map<engineID_t, std::unordered_set<engine::cluster::Cluster*>>& p2c);
	void recomputeClusterAABB(engine::cluster::Cluster* cluster);
	void connectivityValidation_V2(std::unordered_set<engineID_t>* primitivesMap, std::vector<std::unique_ptr<std::vector<engineID_t>>>& foundClusters);
	// end function declaration
public:
	SpatialPartitioner(engine::cluster::ClusterPools& clustrPools);
	~SpatialPartitioner();
	void createClusters(const SpatialPartitionerConf& conf);
};

