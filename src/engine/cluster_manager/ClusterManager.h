#pragma once
#include <engine/camera/frustom_culling/FrustumCuller.h>
#include <engine/camera/frustom_culling/FrustumCulling.h>
#include <engine/cluster_manager/LODGraph/LODGraph_helper.h>

class ClusterManager
{
private:
	
	// vars
	const engine::cluster::LODClusterMeshTree& _lodMeshTree;
	engine::graph::LODGraph _graph;

	std::unordered_set<engineID_t> _currentVisibleClusters;
	std::unordered_set<engineID_t> _newVisibleClusters;
	std::unordered_map<engineID_t,float> _newVisibleClustersDistance;

	FrustumCuller _culler;
	engine::frustum::FrustumSoA _soa;

	// distances
	const float _LOD0_DISTANCE = 1.0f, _LOD1_DISTANCE = 2.0f, _LOD2_DISTANCE = 3.0f;

	//end vars

	// fuctions
	engine::cluster::HierarchyLevel determinLODLevel(float distance);
	
	void pickLODLevel();

	static inline bool isCoarser(engine::cluster::HierarchyLevel a,
		engine::cluster::HierarchyLevel b) {
		return static_cast<int>(a) > static_cast<int>(b);
	}

	static inline bool isFiner(engine::cluster::HierarchyLevel a,
		engine::cluster::HierarchyLevel b) {
		return static_cast<int>(a) < static_cast<int>(b);
	}

	//end functions

public:

	ClusterManager(const engine::cluster::LODClusterMeshTree& lodMeshTree,engine::graph::LODGraph graph);
	~ClusterManager();

	void cullClusters_V2(engine::math::Mat4f& modelMat);

	void cullClusters(engine::math::Mat4f& modelMat);
	void setfrustumPlanesAndCamPos(engine::utils::array<engine::math::Vec4f, 6>& planes,engine::math::Vec4f& camPos);
	int getIntersectedClusterCnt();
	int getSelectedClusterCnt() { return _currentVisibleClusters.size(); };
	std::unordered_set<engineID_t>& getClustersToDraw() { return _currentVisibleClusters; };
	engine::cluster::Cluster* getCluster(engineID_t cid) { return _lodMeshTree.clusters.at(cid); };
};