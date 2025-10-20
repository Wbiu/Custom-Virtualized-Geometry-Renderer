#pragma once
#include <engine/mesh/mesh.h>
#include <unordered_map>
#include <map>


namespace engine::cluster
{
	enum PoolType
	{
		INITIAL_CLUSTER_POOL,
		ROOT_CLUSTERED_MODEL_POOL,
		LOD_MESH_TREE,
		UNASSIGNED_TYPE
	};

	struct ClusterPools : mesh::Model
	{
		engineID_t lastcreatedClusterID = 0;
		engine::cluster::PoolType type = engine::cluster::PoolType::UNASSIGNED_TYPE;
		std::unordered_map<engineID_t, engine::cluster::Cluster*> clusters;
		std::unordered_map<engineID_t, std::unordered_set<engineID_t>> primToCluster;
		std::unordered_map<HierarchyLevel, std::unordered_set<engineID_t>> clusterTypeMap; // Type to cluster type
		std::unordered_map<HierarchyLevel, std::unordered_map<engineID_t, engineID_t>> clusterTypePrimitivePool; // Type assigned pimitive 
	};


	struct Cluster
	{
		engineID_t id = 0;
		engineID_t parent = 0;
		engine::cluster::HierarchyLevel type = engine::cluster::HierarchyLevel::INITIAL_LEVEL;
		engine::mesh::AABB aabb;
		std::unordered_set<engineID_t> primitives;
		std::unordered_set<engineID_t> children;
		std::unordered_set<engineID_t> neighborClusters;

		std::unordered_set<engineID_t> edges;		//local edge
		std::unordered_set<engineID_t> innerEdge;
		std::unordered_set<engineID_t> boundaryEdges;			// edges that surrounds the cluster
		std::unordered_set<engineID_t> boundaryRingEdges;			// edges that are connected to the outer Edges
		std::unordered_set<engineID_t> boundaryVertices;
	};


	inline void computeClusterAABB(Cluster* cluster, const ClusterPools& pool, HierarchyLevel clusterLevel)
	{
		cluster->aabb.reset(); // min=+inf, max=-inf
		for (const auto& p_id : cluster->primitives)
		{
			auto id = *pool.primitiveMap.at(clusterLevel).find(p_id);
			auto primPtr = pool.primitivePool.at(id);
			for (int i = 0; i < 3; ++i)
			{
				auto vertexPtr = pool.vertexPool.at(primPtr->vertices[i]);
				cluster->aabb.expand(engine::math::Vec3f{ vertexPtr->coords.xyzw() });
			}
		}
	}

	struct InitialClusters : ClusterPools
	{
		InitialClusters() {
			type = engine::cluster::PoolType::INITIAL_CLUSTER_POOL;
		};
	};


	struct RootClusterModel : ClusterPools
	{
		RootClusterModel() {
			type = engine::cluster::PoolType::ROOT_CLUSTERED_MODEL_POOL;
		};

	};

	struct LODClusterMeshTree : ClusterPools
	{
		LODClusterMeshTree() {
			type = engine::cluster::PoolType::LOD_MESH_TREE;
		};
	};

	inline void dataClusterPoolTransfer(ClusterPools& dest, ClusterPools& src)
	{
		dest = std::move(src);
	}

	inline void outlineCluster(ClusterPools& root,const engine::cluster::HierarchyLevel clusterType)
	{
		unsigned int minimumEncounteredNeightCnt = 4;

		auto& clusterIDs = root.clusterTypeMap.find(clusterType)->second;
		auto& primitiveTocluster = root.clusterTypePrimitivePool.find(clusterType)->second;

		for (const engineID_t& c_id : clusterIDs)
		{
			auto& clusterPtr = root.clusters.find(c_id)->second;

			size_t edgeCnt = clusterPtr->edges.size();

			clusterPtr->neighborClusters.clear();
			clusterPtr->innerEdge.clear();
			clusterPtr->boundaryEdges.clear();
			clusterPtr->boundaryRingEdges.clear();
			clusterPtr->boundaryVertices.clear();

			clusterPtr->neighborClusters.reserve(8);
			clusterPtr->innerEdge.reserve(edgeCnt);
			clusterPtr->boundaryEdges.reserve(edgeCnt);
			clusterPtr->boundaryRingEdges.reserve(edgeCnt);
			clusterPtr->boundaryVertices.reserve(edgeCnt * 2);


			std::pmr::monotonic_buffer_resource arena(1 << 20); // 1 MB chunk (tune)

			std::pmr::unordered_set<engineID_t> boundaryPrimsTmp{&arena};
			std::pmr::unordered_map<engineID_t, engineID_t> clusterEncounterCnt{ &arena };

			std::vector<engineID_t> innerEdgeTmp;

			clusterEncounterCnt.reserve(16);
			boundaryPrimsTmp.reserve(edgeCnt);

			innerEdgeTmp.reserve(edgeCnt);

			for (auto& e_id : clusterPtr->edges)
			{
				const auto edgePtr = root.edgePool.at(e_id);

				const auto* prims = &edgePtr->prims.find(clusterType)->second; // find once

				const engineID_t&& p0 = (*prims)[0], p1 = (*prims)[1];

				if (p0 == 0 || p1 == 0)
				{
					clusterPtr->boundaryEdges.insert(e_id);

					clusterPtr->boundaryRingEdges.insert(e_id);

					clusterPtr->boundaryVertices.insert(edgePtr->vertices[0]);
					clusterPtr->boundaryVertices.insert(edgePtr->vertices[1]);

					engineID_t& boundaryPrim = edgePtr->prims.find(clusterType)->second[0] == 0 ? edgePtr->prims.find(clusterType)->second[1] : edgePtr->prims.find(clusterType)->second[0];

					boundaryPrimsTmp.insert(boundaryPrim);
					continue; // skip to next edge
				}
				else
				{
					const engineID_t& cluster_of_prim_1 = primitiveTocluster.find(edgePtr->prims.find(clusterType)->second[0])->second;
					const engineID_t& cluster_of_prim_2 = primitiveTocluster.find(edgePtr->prims.find(clusterType)->second[1])->second;

					// innner edge case
					if (cluster_of_prim_1 == cluster_of_prim_2)
					{
						innerEdgeTmp.push_back(e_id);
					}
					else // border edge case
					{
						// marking border prims
						if (cluster_of_prim_1 == c_id)
						{
							boundaryPrimsTmp.insert(edgePtr->prims.at(clusterType)[0]);
							// marking neibor and count
							++clusterEncounterCnt[cluster_of_prim_2];
						}
						else
						{
							boundaryPrimsTmp.insert(edgePtr->prims.at(clusterType)[1]);
							// marking neibor and count
							++clusterEncounterCnt[cluster_of_prim_1];
						}
						
						// adding boundary edges and vertices
						clusterPtr->boundaryEdges.insert(e_id);
						clusterPtr->boundaryRingEdges.insert(e_id);
						clusterPtr->boundaryVertices.insert(edgePtr->vertices[0]);
						clusterPtr->boundaryVertices.insert(edgePtr->vertices[1]);
					}

				}

			}

			// resolving inner and ring edges
			for (auto& e_id : innerEdgeTmp)
			{
				auto e_ptr = root.edgePool.find(e_id)->second;

				bool e_prim_1 = boundaryPrimsTmp.find(e_ptr->prims.find(clusterType)->second[0]) == boundaryPrimsTmp.end();
				bool e_prim_2 = boundaryPrimsTmp.find(e_ptr->prims.find(clusterType)->second[1]) == boundaryPrimsTmp.end();

				bool isRingEge = e_prim_1 != e_prim_2;

				bool connects_to_boundary_vert_1 = clusterPtr->boundaryVertices.find(e_ptr->vertices[0]) == clusterPtr->boundaryVertices.end() ? false : true;
				bool connects_to_boundary_vert_2 = clusterPtr->boundaryVertices.find(e_ptr->vertices[1]) == clusterPtr->boundaryVertices.end() ? false : true;

				if (connects_to_boundary_vert_1 && connects_to_boundary_vert_2)
				{
					// adding boundary edges and vertices
					clusterPtr->boundaryEdges.insert(e_id);
					clusterPtr->boundaryRingEdges.insert(e_id);
					clusterPtr->boundaryVertices.insert(e_ptr->vertices[0]);
					clusterPtr->boundaryVertices.insert(e_ptr->vertices[1]);
				}
				else if (isRingEge)
				{
					clusterPtr->boundaryRingEdges.insert(e_id);
				}
				else
				{
					clusterPtr->innerEdge.insert(e_id);
				}

			}

			// resolving neighbor adjacency
			for (auto& [c_id, cnt] : clusterEncounterCnt)
			{
				if (cnt >= minimumEncounteredNeightCnt)
					clusterPtr->neighborClusters.insert(c_id);
			}

		}
	}

};
