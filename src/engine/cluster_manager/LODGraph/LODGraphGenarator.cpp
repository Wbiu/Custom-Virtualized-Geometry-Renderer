#include "LODGraphGenarator.h"

LODGraphGenarator::LODGraphGenarator(engine::cluster::LODClusterMeshTree& LODClusterMeshTree)
	: _LODClusterMeshTree(LODClusterMeshTree)
{
}

LODGraphGenarator::~LODGraphGenarator()
{
}

void LODGraphGenarator::genarate()
{
	engine::cluster::HierarchyLevel levels[] = { engine::cluster::HierarchyLevel::LOD0, engine::cluster::HierarchyLevel::LOD1,engine::cluster::HierarchyLevel::LOD2};

	for (int i = 0; i < 3; i++)
	{
		engine::cluster::HierarchyLevel curlevel = levels[i];

		engine::cluster::HierarchyLevel prevlevel = i > 0 ? levels[i - 1] : levels[i];

		engine::cluster::HierarchyLevel nextlevel = (i + 1) < 3 ? levels[i + 1] : levels[i];

		auto& clusters = _LODClusterMeshTree.clusterTypeMap.at(curlevel);


		auto& vec = _graph.graph[curlevel];
		vec.reserve(clusters.size()); // reduce reallocs

		for (auto& c_id : clusters)
		{
			auto* c_ptr = _LODClusterMeshTree.clusters.at(c_id);
			
			engine::graph::LODNode node;
			node.id = _lastCreatedNodeID++;
			node.clusterId = c_id;
			node.currentlevel = curlevel;
			node.prevLevel = prevlevel;
			node.nextLevel = nextlevel;

			node.parentClusterId = c_ptr->parent;

			node.childClusterIds.reserve(c_ptr->children.size());

			for (auto& child_id : c_ptr->children)
			{
				node.childClusterIds.emplace_back(child_id);
			}

			// add node to the node pool
			_graph.nodes[node.id] = node;

			// reference by ID
			vec.push_back(std::move(node.id));
			_graph.clusterToNode[curlevel][c_id] = node.id;
			_graph.clusterToLevelMap[c_id] = curlevel;
		}

	}

	for (int i = 0; i < 3; i++)
	{
		engine::cluster::HierarchyLevel curlevel = levels[i];
		engine::cluster::HierarchyLevel prevlevel = i > 0 ? levels[i - 1] : levels[i];
		engine::cluster::HierarchyLevel nextlevel = (i + 1) < 3 ? levels[i + 1] : levels[i];

		auto& nodeIDs = _graph.graph.at(curlevel);


		for (auto& nodeID : nodeIDs)
		{
			auto& node = _graph.nodes.at(nodeID);
			// check if reached last level
			// referencing parent node ID
			node.parentNodeId = curlevel == nextlevel ? 0 : _graph.clusterToNode.at(nextlevel).at(node.parentClusterId);

			node.childNodeIds.reserve(node.childClusterIds.size());

			// referencing children node IDs
			for (auto& childClusterId : node.childClusterIds)
			{
				engineID_t childNodeId = _graph.clusterToNode.at(prevlevel).at(childClusterId);
				node.childNodeIds.push_back(childNodeId);
			}
		}

	}
}

