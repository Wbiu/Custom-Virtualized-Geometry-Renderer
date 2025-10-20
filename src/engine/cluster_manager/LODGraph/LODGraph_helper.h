#pragma once
#include <engine/clustering/cluster.h>

namespace engine::graph
{
	struct LODNode
	{
		engineID_t id;
		engineID_t parentNodeId;

		engineID_t parentClusterId;
		engineID_t clusterId;

		std::vector<engineID_t> childClusterIds;
		std::vector<engineID_t> childNodeIds;

		engine::cluster::HierarchyLevel currentlevel;
		engine::cluster::HierarchyLevel nextLevel;
		engine::cluster::HierarchyLevel prevLevel;

	};

	struct LODGraph
	{
		engineID_t id = 0;
		std::unordered_map<engineID_t, LODNode> nodes; // node pool
		std::unordered_map<engine::cluster::HierarchyLevel,std::vector<engineID_t>> graph; // references by node ids
		std::unordered_map<engine::cluster::HierarchyLevel,std::unordered_map<engineID_t /*clusterId*/, engineID_t /*nodeId*/>> clusterToNode;
		std::unordered_map<engineID_t /*cluster id*/, engine::cluster::HierarchyLevel> clusterToLevelMap;
	};

	struct LODTransition
	{
		engine::cluster::HierarchyLevel currentLevel;
		engine::cluster::HierarchyLevel targetLevel;
	};

}