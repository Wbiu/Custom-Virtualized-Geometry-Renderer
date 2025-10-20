#pragma once
#include "LODGraph_helper.h"

class LODGraphGenarator
{
private:

	//vars 
	engine::cluster::LODClusterMeshTree& _LODClusterMeshTree;
	engine::graph::LODGraph _graph;
	engineID_t _lastCreatedNodeID = 0;

	//end vars


	//functions


	//end functions

public:
	LODGraphGenarator(engine::cluster::LODClusterMeshTree&  LODClusterMeshTree);
	~LODGraphGenarator();

	void genarate();
	engine::graph::LODGraph getGraph() { return _graph; };
};
