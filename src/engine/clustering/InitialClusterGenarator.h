#pragma once
#include "SpatialPartitioner.h"

class InitialClusterGenarator
{
private:
	engine::mesh::Model& _loadedModel;
	const unsigned int _DEFAULT_SUBDIVITION_COUNT = 8;


public:
	InitialClusterGenarator(engine::mesh::Model& loadedModel);
	~InitialClusterGenarator();
	
	void create(engine::cluster::InitialClusters& initialClusters);

};