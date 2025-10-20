#pragma once

#include "SpatialPartitioner.h"

class RootClusterGenarator
{
private:

	engine::cluster::InitialClusters& _initialClusters;

public:


	RootClusterGenarator(engine::cluster::InitialClusters& initialClusters);
	~RootClusterGenarator();

	void create(engine::cluster::RootClusterModel& clusteredModel);

};