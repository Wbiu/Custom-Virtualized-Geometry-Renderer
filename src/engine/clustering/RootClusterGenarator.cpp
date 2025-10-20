#include "RootClusterGenarator.h"

RootClusterGenarator::RootClusterGenarator(engine::cluster::InitialClusters& initialClusters)
	: _initialClusters(initialClusters)
{
}

RootClusterGenarator::~RootClusterGenarator()
{
}

void RootClusterGenarator::create(engine::cluster::RootClusterModel& clusteredModel)
{
	engine::cluster::dataClusterPoolTransfer(clusteredModel, _initialClusters);

	SpatialPartitioner _partitioner(clusteredModel);

	// default confs
	SpatialPartitionerConf conf;
	conf.newClusterType = engine::cluster::HierarchyLevel::LOD0;
	conf.prevClusterType = engine::cluster::HierarchyLevel::INITIAL_LEVEL;

	_partitioner.createClusters(conf);
}
