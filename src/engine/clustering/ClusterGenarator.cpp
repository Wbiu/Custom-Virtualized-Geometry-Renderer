#include "ClusterGenarator.h"


ClusterGenarator::ClusterGenarator(engine::mesh::Model* loadedModel)
	: _modelTmpPtr(loadedModel)
{

}

ClusterGenarator::~ClusterGenarator()
{
}

engine::cluster::RootClusterModel ClusterGenarator::createClusters()
{
	_initialClusterGenarator = new InitialClusterGenarator(*_modelTmpPtr.get());

	// the number of returned initial clusters will depend on the number of the primitives
 	_initialClusterGenarator->create(_initialClusters);

	engine::cluster::outlineCluster(_initialClusters,engine::cluster::HierarchyLevel::INITIAL_LEVEL);

	free(_initialClusterGenarator);
	clustering();
	return _clusteredModel;
}

void ClusterGenarator::clustering()
{
	// from the initial clusters
	// create the root clusters 
	RootClusterGenarator rootgen(_initialClusters);
	rootgen.create(_clusteredModel);
	engine::cluster::outlineCluster(_clusteredModel, engine::cluster::HierarchyLevel::LOD0);
}


