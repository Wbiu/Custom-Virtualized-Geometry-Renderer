#pragma once

#include "InitialClusterGenarator.h"
#include "RootClusterGenarator.h"

class ClusterGenarator
{

private:
	// variables 
	std::unique_ptr<engine::mesh::Model> _modelTmpPtr; 

	InitialClusterGenarator* _initialClusterGenarator = nullptr;
	std::vector<std::unique_ptr<SpatialPartitioner>> _partitioners;

	engine::cluster::InitialClusters _initialClusters;
	engine::cluster::RootClusterModel _clusteredModel;

	std::unordered_map<int,engine::cluster::Cluster*> _clusterMap;
	int _lastCreatedClusterID = 1;


	// end vairbles


	// function declarations
	void clustering();
	// end function declarations


public:
	ClusterGenarator(engine::mesh::Model* loadedModel);
	~ClusterGenarator();

	engine::cluster::RootClusterModel createClusters();

};

