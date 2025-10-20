#pragma once
#include <engine/loader/ModelLoader.h>
#include <engine/clustering/ClusterGenarator.h>
#include <engine/mesh_decimation/LODGenarator.h>
#include <engine/renderer/buffer_data_genarator/BufferDataGenarator.h>
#include <engine/cluster_manager/LODGraph/LODGraphGenarator.h>
#include <engine/cluster_manager/ClusterManager.h>

class MeshProcessor
{
private:

	//vars
	ModelLoader* _modelLoader = nullptr;

	engine::mesh::Model* _loadedModel;
	ClusterGenarator* _clusterGenarator = nullptr;
	engine::cluster::RootClusterModel _clusteredModel;

	LODGenarator* _lodGenarator = nullptr;

	engine::mesh::LinearizedModelData* _linearizedData;

	ClusterManager* _clusterManager = nullptr;

	LODGraphGenarator* _lodGraphGenarator = nullptr;

	engine::cluster::LODClusterMeshTree  _LODClusterMeshTree;

	// end vars

	// functions

	void loadModel(const char* modelPath, std::pmr::memory_resource* mr);
	void clusterModel();
	void createLODs();
	void cleanUp();
	// end functions


public:

	MeshProcessor();
	~MeshProcessor();
	void process(const char* modelPath, std::pmr::memory_resource* mr);
	ClusterManager* getClusterManager() { return _clusterManager; };
	engine::mesh::LinearizedModelData* getLinearizedMeshData() { return _linearizedData; };
};
