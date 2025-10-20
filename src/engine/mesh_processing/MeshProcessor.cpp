#include "MeshProcessor.h"

MeshProcessor::MeshProcessor()
{
}

MeshProcessor::~MeshProcessor()
{
}

void MeshProcessor::process(const char* modelPath, std::pmr::memory_resource* mr)
{
	loadModel(modelPath,mr);
	clusterModel();
	createLODs();
	cleanUp();
}

void MeshProcessor::loadModel(const char* modelPath, std::pmr::memory_resource* mr)
{
	_modelLoader = new ModelLoader();
	_loadedModel = _modelLoader->load(modelPath,mr);
	delete(_modelLoader);

}

void MeshProcessor::clusterModel()
{
	// Cluster genaration
	_clusterGenarator = new ClusterGenarator(_loadedModel);

	// creates clusters
	_clusteredModel = _clusterGenarator->createClusters();

	delete(_clusterGenarator);
}

void MeshProcessor::createLODs()
{
		_lodGenarator = new LODGenarator(_clusteredModel);

		_lodGenarator->generateLODs(_LODClusterMeshTree);

		delete(_lodGenarator);

		// LOD Graph
		_lodGraphGenarator = new LODGraphGenarator(_LODClusterMeshTree);
		_lodGraphGenarator->genarate();

		_clusterManager = new ClusterManager(_LODClusterMeshTree, _lodGraphGenarator->getGraph());

		BufferDataGenarator buffGen(_LODClusterMeshTree);
		_linearizedData = new engine::mesh::LinearizedModelData();
		buffGen.createBufferData(_linearizedData);
}

void MeshProcessor::cleanUp()
{
	for (auto& kv : _LODClusterMeshTree.vertexPool)     delete kv.second;
	for (auto& kv : _LODClusterMeshTree.edgePool)       delete kv.second;
	for (auto& kv : _LODClusterMeshTree.primitivePool)  delete kv.second;

	_LODClusterMeshTree.vertexToPrimMap.clear();

	_LODClusterMeshTree.edgeKeyToEdgeMap.clear();
	_LODClusterMeshTree.edgeMap.clear();
	_LODClusterMeshTree.edgePool.clear();

	_LODClusterMeshTree.vertexIndcicesBufferOrder.clear();

	_LODClusterMeshTree.primKeyToPrimMap.clear();
	_LODClusterMeshTree.primitiveMap.clear();
	_LODClusterMeshTree.primitivePool.clear();

	if (auto* pool = dynamic_cast<std::pmr::unsynchronized_pool_resource*>(
		_LODClusterMeshTree.memoryPool)) {
		pool->release(); // returns free blocks to upstream (new_delete_resource)
	}
	else if (auto* mono = dynamic_cast<std::pmr::monotonic_buffer_resource*>(
		_LODClusterMeshTree.memoryPool)) {
		mono->release(); // frees all memory from this resource
	}
}