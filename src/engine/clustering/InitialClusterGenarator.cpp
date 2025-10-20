#include "InitialClusterGenarator.h"

InitialClusterGenarator::InitialClusterGenarator(engine::mesh::Model& loadedModel)
    : _loadedModel(loadedModel)
{
    
}

InitialClusterGenarator::~InitialClusterGenarator()
{
}

void InitialClusterGenarator::create(engine::cluster::InitialClusters& initialClusters)
{

    engine::mesh::dataPoolTransfer(initialClusters, _loadedModel);

    // creating cluster that with hold all the primitive
    // as the starter cluster to be further subdived.
    // ONLY cluster/s be  subdivided
    // Not pools
    engine::cluster::Cluster* c = new engine::cluster::Cluster();
    c->aabb = initialClusters.aabb;
    for (auto& [p_id, p_ptr] : initialClusters.primitivePool)
        c->primitives.insert(p_id);

    initialClusters.clusters[c->id] = c;

    SpatialPartitioner _partitioner(initialClusters);

    int totalPrimitivesCount = static_cast<int>(initialClusters.primitivePool.size());
    unsigned int maxsubdivisionCnt = 1;
    unsigned int subdivitionCount = _DEFAULT_SUBDIVITION_COUNT; // default
    if (totalPrimitivesCount < 5000) subdivitionCount = 2;
    else if (totalPrimitivesCount < 10000) subdivitionCount = 4;


    // determin subdivision parameter
    SpatialPartitionerConf conf;
    conf._MAX_SUBDIVISION_DEPTH = 1;
    conf._DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT = c->primitives.size() / subdivitionCount;
    conf._MIN_CLUSTER_SIZE_THRESHOLD = conf._DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT + (conf._DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT >> 3);
    conf._LEAST_CLUSTER_SIZE = conf._DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT >> 2;
    conf.AABB_CNT = subdivitionCount;
    conf.prevClusterType = engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL;
    conf.newClusterType = engine::cluster::HierarchyLevel::INITIAL_LEVEL;

    _partitioner.createClusters(conf);
}
