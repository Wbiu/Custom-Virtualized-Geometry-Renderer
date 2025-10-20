#include "BufferDataGenarator.h"


BufferDataGenarator::BufferDataGenarator(engine::cluster::ClusterPools& pool)
	: _pool(pool)
{

}

BufferDataGenarator::~BufferDataGenarator()
{
}

void BufferDataGenarator::createBufferData(engine::mesh::LinearizedModelData* linearizedData)
{

    linearizedData->vertices.reserve(_pool.vertexPool.size());

    // vertices
    for (engineID_t i = 1; i < (_pool.vertexPool.size() + 1); ++i) {
        linearizedData->vertexIdToIdx[_pool.vertexPool.at(i)->id] = static_cast<uint32_t>(linearizedData->vertices.size());
        linearizedData->vertices.emplace_back(*_pool.vertexPool.at(i));
    }

    // indices laid out level -> cluster -> primitives
    uint32_t runningIndex = 0;

    for (auto& [level, c_ids] : _pool.clusterTypeMap) {
        // skip non-render levels as you already do
        if (level < engine::cluster::HierarchyLevel::LOD0) continue;

        linearizedData->perLevelStartingVertexIdx[level] = runningIndex;
        uint32_t levelCount = 0;

        for (engineID_t c_id : c_ids) {
            auto* c = _pool.clusters.at(c_id);

            const uint32_t clusterFirst = runningIndex;

            for (engineID_t p_id : c->primitives) {
                auto* p = _pool.primitivePool.at(p_id);
                // append 3 indices
                for (int i = 0; i < 3; ++i) {
                    linearizedData->indices.emplace_back(linearizedData->vertexIdToIdx.at(p->vertices[i]));
                    ++runningIndex;
                }
            }

            const uint32_t clusterCount = runningIndex - clusterFirst;
            linearizedData->clusterDrawRange[c_id] = { clusterFirst, clusterCount, c_id, level };
            levelCount += clusterCount;
        }

        linearizedData->perLevelIndexCount[level] = levelCount;
    }
}
