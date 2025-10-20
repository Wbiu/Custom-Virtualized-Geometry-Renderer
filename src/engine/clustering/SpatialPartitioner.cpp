#include "SpatialPartitioner.h"



SpatialPartitioner::SpatialPartitioner(engine::cluster::ClusterPools& clusterPools)
	: _clusterPools(clusterPools)
{
}

SpatialPartitioner::~SpatialPartitioner()
{
}

void SpatialPartitioner::createClusters(const SpatialPartitionerConf& conf)
{
    _prevClusterType = conf.prevClusterType;
    _newClusterType = conf.newClusterType;

    _MAX_SUBDIVISION_DEPTH = conf._MAX_SUBDIVISION_DEPTH > 8 ? _DEFAULT_SUBDIVITION_COUNT : conf._MAX_SUBDIVISION_DEPTH;
    _DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT = conf._DEFAULT_MINIMUM_CLUSTER_SIZE_COUNT;
    _MIN_CLUSTER_SIZE_THRESHOLD = conf._MIN_CLUSTER_SIZE_THRESHOLD;
    _LEAST_CLUSTER_SIZE = conf._LEAST_CLUSTER_SIZE;


    // 1) Collect the primitive IDs you’ll process
    std::vector<engineID_t> allPrimIds;
    allPrimIds.reserve(_clusterPools.primitivePool.size());
    for (const auto& [pid, primPtr] : _clusterPools.primitivePool)
        allPrimIds.push_back(pid);

    // 2) Build the global SoA and the primID->index map once
    buildGlobalTriAABBSoA(allPrimIds, _clusterPools,_prevClusterType);

	for (auto& [c_id, c_ptr] : _clusterPools.clusters)
	{
		subdivideCluster(conf.AABB_CNT, c_ptr, 1 /* start at depth*/);
	}

    // clear out the first cluster 
    _clusterPools.clusters.clear();

    // releasing the leaf cluster from the areana
    // to prevent it to be destoryed
    for (auto& [id, c_ptr] : _leafClusters)
    {
        for (std::unique_ptr<engine::cluster::Cluster>& arena : _clusterArena)
        {
            if (c_ptr == arena.get())
            {
                arena.release();
                break;
            }
        }
    }


    _clusterPools.clusters = std::move(_leafClusters);

    // the highest ID will be the last created ID
    engineID_t highestCreatedID = 0;

    // filling the pritive to cluster map
    for (auto& [c_id, c_ptr] : _clusterPools.clusters)
    {

        if (c_id > highestCreatedID)
            highestCreatedID = c_id;

        _clusterPools.clusterTypeMap[_newClusterType].insert(c_id);

        for (auto& prim_id : c_ptr->primitives)
        {
            _clusterPools.primToCluster[prim_id].insert(c_id);
            _clusterPools.clusterTypePrimitivePool[_newClusterType][prim_id] = c_id;

            //filling the local edge set
            auto p_id = *_clusterPools.primitiveMap.at(_prevClusterType).find(prim_id);
            auto primPtr = _clusterPools.primitivePool.at(p_id);

            for (int i = 0; i < 3; i++)
            {
                c_ptr->edges.insert(primPtr->edges[i]);
            }
        }
    }

    // change the edge to be in the next cluster type
    // by chaging the key of the map.
    for (auto& [e_id, e_ptr] : _clusterPools.edgePool)
    {
        auto keyNode = e_ptr->prims.extract(_prevClusterType);
        keyNode.key() = _newClusterType;
        e_ptr->prims.insert(std::move(keyNode));
    }

    // changing the keys
    {
        auto keyNode = _clusterPools.edgeMap.extract(_prevClusterType);
        keyNode.key() = _newClusterType;
        _clusterPools.edgeMap.insert(std::move(keyNode));
    }

    {
        auto keyNode = _clusterPools.primitiveMap.extract(_prevClusterType);
        keyNode.key() = _newClusterType;
        _clusterPools.primitiveMap.insert(std::move(keyNode));
    }

    
    {
        auto keyNode = _clusterPools.vertexToPrimMap.extract(_prevClusterType);
        keyNode.key() = _newClusterType;
        _clusterPools.vertexToPrimMap.insert(std::move(keyNode));
    }

    _clusterPools.lastcreatedClusterID = highestCreatedID;

    for (auto& [c_id, c_ptr] : _clusterPools.clusters)
    {
        _clusterPools.clusterTypeMap[_newClusterType].insert(c_id);
        for (auto& p_id : c_ptr->primitives)
        {
            _clusterPools.clusterTypePrimitivePool[_newClusterType][p_id] = c_id;
        }
    }

}

void SpatialPartitioner::subdivideCluster(unsigned int aabbCnt, engine::cluster::Cluster* cluster, unsigned int DEPTH_COUNTER)
{
    std::vector<engine::mesh::AABB> aabbs = subdivideAABBUniform(cluster->aabb, aabbCnt);

    std::unordered_map<engineID_t, std::unordered_set<engineID_t>> localPrimitiveToClusterTrackingMap;
    localPrimitiveToClusterTrackingMap.reserve(cluster->primitives.size());

    // building a map to access the each 
    std::unordered_map<engineID_t, engine::cluster::Cluster*> localClusterMap;
    localClusterMap.reserve(aabbCnt);

    engineID_t lastCreatedLocalClusterId = 0;
    // creating a new arena cluster and assigning aabb to a Cluster

    for (int i = 0; i < aabbCnt; i++)
	{
		// this creates a new cluster and push it into the arena list.
		engine::cluster::Cluster* newCluster = createNewClusterArena(i, aabbs[i]);

		intersectionValidation_V2(newCluster, &cluster->primitives, localPrimitiveToClusterTrackingMap);
		// adding early filtering
		// filter out empty cluster
		if (newCluster->primitives.empty())
		{
			// if empty, remove the cluster of the _clusterArena
			// this is located at the end of the list 
			// since it just got created above.
			// erasing !
			_clusterArena.erase(std::prev(_clusterArena.end()));
		}
		else
		{
			newCluster->type = _newClusterType;
			localClusterMap[i] = newCluster;
			lastCreatedLocalClusterId = i;
		}
	}

    if (localPrimitiveToClusterTrackingMap.size() != cluster->primitives.size())
        std::abort();

    // duplication resolution
   // Pritimives could be assinged into mutliple clusters
   // so the cluster that got the lowest amount of primitive cout will be the one that 
   // the primitive will be assign to.
   // and also update the global tracking map
    for (auto& [pid, childSet] : localPrimitiveToClusterTrackingMap)
    {
        if (childSet.size() > 1)
        {
            int smallestCnt = INT_MAX;
            engineID_t smallCluster = 0;

            // find the smallest cluster
            for (auto& c : childSet)
            {
                if (localClusterMap.at(c)->primitives.size() < smallestCnt)
                {
                    smallestCnt = localClusterMap.at(c)->primitives.size();
                    smallCluster = c;
                }
            }

            // removes primitives primitives from other cluster
            // childSet tell us to which clusters the primitive got included into
            // so the means we can use these as an entry to the individual cluster to erase the primitive
            // but ignoring the smallest cluster.
            for (auto& c : childSet)
            {
                if (c != smallCluster)
                    localClusterMap.at(c)->primitives.erase(pid);

                // since we are removing primitive from
                // the cluster can become empty 
                // so if empty the cluster wil be removed
                if (localClusterMap.at(c)->primitives.empty())
                    localClusterMap.erase(c);
            }

            // reset the primitive to point to this only chosen small cluster
            childSet = { smallCluster };

        }
        _primitiveToClusterMapGlobal[pid] = { localClusterMap.at((*childSet.begin()))};
    }

    // subcluster connectivity validation
    // frist will check if each primitives of a subclusters are connected to each other. USES BFS
    for (auto locaClusterMapItr = localClusterMap.begin(); locaClusterMapItr != localClusterMap.end();)
    {
        // groups of found cluster
        std::vector<std::unique_ptr<std::vector<engineID_t>>> foundClusters;

        connectivityValidation_V2(&(*locaClusterMapItr).second->primitives, foundClusters);

        // if greater then 1. Current cluster connectivity is splitted!
        if (foundClusters.size() > 1)
        {
            // validating each group of cluster
            // rules :
            // - if cluster is still large keep it for further subdivition
            // - if cluster is with in threshold push it into the _leafcluster
            // - if cluster is too small it will be merged into the a optimal neighboring cluster 
            for (std::unique_ptr<std::vector<engineID_t>>& foundCluster : foundClusters)
            {

                int sizeOfFoundCluster = foundCluster->size();
                // first block - if cluster is still large keep it for further subdivition
                if (sizeOfFoundCluster > _MIN_CLUSTER_SIZE_THRESHOLD)
                {
                    // create a new Cluster Arena 
                    engineID_t newID = ++lastCreatedLocalClusterId;
                    engine::cluster::Cluster* newSubcluster = createNewClusterArena(newID);

                    // 2.1 move primitives
                    for (engineID_t& p : *foundCluster)
                    {
                        newSubcluster->primitives.insert(p);                 // add
                        (*locaClusterMapItr).second->primitives.erase(p);    // remove from parent
                        _primitiveToClusterMapGlobal[p] = { newSubcluster };
                    }

                    recomputeClusterAABB(newSubcluster);

                    // register in localClusterMap so later passes see it
                    localClusterMap[newID] = newSubcluster;
                }
                // secound - if cluster is with in threshold push it into the _leafcluster
                else if (sizeOfFoundCluster >= _LEAST_CLUSTER_SIZE)
                {
                    // creates a leaf cluster
                    engine::cluster::Cluster* leaf = createNewClusterArena(0);

                    // adding primitives to the leaf cluster
                    for (engineID_t& p : *foundCluster)
                    {
                        leaf->primitives.insert(p);
                        (*locaClusterMapItr).second->primitives.erase(p);
                        _primitiveToClusterMapGlobal[p] = { leaf };
                    }
                    leaf->id = _leafClusterID++;
                    leaf->type = _newClusterType;
                    recomputeClusterAABB(leaf);

                    _leafClusters[leaf->id] = leaf;  // final leaf container
                }
                // third block - if cluster is too small it will be merged into the a optimal neighboring cluster
                else
                {
                    engine::cluster::Cluster* best = pickBestNeighbour(locaClusterMapItr->second, *foundCluster, _primitiveToClusterMapGlobal);
                    if (best == nullptr)
                        throw std::runtime_error("WARNING::NO ALTERNATIVE CLUSTER FOUND!\n");

                    for (engineID_t& p : *foundCluster)
                    {
                        best->primitives.insert(p);
                        (*locaClusterMapItr).second->primitives.erase(p);
                        _primitiveToClusterMapGlobal[p] = { best };
                    }
                    if (best != cluster) recomputeClusterAABB(best);
                }

            }
            //  --- end foundClusters loop ---  

           // add debugging line
            if (!(*locaClusterMapItr).second->primitives.empty())
                std::cout << "WARING :: THIS MUST BE EMPTY!\n";

            // since the primitives of splitted cluster has been included into a different clusters
            // the current cluster MUST be empty!
            if ((*locaClusterMapItr).second->primitives.empty())
            {
                locaClusterMapItr = localClusterMap.erase(locaClusterMapItr);
            }

        }
        // enters here when the local cluster is fully connected
        else
        {
            const int clusterSize = static_cast<int>((*locaClusterMapItr).second->primitives.size());

            // - if local cluser is with the threshold the it will be added the leaf cluster list
            if (clusterSize <= _MIN_CLUSTER_SIZE_THRESHOLD && clusterSize >= _LEAST_CLUSTER_SIZE)
            {
                // set new ID
                (*locaClusterMapItr).second->id = _leafClusterID++;
                // push the raw cluster pointer into the _cluster list
                _leafClusters[(*locaClusterMapItr).second->id] = (*locaClusterMapItr).second;

                // since the cluster has saved as leaf cluster means it wont be no further suddivition for this cluster
                // hince removing it from the local cluster
                locaClusterMapItr = localClusterMap.erase(locaClusterMapItr);
            }
            // enters here when the cluster is tiny
            else if (clusterSize < _LEAST_CLUSTER_SIZE)
            {
                engine::cluster::Cluster* best = pickBestNeighbour(locaClusterMapItr->second, locaClusterMapItr->second->primitives, _primitiveToClusterMapGlobal);

                // added for debugging only
                if (best == nullptr)
                    throw std::runtime_error( " WARNING :: NO ALTERNATIVE CLUSTER FOUND!\n");

                for (auto& pid : (*locaClusterMapItr).second->primitives)
                {
                    best->primitives.insert(pid);
                    _primitiveToClusterMapGlobal[pid] = { best };
                }

                if (best != cluster) recomputeClusterAABB(best);

                // since the primitives has been moved into another cluster 
                // this local cluster will become empty and it can be erased
                locaClusterMapItr = localClusterMap.erase(locaClusterMapItr);
            }
            // enters here when local clutser is still large enough
            else
            {
                ++locaClusterMapItr;
            }

        }
    }

    for (auto& [cid, child] : localClusterMap)
    {

        // check depth
        if (DEPTH_COUNTER >= _MAX_SUBDIVISION_DEPTH)
        {  
            // if true; push all the remaining local cluster into the leaf
            child->id = _leafClusterID++;
            _leafClusters[child->id] = child;
        }
        // else calls the subdivition function again
        else
        {
            subdivideCluster(aabbCnt,child, DEPTH_COUNTER + 1);
        }
    }
}

engine::cluster::Cluster* SpatialPartitioner::pickBestNeighbour(engine::cluster::Cluster* current, 
    const std::vector<engineID_t>& part, 
    const std::unordered_map<engineID_t,std::unordered_set<engine::cluster::Cluster*>>& p2c)
{
    std::unordered_map<engine::cluster::Cluster*, int> contactCnt;

    for (engineID_t prim : part)
    {
        for (int e = 0; e < 3; ++e)                    // each edge of the tri
        {
            for (int k = 0; k < 2; ++k)                // the two opposite prims
            {
                auto p_id =  *_clusterPools.primitiveMap.at(_prevClusterType).find(prim);
                auto primPtr = _clusterPools.primitivePool.at(p_id);

                auto e_id = *_clusterPools.edgeMap.at(_prevClusterType).find(primPtr->edges[e]);
                auto edgePtr = _clusterPools.edgePool.at(e_id);
                engineID_t neighPrim = edgePtr->prims.at(_prevClusterType)[k];

                if (!neighPrim || neighPrim == prim) continue;

                auto it = p2c.find(neighPrim);
                if (it == p2c.end()) continue;         // should not happen

                for (engine::cluster::Cluster* c : it->second)
                    if (c != current) ++contactCnt[c]; // skip self
            }
        }
    }

    /* choose cluster with max contact count
       tie-break: larger size (keeps tree balanced)  */
    engine::cluster::Cluster* best = nullptr;
    int                       bestCnt = -1;
    int                       bestSize = -1;

    for (auto& [c, cnt] : contactCnt)
    {
        int sz = (int)c->primitives.size();
        if (cnt > bestCnt || (cnt == bestCnt && sz > bestSize))
        {
            best = c;
            bestCnt = cnt;
            bestSize = sz;
        }
    }
    return best;          // may be nullptr -> caller must handle
}

engine::cluster::Cluster* SpatialPartitioner::pickBestNeighbour(engine::cluster::Cluster* current, const std::unordered_set<engineID_t>& part, const std::unordered_map<engineID_t, std::unordered_set<engine::cluster::Cluster*>>& p2c)
{
    std::unordered_map<engine::cluster::Cluster*, int> contactCnt;

    for (auto& prim : part)
    {
        for (int e = 0; e < 3; ++e)                    // each edge of the tri
        {
            for (int k = 0; k < 2; ++k)                // the two opposite prims
            {
                auto p_id = *_clusterPools.primitiveMap.at(_prevClusterType).find(prim);
                auto primPtr = _clusterPools.primitivePool.at(p_id);

                auto e_id = *_clusterPools.edgeMap.at(_prevClusterType).find(primPtr->edges[e]);
                auto edgePtr = _clusterPools.edgePool.at(e_id);
                engineID_t neighPrim = edgePtr->prims.at(_prevClusterType)[k];

                if (!neighPrim || neighPrim == prim) continue;

                auto it = p2c.find(neighPrim);
                if (it == p2c.end()) continue;         // should not happen

                for (engine::cluster::Cluster* c : it->second)
                    if (c != current) ++contactCnt[c]; // skip self
            }
        }
    }

    /* choose cluster with max contact count
       tie-break: larger size (keeps tree balanced)           */
    engine::cluster::Cluster* best = nullptr;
    int                       bestCnt = -1;
    int                       bestSize = -1;

    for (auto& [c, cnt] : contactCnt)
    {
        int sz = (int)c->primitives.size();
        if (cnt > bestCnt || (cnt == bestCnt && sz > bestSize))
        {
            best = c;
            bestCnt = cnt;
            bestSize = sz;
        }
    }
    return best;          // may be nullptr -> caller must handle
}

void SpatialPartitioner::recomputeClusterAABB(engine::cluster::Cluster* cluster)
{
    cluster->aabb.reset(); // min=+inf, max=-inf
    for (auto& id: cluster->primitives) {
        for (int i = 0; i < 3; ++i)
        {   
            auto prim = *_clusterPools.primitiveMap.at(_prevClusterType).find(id);
            auto primPtr = _clusterPools.primitivePool.at(prim);

            auto vert = _clusterPools.vertexPool.at(primPtr->vertices[i]);
            cluster->aabb.expand(engine::math::Vec3f{ vert->coords.xyzw() });
        }
    }
}

void SpatialPartitioner::connectivityValidation_V2(std::unordered_set<engineID_t>* primitivesMap,
    std::vector<std::unique_ptr<std::vector<engineID_t>>>& foundClusters)
{
    foundClusters.clear();

    // 1) vertex -> primitives
    std::unordered_map<engineID_t, std::vector<engineID_t>> vertexToPrims;
    vertexToPrims.reserve(primitivesMap->size() * 3);

    for (engineID_t primID : *primitivesMap)
    {
        engineID_t poolId = *_clusterPools.primitiveMap.at(_prevClusterType).find(primID);
        const auto* prim = _clusterPools.primitivePool.at(poolId);
        for (int i = 0; i < 3; ++i)
            vertexToPrims[prim->vertices[i]].push_back(prim->id);
    }

    // 2) count shared vertices per tri and per pair
    struct PairKey { engineID_t a, b; };
    struct PairHash {
        size_t operator()(const PairKey& k) const noexcept {
            uint64_t x = (uint64_t)k.a, y = (uint64_t)k.b;
            x ^= y + 0x9e3779b97f4a7c15ULL + (x << 6) + (x >> 2);
            return (size_t)x;
        }
    };
    struct PairEq { bool operator()(const PairKey& l, const PairKey& r) const noexcept { return l.a == r.a && l.b == r.b; } };

    std::unordered_map<PairKey, uint8_t, PairHash, PairEq> pairSharedCount; // how many vertices a pair shares
    pairSharedCount.reserve(primitivesMap->size() * 3);

    std::unordered_map<engineID_t, uint8_t> triSharedVertexCount;          // 0..3
    triSharedVertexCount.reserve(primitivesMap->size());
    for (engineID_t t : *primitivesMap) triSharedVertexCount[t] = 0;

    for (auto& [v, list] : vertexToPrims)
    {
        if (list.size() <= 1) continue;

        // mark this vertex as shared for all triangles using it
        for (engineID_t t : list) ++triSharedVertexCount[t];

        // bump pair counters for this vertex
        for (size_t i = 0; i + 1 < list.size(); ++i)
            for (size_t j = i + 1; j < list.size(); ++j)
            {
                engineID_t a = list[i], b = list[j];
                if (a > b) std::swap(a, b);
                ++pairSharedCount[{a, b}];
            }
    }

    // 3) Build adjacency using only pairs that share an EDGE (>=2 vertices)
    std::unordered_map<engineID_t, std::vector<engineID_t>> adj;
    adj.reserve(primitivesMap->size());
    for (engineID_t id : *primitivesMap) adj[id]; // ensure every tri appears even if isolated

    for (auto& [pk, cnt] : pairSharedCount)
    {
        if (cnt >= 2) {
            adj[pk.a].push_back(pk.b);
            adj[pk.b].push_back(pk.a);
        }
    }

    // 4) DFS over edge-adjacent graph. Hinges (share exactly one vertex) will be singletons.
    std::unordered_set<engineID_t> visited;
    visited.reserve(primitivesMap->size());

    for (auto& [start, _] : adj)
    {
        if (visited.count(start)) continue;

        auto component = std::make_unique<std::vector<engineID_t>>();
        std::stack<engineID_t> st;
        st.push(start);

        while (!st.empty())
        {
            engineID_t u = st.top(); st.pop();
            if (!visited.insert(u).second) continue;
            component->push_back(u);
            for (engineID_t v : adj[u])
                if (!visited.count(v)) st.push(v);
        }

        // Push every component. If it’s a hinge, size()==1 and triSharedVertexCount[*] == 1.
        if (!component->empty())
            foundClusters.push_back(std::move(component));
    }


}

void SpatialPartitioner::subdivideAABB(unsigned int& subdivitionCount,
    engine::mesh::AABB& aabb, 
    std::unordered_map<engineID_t, engine::cluster::Cluster*>& clusters)
{
    std::vector<engine::mesh::AABB> aabbs = subdivideAABBUniform(aabb, subdivitionCount);

    // assigning each cluster with their own AABB
    for (int i = 0; i < aabbs.size(); i++)
    {
        engine::cluster::Cluster* cluster = new engine::cluster::Cluster();
        cluster->aabb = aabbs[i];
        // also asigning the cluster whi an ID.
        // this only temporary so the SpatialParitioner can differentiate between clusters
        // and it will reassign back to 0 when to partitioning is done.
        cluster->id = i;
        clusters[i] = cluster;
    }
}

// Generates N AABBs by subdividing the input AABB uniformly
std::vector<engine::mesh::AABB> SpatialPartitioner::subdivideAABBUniform(const engine::mesh::AABB& aabb,unsigned int numSubdivisions)
{

    using namespace engine::math;
    using namespace engine::mesh;

    // ============ AABB Layout
    /*
        Front (Zmin -> Zcenter)
        +-----+-----+
        |  0  |  1  |  <- top (Ymax)
        +-----+-----+
        |  3  |  2  |  <- bottom (Ymin)
        +-----+-----+

        Back (Zcenter -> Zmax)
        +-----+-----+
        |  4  |  5  |  <- top (Ymax)
        +-----+-----+
        |  7  |  6  |  <- bottom (Ymin)
        +-----+-----+
*/

    std::vector<AABB> results;
    Vec3f min = aabb.min;
    Vec3f max = aabb.max;

    Vec3f size = max - min;
    Vec3f center = min + size * 0.5f;

    // Determine subdivision count along each axis
    /*

        2 = Splitts along Z axis (Fron/Back)
        4 = Splitts along X - Y axis
        8 = Splitts along X - Y - z axis

    */
    int xCount = 1, yCount = 1, zCount = 1;
    if (numSubdivisions == 2) zCount = 2;             // Split along Z (Front/Back)
    else if (numSubdivisions == 4) { xCount = 2; zCount = 2; } // Split X and Z
    else if (numSubdivisions == 8) { xCount = 2; yCount = 2; zCount = 2; } // Octree style
    else throw std::invalid_argument("Unsupported subdivision count. Only 2, 4, or 8 are supported.");

    Vec3f step = Vec3f(size.x / xCount, size.y / yCount, size.z / zCount);

    for (int xi = 0; xi < xCount; ++xi) {
        for (int yi = 0; yi < yCount; ++yi) {
            for (int zi = 0; zi < zCount; ++zi) {
                Vec3f cellMin = min + Vec3f(xi * step.x, yi * step.y, zi * step.z);
                Vec3f cellMax = cellMin + step;
                results.push_back({ cellMin, cellMax });
            }
        }
    }

    return results;
}

engine::cluster::Cluster* SpatialPartitioner::createNewClusterArena(engineID_t id, engine::mesh::AABB aabb)
{
    auto cluster = std::make_unique<engine::cluster::Cluster>();
    cluster->id = static_cast<int>(id);
    cluster->aabb = aabb;
    cluster->type = _newClusterType;
    _clusterArena.push_back(std::move(cluster));
    return _clusterArena.back().get();
}

engine::cluster::Cluster* SpatialPartitioner::createNewClusterArena(engineID_t id)
{
    auto cluster = std::make_unique<engine::cluster::Cluster>();
    cluster->id = static_cast<int>(id);
    cluster->type = _newClusterType;
    _clusterArena.push_back(std::move(cluster));
    return _clusterArena.back().get();
}

void SpatialPartitioner::intersectionValidation_V2(engine::cluster::Cluster* cluster,
    std::unordered_set<engineID_t>* primitives,
    std::unordered_map<engineID_t, std::unordered_set<engineID_t>>& primitiveToClusterTrackMap)
{
    using namespace engine::math;

    const Vec3f c = cluster->aabb.getCenter();
    const Vec3f h = cluster->aabb.getHalfExtent();
    const Vec3f bMin = c - h;
    const Vec3f bMax = c + h;

    engineID_t batchPrim[8];
    uint32_t   batchIdx[8];
    int        fill = 0;

    auto flushBatch = [&](int count)
    {
            if (count == 0) return;

            uint32_t mask = triAABBOverlapMask8_ids(_triSoA, batchIdx, count, bMin, bMax);

            for (int lane = 0; lane < count; ++lane) {
                if ((mask & (1u << lane)) == 0) continue; // failed broad-phase

                engineID_t primID = batchPrim[lane];
                const auto idInPool = *_clusterPools.primitiveMap.at(_prevClusterType).find(primID);
                const auto primPtr = _clusterPools.primitivePool.at(idInPool);

                if (triBoxOverlap(primPtr, cluster->aabb)) {
                    cluster->primitives.insert(primID);
                    primitiveToClusterTrackMap[primID].insert(cluster->id);
                }
            }
     };

    for (auto it = primitives->begin(); it != primitives->end(); ++it) 
    {
        const engineID_t primID = *it;
        const auto f = _primIdToIdx.find(primID);
        if (f == _primIdToIdx.end()) continue; // should not happen if SoA is complete

        batchPrim[fill] = primID;
        batchIdx[fill] = f->second;
        ++fill;

        if (fill == 8) { flushBatch(fill); fill = 0; }
    }

    flushBatch(fill);
}

 uint32_t SpatialPartitioner::triAABBOverlapMask8_ids(
    const TriAABBSoA& S,
    const uint32_t idx[8], int count,
    const engine::math::Vec3f& bMin,
    const engine::math::Vec3f& bMax)
{
#ifndef __AVX2__
    uint32_t m = 0;
    for (int k = 0; k < count; ++k) {
        const uint32_t i = idx[k];
        bool overlap =
            (S.maxX[i] >= bMin.x) && (S.minX[i] <= bMax.x) &&
            (S.maxY[i] >= bMin.y) && (S.minY[i] <= bMax.y) &&
            (S.maxZ[i] >= bMin.z) && (S.minZ[i] <= bMax.z);
        m |= (uint32_t(overlap) << k);
    }
    return m;
#else
    float minX[8] = {}, minY[8] = {}, minZ[8] = {}, maxX[8] = {}, maxY[8] = {}, maxZ[8] = {};
    for (int k = 0; k < count; ++k) {
        const uint32_t i = idx[k];
        minX[k] = S.minX[i]; minY[k] = S.minY[i]; minZ[k] = S.minZ[i];
        maxX[k] = S.maxX[i]; maxY[k] = S.maxY[i]; maxZ[k] = S.maxZ[i];
    }

    __m256 triMinX = _mm256_loadu_ps(minX);
    __m256 triMinY = _mm256_loadu_ps(minY);
    __m256 triMinZ = _mm256_loadu_ps(minZ);
    __m256 triMaxX = _mm256_loadu_ps(maxX);
    __m256 triMaxY = _mm256_loadu_ps(maxY);
    __m256 triMaxZ = _mm256_loadu_ps(maxZ);

    __m256 BminX = _mm256_set1_ps(bMin.x);
    __m256 BminY = _mm256_set1_ps(bMin.y);
    __m256 BminZ = _mm256_set1_ps(bMin.z);
    __m256 BmaxX = _mm256_set1_ps(bMax.x);
    __m256 BmaxY = _mm256_set1_ps(bMax.y);
    __m256 BmaxZ = _mm256_set1_ps(bMax.z);

    __m256 x = _mm256_and_ps(_mm256_cmp_ps(triMaxX, BminX, _CMP_GE_OQ),
        _mm256_cmp_ps(triMinX, BmaxX, _CMP_LE_OQ));
    __m256 y = _mm256_and_ps(_mm256_cmp_ps(triMaxY, BminY, _CMP_GE_OQ),
        _mm256_cmp_ps(triMinY, BmaxY, _CMP_LE_OQ));
    __m256 z = _mm256_and_ps(_mm256_cmp_ps(triMaxZ, BminZ, _CMP_GE_OQ),
        _mm256_cmp_ps(triMinZ, BmaxZ, _CMP_LE_OQ));

    __m256 xyz = _mm256_and_ps(_mm256_and_ps(x, y), z);
    return (uint32_t)_mm256_movemask_ps(xyz); // lower 'count' bits valid
#endif
}

  void SpatialPartitioner::buildGlobalTriAABBSoA(const std::vector<engineID_t>& allPrimIds,
     const engine::cluster::ClusterPools& pools, engine::cluster::HierarchyLevel level)
 {
     const size_t N = allPrimIds.size();
     _triSoA.minX.resize(N); _triSoA.minY.resize(N); _triSoA.minZ.resize(N);
     _triSoA.maxX.resize(N); _triSoA.maxY.resize(N); _triSoA.maxZ.resize(N);
     _primIdToIdx.reserve(N);

     for (size_t i = 0; i < N; ++i) {
         engineID_t primID = allPrimIds[i];
         _primIdToIdx[primID] = (uint32_t)i;

         const auto pid = *pools.primitiveMap.at(level).find(primID);
         const auto prim = pools.primitivePool.at(pid);

         const auto& v0 = pools.vertexPool.at(prim->vertices[0])->coords;
         const auto& v1 = pools.vertexPool.at(prim->vertices[1])->coords;
         const auto& v2 = pools.vertexPool.at(prim->vertices[2])->coords;

         _triSoA.minX[i] = std::min({ v0.x, v1.x, v2.x });
         _triSoA.minY[i] = std::min({ v0.y, v1.y, v2.y });
         _triSoA.minZ[i] = std::min({ v0.z, v1.z, v2.z });
         _triSoA.maxX[i] = std::max({ v0.x, v1.x, v2.x });
         _triSoA.maxY[i] = std::max({ v0.y, v1.y, v2.y });
         _triSoA.maxZ[i] = std::max({ v0.z, v1.z, v2.z });
     }
 }

bool SpatialPartitioner::triBoxOverlap(const engine::mesh::Primitive* prim, const engine::mesh::AABB& b)
{
    using namespace engine::math;

    const Vec3f c = b.getCenter();
    const Vec3f h = b.getHalfExtent();

    /*
        triangle vertices translated so that the AABB is at the origin #
        primitive vertives has to be converted to a Vec3 first since by defaut vertices are Vec4
    */

    const Vec3f v0 = Vec3f{ _clusterPools.vertexPool.at(prim->vertices[0])->coords.x,_clusterPools.vertexPool.at(prim->vertices[0])->coords.y,_clusterPools.vertexPool.at(prim->vertices[0])->coords.z } - c;
    const Vec3f v1 = Vec3f{ _clusterPools.vertexPool.at(prim->vertices[1])->coords.x,_clusterPools.vertexPool.at(prim->vertices[1])->coords.y,_clusterPools.vertexPool.at(prim->vertices[1])->coords.z } - c;
    const Vec3f v2 = Vec3f{ _clusterPools.vertexPool.at(prim->vertices[2])->coords.x,_clusterPools.vertexPool.at(prim->vertices[2])->coords.y,_clusterPools.vertexPool.at(prim->vertices[2])->coords.z } - c;
    
    const Vec3f f0 = v1 - v0;
    const Vec3f f1 = v2 - v1;
    const Vec3f f2 = v0 - v2;

    auto proj = [](const Vec3f& v, const Vec3f& a) { return dot(v, a); };

    auto axisTest = [&](const Vec3f& a)
        {
            float p0 = proj(v0, a);
            float p1 = proj(v1, a);
            float p2 = proj(v2, a);

            float minP = std::min({ p0,p1,p2 });
            float maxP = std::max({ p0,p1,p2 });

            float r = fabs(a.x) * h.x + fabs(a.y) * h.y + fabs(a.z) * h.z;
            return (minP > r) || (maxP < -r);   // disjoint?
        };

    /* 1. 9 cross-product axes */
    if (axisTest(Vec3f{ 0,   -f0.z,  f0.y }) ||
        axisTest(Vec3f{ 0,   -f1.z,  f1.y }) ||
        axisTest(Vec3f{ 0,   -f2.z,  f2.y }) ||
        axisTest(Vec3f{ f0.z, 0,    -f0.x }) ||
        axisTest(Vec3f{ f1.z, 0,    -f1.x }) ||
        axisTest(Vec3f{ f2.z, 0,    -f2.x }) ||
        axisTest(Vec3f{ -f0.y,f0.x,  0 }) ||
        axisTest(Vec3f{ -f1.y,f1.x,  0 }) ||
        axisTest(Vec3f{ -f2.y,f2.x,  0 }))      return false;

    /* 2. AABB face normals */
    if (std::max({ v0.x,v1.x,v2.x }) < -h.x || std::min({ v0.x,v1.x,v2.x }) > h.x) return false;
    if (std::max({ v0.y,v1.y,v2.y }) < -h.y || std::min({ v0.y,v1.y,v2.y }) > h.y) return false;
    if (std::max({ v0.z,v1.z,v2.z }) < -h.z || std::min({ v0.z,v1.z,v2.z }) > h.z) return false;

    /* 3. triangle plane */
    Vec3f n = cross(f0, f1);
    float d = dot(n, v0);
    if (fabs(d) > fabs(n.x) * h.x + fabs(n.y) * h.y + fabs(n.z) * h.z) return false;

    return true; // overlap
}
