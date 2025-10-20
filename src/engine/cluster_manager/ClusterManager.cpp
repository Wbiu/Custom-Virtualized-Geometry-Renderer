#include "ClusterManager.h"



ClusterManager::ClusterManager(const engine::cluster::LODClusterMeshTree& lodMeshTree, engine::graph::LODGraph graph)
	: _lodMeshTree(lodMeshTree) , _graph(graph)
{
	_currentVisibleClusters.reserve(lodMeshTree.clusterTypeMap.at(engine::cluster::HierarchyLevel::LOD0).size());
	_newVisibleClusters.reserve(lodMeshTree.clusterTypeMap.at(engine::cluster::HierarchyLevel::LOD0).size());
	
	// setting LOD0 cluster to be the starting visible clsuters
	_currentVisibleClusters = lodMeshTree.clusterTypeMap.at(engine::cluster::HierarchyLevel::LOD0);
}

ClusterManager::~ClusterManager()
{
}

void ClusterManager::cullClusters_V2(engine::math::Mat4f& modelMat)
{
    _currentVisibleClusters.clear();

    // World OBB axes for this mesh instance
    const engine::math::Vec3f ax{ modelMat[0][0], modelMat[0][1], modelMat[0][2] };
    const engine::math::Vec3f ay{ modelMat[1][0], modelMat[1][1], modelMat[1][2] };
    const engine::math::Vec3f az{ modelMat[2][0], modelMat[2][1], modelMat[2][2] };

    std::vector<engineID_t> stack;
    stack.reserve(256);

    // Seed with coarsest nodes
    for (engineID_t rootCid : _lodMeshTree.clusterTypeMap.at(engine::cluster::HierarchyLevel::LOD2)) {
        engineID_t rootNid = _graph.clusterToNode.at(engine::cluster::HierarchyLevel::LOD2).at(rootCid);
        stack.push_back(rootNid);
    }

    auto rank = [](engine::cluster::HierarchyLevel l) { return (int)l; }; 

    while (!stack.empty()) {
        const engineID_t nid = stack.back();
        stack.pop_back();

        const auto& node = _graph.nodes.at(nid);
        const auto  cptr = _lodMeshTree.clusters.at(node.clusterId);

        // OBB frustum test
        engine::math::Vec4f centerWS4 = cptr->aabb.getCenter() * modelMat;
        const engine::math::Vec3f C{ centerWS4.x, centerWS4.y, centerWS4.z };
        const engine::math::Vec3f H{ cptr->aabb.getHalfExtent().x,
                                     cptr->aabb.getHalfExtent().y,
                                     cptr->aabb.getHalfExtent().z };

        if (!_culler.intersectsOBB_AVX(C, ax, ay, az, H, _soa))
            continue; // culled -> prune subtree

        // LOD decision
        const float dist = _culler.measureDistanceFromClusterCenter(centerWS4);
        const auto  tgt = determinLODLevel(dist);

        const bool isLeaf = node.childNodeIds.empty();
        const bool needRefine = rank(node.currentlevel) > rank(tgt); // coarser than target?

        if (needRefine && !isLeaf) {
            // refine: descend
            for (engineID_t childN : node.childNodeIds)
                stack.push_back(childN);
        }
        else {
            // good enough (or leaf): draw this node, do not descend
            _currentVisibleClusters.insert(node.clusterId);
        }
    }

}


// call/set per frame
void ClusterManager::setfrustumPlanesAndCamPos(engine::utils::array<engine::math::Vec4f, 6>& planes,engine::math::Vec4f& camPos)
{
	_culler.setFrustomPlanes(planes);
	_culler.setCamPos(camPos);
	_soa = engine::frustum::makeFrustumSoA(planes);
}

int ClusterManager::getIntersectedClusterCnt()
{
	return _newVisibleClustersDistance.size();
}
 
engine::cluster::HierarchyLevel ClusterManager::determinLODLevel(float distance)
{
	if(distance <= _LOD0_DISTANCE) return engine::cluster::HierarchyLevel::LOD0;
	if(distance <= _LOD1_DISTANCE) return engine::cluster::HierarchyLevel::LOD1;
	if(distance <= _LOD2_DISTANCE) return engine::cluster::HierarchyLevel::LOD2;
	return engine::cluster::HierarchyLevel::LOD3;
}

void ClusterManager::pickLODLevel()
{
    using L = engine::cluster::HierarchyLevel;

    // --- helpers bound to current graph ---
    auto nodeIdFor = [&](L lvl, engineID_t clusterId) -> engineID_t {
        return _graph.clusterToNode.at(lvl).at(clusterId);
        };

    auto ancestorAt = [&](engineID_t nid, L target) -> engineID_t {
        while (_graph.nodes.at(nid).currentlevel != target) {
            engineID_t p = _graph.nodes.at(nid).parentNodeId;
            if (!p) break; // at top
            nid = p;
        }
        return nid;
        };

    auto hasAncestorIn = [&](engineID_t nid,
        const std::unordered_set<engineID_t>& chosenAncestors) -> bool {
            engineID_t cur = nid;
            while (cur) {
                if (chosenAncestors.count(cur)) return true;
                cur = _graph.nodes.at(cur).parentNodeId;
            }
            return false;
        };

    // DFS: collect all descendants of 'start' that are exactly at 'target' level
    auto collectDescAt = [&](engineID_t start, L target,
        std::vector<engineID_t>& out) {
            std::vector<engineID_t> stack; stack.push_back(start);
            while (!stack.empty()) {
                engineID_t nid = stack.back(); stack.pop_back();
                const auto& n = _graph.nodes.at(nid);
                if (n.currentlevel == target || n.childNodeIds.empty()) {
                    out.push_back(nid);
                }
                else {
                    for (engineID_t c : n.childNodeIds) stack.push_back(c);
                }
            }
        };

    // Build transitions (current level -> desired level by distance)
    struct LODTransition { L currentLevel, targetLevel; };
    std::unordered_map<engineID_t, LODTransition> trans; // by clusterId
    trans.reserve(_newVisibleClustersDistance.size());
    for (auto& [cid, dist] : _newVisibleClustersDistance) {
        L cur = _lodMeshTree.clusters.at(cid)->type;
        L tgt = determinLODLevel(dist);
        trans[cid] = { cur, tgt };
    }

    std::unordered_set<engineID_t> chosenAncestors; // nodeIds of coarse picks
    std::unordered_set<engineID_t> drawClusters;    // clusterIds to draw
    drawClusters.reserve(trans.size());

    // 1) Coarsen first (up: LOD0->LOD1->LOD2). This prevents fine nodes under a chosen parent.
    for (auto& [cid, t] : trans) {
        if (!isCoarser(t.targetLevel, t.currentLevel)) continue; // not coarsening
        engineID_t curN = nodeIdFor(t.currentLevel, cid);
        engineID_t ancN = ancestorAt(curN, t.targetLevel);
        if (!hasAncestorIn(ancN, chosenAncestors)) {
            chosenAncestors.insert(ancN);
            drawClusters.insert(_graph.nodes.at(ancN).clusterId);
        }
    }

    // 2) Keep same level (only if not covered by a chosen ancestor)
    for (auto& [cid, t] : trans) {
        if (t.targetLevel != t.currentLevel) continue;
        engineID_t nid = nodeIdFor(t.currentLevel, cid);
        if (!hasAncestorIn(nid, chosenAncestors))
            drawClusters.insert(cid);
    }

    // 3) Refine (down: LOD2->LOD1->LOD0). Expand to children at target level unless covered.
    std::vector<engineID_t> bucket;
    for (auto& [cid, t] : trans) {
        if (!isFiner(t.targetLevel, t.currentLevel)) continue; // not refining
        engineID_t curN = nodeIdFor(t.currentLevel, cid);
        if (hasAncestorIn(curN, chosenAncestors)) continue;     // already covered by a coarse pick
        bucket.clear();
        collectDescAt(curN, t.targetLevel, bucket);
        for (engineID_t n : bucket) {
            if (!hasAncestorIn(n, chosenAncestors))
                drawClusters.insert(_graph.nodes.at(n).clusterId);
        }
    }

    // Commit selection
    _currentVisibleClusters.swap(drawClusters);
}
