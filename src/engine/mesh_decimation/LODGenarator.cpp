#include "LODGenarator.h"

LODGenarator::LODGenarator(engine::cluster::RootClusterModel& rootClustermodel)
	: _rootClustermodel(rootClustermodel)
{
}

LODGenarator::~LODGenarator()
{
}

bool LODGenarator::linkCheck(engine::lod::EdgeToCollapseCanidate canidate, engine::cluster::LODClusterMeshTree& LODClusterMeshTree, engine::cluster::HierarchyLevel& level)
{
	auto canidate_ptr = LODClusterMeshTree.edgePool.at(canidate.edge);
	auto& canidateVextex_1 = LODClusterMeshTree.vertexPool.at(canidate_ptr->vertices[0]);
	auto& canidateVextex_2 = LODClusterMeshTree.vertexPool.at(canidate_ptr->vertices[1]);
	
	auto& prims_can_v1 = LODClusterMeshTree.vertexToPrimMap.at(level).at(canidate_ptr->vertices[0]);
	auto& prims_can_v2 = LODClusterMeshTree.vertexToPrimMap.at(level).at(canidate_ptr->vertices[1]);

	std::unordered_set<engineID_t> outerEdges;

	for(auto& p_id : prims_can_v1)
	{
		auto p_ptr = LODClusterMeshTree.primitivePool.at(p_id);

		for(int i = 0; i < 3; i++)
		{
			auto e_ptr = LODClusterMeshTree.edgePool.at(p_ptr->edges[i]);
			
			// check if outer edge of the patch
			if ((canidateVextex_1->id != e_ptr->vertices[0] && canidateVextex_2->id != e_ptr->vertices[1])
				&& (canidateVextex_1->id != e_ptr->vertices[1] && canidateVextex_2->id != e_ptr->vertices[0])) // also check in reverse
			{
				outerEdges.insert(e_ptr->id);
			}
		}
	}


	for (auto& p_id : prims_can_v2)
	{
		auto p_ptr = LODClusterMeshTree.primitivePool.at(p_id);

		for (int i = 0; i < 3; i++)
		{
			auto e_ptr = LODClusterMeshTree.edgePool.at(p_ptr->edges[i]);

			if (outerEdges.count(e_ptr->id))
				return true;
		}
	}


	return false;
}

void LODGenarator::generateLODs(engine::cluster::LODClusterMeshTree& LODClusterMeshTree)
{
	engine::cluster::dataClusterPoolTransfer(LODClusterMeshTree, _rootClustermodel);
	engine::cluster::HierarchyLevel loweredLevels[] = { engine::cluster::HierarchyLevel::LOD1,engine::cluster::HierarchyLevel::LOD2};
	engine::cluster::HierarchyLevel levels[] = {engine::cluster::HierarchyLevel::LOD0, engine::cluster::HierarchyLevel::LOD1,engine::cluster::HierarchyLevel::LOD2};

	for (int level = 0; level < 2; level++)
	{
		aggregateClusters_V2(LODClusterMeshTree, levels[level], loweredLevels[level]);
		auto& clusterIDs = LODClusterMeshTree.clusterTypeMap.at(loweredLevels[level]);

		// begin decimating
		for (auto& c_id : clusterIDs)
		{
			auto clusterPtr = LODClusterMeshTree.clusters.at(c_id);
			std::unordered_map<engineID_t, engine::math::Mat4f> Q_mx = computeQEM_Quadrics_V2(clusterPtr, LODClusterMeshTree, loweredLevels[level]);

			auto costLess = [](const engine::lod::EdgeToCollapseCanidate& a, const engine::lod::EdgeToCollapseCanidate& b) { return a.error > b.error; };

			std::priority_queue<
				engine::lod::EdgeToCollapseCanidate,
				std::vector<engine::lod::EdgeToCollapseCanidate>,
				decltype(costLess)> MinHeap;

			const int targetCount = clusterPtr->primitives.size() * (1.0 - (_REDUCTION_RATE_PERCENTAGE / 100.0));

			for (auto& e_id : clusterPtr->innerEdge)
			{
				engine::lod::EdgeToCollapseCanidate canidate = computeQEM_EdgeCost_V2(Q_mx, LODClusterMeshTree.edgePool.at(e_id), clusterPtr, LODClusterMeshTree, loweredLevels[level]);
				if (canidate.error != FLT_MAX)                       // valid candidate?
					MinHeap.push(std::move(canidate));
			}



			unsigned int startingSize = clusterPtr->primitives.size();
			engineID_t currEdgeID = 0;

			while (clusterPtr->primitives.size() > targetCount && !MinHeap.empty())
			{
				engine::lod::EdgeToCollapseCanidate best = MinHeap.top();
				MinHeap.pop();
				currEdgeID = best.edge;

				// check the edge is still valid at this level
				if (LODClusterMeshTree.edgePool.count(best.edge) == 0)
				{
					continue;
				}

				auto& e_ptr = LODClusterMeshTree.edgePool.at(best.edge);
				if (e_ptr->prims.count(loweredLevels[level]) == 0)
				{
					continue; // skips invalid edges
				}

				if (Q_mx.count(e_ptr->vertices[0]) == 0 || Q_mx.count(e_ptr->vertices[1]) == 0)
				{
					continue;
				}


				if (linkCheck(best, LODClusterMeshTree, loweredLevels[level]))
				{
					continue;
				}

				// skips if the vertices are boundarary
				if (clusterPtr->boundaryVertices.count(e_ptr->vertices[0]) && 
					clusterPtr->boundaryVertices.count(e_ptr->vertices[1]))
				{
					continue;
				}

				CollapseResult collapseResult = collapseEdge_v2(best, clusterPtr, LODClusterMeshTree, loweredLevels[level], levels[level]);

				// skips if the edge hits the topology capacity 
				if (collapseResult.w == 0 && collapseResult.u == 0 && collapseResult.v == 0)
				{
					continue;
				}

				// w,u,v from CollapseResult
				if (collapseResult.w != collapseResult.u && collapseResult.w != collapseResult.v) {
					// new vertex
					Q_mx[collapseResult.w] = Q_mx.at(collapseResult.u) + Q_mx.at(collapseResult.v);
					Q_mx.erase(collapseResult.u);
					Q_mx.erase(collapseResult.v);
				}
				else if (collapseResult.w == collapseResult.u) {
					Q_mx.at(collapseResult.u) = Q_mx.at(collapseResult.u) + Q_mx.at(collapseResult.v);
					Q_mx.erase(collapseResult.v);
				}
				else { // w == v
					Q_mx.at(collapseResult.v) = Q_mx.at(collapseResult.v) + Q_mx.at(collapseResult.u);
					Q_mx.erase(collapseResult.u);
				}

				//Dedupe neighbors
				std::unordered_set<engineID_t> neighbors;
				for (engineID_t e_id : collapseResult.incidentEdges) {
					auto& e = *LODClusterMeshTree.edgePool.at(e_id);
					engineID_t x = (e.vertices[0] == collapseResult.w) ? e.vertices[1] : e.vertices[0];
					neighbors.insert(x);
				}

				for (engineID_t x : neighbors) {
					Q_mx[x] = computeQ(x, LODClusterMeshTree, loweredLevels[level]);
				}

				for (engineID_t e_id : collapseResult.incidentEdges) {
					auto* e = LODClusterMeshTree.edgePool.at(e_id);

					// skip if not inner for this level
					auto pit = e->prims.find(loweredLevels[level]);
					if (pit == e->prims.end()) continue;      // not present at this level
					const auto& ps = pit->second;
					if (ps.size() < 2 || !clusterPtr->primitives.count(ps[0]) || !clusterPtr->primitives.count(ps[1]))
						continue;

					auto cand = computeQEM_EdgeCost_V2(Q_mx, e, clusterPtr, LODClusterMeshTree, loweredLevels[level]);
					if (cand.error != FLT_MAX) MinHeap.push(std::move(cand));
				}
			}
		}
	}
}

// Helper function to calculate a primitive's normal
engine::math::Vec4f LODGenarator::calculate_primitive_normal(const engine::math::Vec4f& p_pos_0,
	const engine::math::Vec4f& p_pos_1, 
	const engine::math::Vec4f& p_pos_2 )
{
	engine::math::Vec4f e1 = p_pos_1 - p_pos_0;
	engine::math::Vec4f e2 = p_pos_2 - p_pos_0;

	engine::math::Vec4f normal = e1.cross(e2);
	normal.w = 0.0f;  // make intent explicit
	// Handle degenerate cases before normalizing
	float len = engine::math::length(normal);
	if (len < std::numeric_limits<float>::epsilon()) {
		return engine::math::Vec4f{ 0.0f, 0.0f, 0.0f ,0.0f }; // Degenerate triangle, return zero normal
	}
	return engine::math::normalize(normal);
}

void LODGenarator::aggregateClusters_V2(engine::cluster::LODClusterMeshTree& LODClusterMeshTree,
	engine::cluster::HierarchyLevel prevType,
	engine::cluster::HierarchyLevel newType)
{

	// aggregate clusters
	std::unordered_set<engineID_t> usedClusters;
	std::unordered_set<engineID_t> smallClusters;
	std::unordered_map<engineID_t, engine::cluster::Cluster*> parents;

	auto& clusterIDs = LODClusterMeshTree.clusterTypeMap.at(prevType);

	for (auto& c_id : clusterIDs)
	{
		auto clusterPtr = LODClusterMeshTree.clusters.at(c_id);

		if (usedClusters.count(c_id)) continue;

		engine::cluster::Cluster* parent = new engine::cluster::Cluster();
		parent->id = ++LODClusterMeshTree.lastcreatedClusterID;
		parents[parent->id] = parent;
		parent->children.insert(c_id);

		usedClusters.insert(c_id);
		clusterPtr->parent = parent->id;

		for (auto& p_id : clusterPtr->primitives)
		{
			// adding prims to the parent cluster
			parent->primitives.insert(p_id);

			LODClusterMeshTree.primitiveMap[newType].insert(p_id);
			for (int i = 0; i < 3; i++)
			{
				// adding edges to the parent cluster
				parent->edges.insert(LODClusterMeshTree.primitivePool.at(p_id)->edges[i]);
				LODClusterMeshTree.edgeMap[newType].insert(LODClusterMeshTree.primitivePool.at(p_id)->edges[i]);
			}
		}

		/* -- breadth-first fill ------------------------------------------------ */
		std::queue<engine::cluster::Cluster*> q;
		q.push(clusterPtr);                           // ring-0

		while (!q.empty())
		{
			engine::cluster::Cluster* cur = q.front();
			q.pop();

			for (auto& nb_ID : cur->neighborClusters)
			{
				if (usedClusters.count(nb_ID))    continue;      // seen already

				// overflow check
				if (parent->primitives.size() +
					LODClusterMeshTree.clusters.at(nb_ID)->primitives.size() >
					_MAX_PRIM_CNT_PER_CLUSTER) continue;


				/* accept neighbour ------------------------------------------- */
				parent->children.insert(nb_ID);
				LODClusterMeshTree.clusters.at(nb_ID)->parent = parent->id;
				usedClusters.insert(nb_ID);

				for (auto& nb_p_id : LODClusterMeshTree.clusters.at(nb_ID)->primitives)
				{
					// adding prims to the parent cluster
					parent->primitives.insert(nb_p_id);
					LODClusterMeshTree.primitiveMap[newType].insert(nb_p_id);
					for (int i = 0; i < 3; i++)
					{
						// adding edges to the parent cluster
						parent->edges.insert(LODClusterMeshTree.primitivePool.at(nb_p_id)->edges[i]);
						LODClusterMeshTree.edgeMap[newType].insert(LODClusterMeshTree.primitivePool.at(nb_p_id)->edges[i]);
					}
				}
				q.push(LODClusterMeshTree.clusters.at(nb_ID));                                 // explore later
			}
		}

		engine::cluster::computeClusterAABB(parent, LODClusterMeshTree, newType);

		/* set LOD */
		parent->type = newType;

		/* store the finished parent */
		parents[parent->id] = parent;
	}


	// adding these parents to the Pool
	for (auto& [c_id, c_ptr] : parents)
	{
		LODClusterMeshTree.clusters.insert({ c_id,c_ptr });
		LODClusterMeshTree.clusterTypeMap[c_ptr->type].insert(c_id);

		for (auto& p_id : c_ptr->primitives)
		{
			LODClusterMeshTree.primToCluster.at(p_id).insert(c_id);
			LODClusterMeshTree.clusterTypePrimitivePool[c_ptr->type][p_id] = c_id;

			// updating vertices to primitives map 
			// adding the new level
			for (int i = 0; i < 3; i++)
			{
				auto primPtr = LODClusterMeshTree.primitivePool.at(p_id);
				LODClusterMeshTree.vertexToPrimMap[newType][primPtr->vertices[i]].insert(p_id);
			}

		}

	}


	// adding level to the edge 
	for (auto& e_id : LODClusterMeshTree.edgeMap.at(newType))
	{
		auto& e_ptr = LODClusterMeshTree.edgePool.at(e_id);

		e_ptr->prims[newType].add(e_ptr->prims.at(prevType)[0]);
		e_ptr->prims[newType].add(e_ptr->prims.at(prevType)[1]);
	}

	engine::cluster::outlineCluster(LODClusterMeshTree, newType);

	// check tiny clusters
	for (auto& [parent_id, parent_ptr] : parents)
	{
		if (parent_ptr->primitives.size() < 500)
		{
			auto& neighbors = parent_ptr->neighborClusters;

			bool foundAlternative = false;
			engineID_t chosenCluster = 0;
			engineID_t neighborClusterSize = UINT32_MAX;

			// find the smallest cluster
			for (auto neighbor : neighbors)
			{
				if (LODClusterMeshTree.clusters.at(neighbor)->primitives.size() < neighborClusterSize)
				{
					chosenCluster = neighbor;
					foundAlternative = true;
					neighborClusterSize = LODClusterMeshTree.clusters.at(neighbor)->primitives.size();
				}
			}

			engine::cluster::Cluster* smallestClusterPtr = LODClusterMeshTree.clusters.at(chosenCluster);

			// removing the tiny parent cluster from its neighbor 
			for (auto neighbor : neighbors)
			{
				auto n_ptr = LODClusterMeshTree.clusters.at(neighbor);
				n_ptr->neighborClusters.erase(parent_id);
			}

			// re-ference the children cluster to the chosen smallest cluster
			for (auto& cd_id : parent_ptr->children)
			{
				auto& child_ptr = LODClusterMeshTree.clusters.at(cd_id);
				child_ptr->parent = smallestClusterPtr->id; // update parent

				smallestClusterPtr->children.insert(cd_id);

				// transfer data over to the chosen neighbor
				for (auto& prim_id : child_ptr->primitives)
				{
					// Inserting the prims into the chosen cluster
					smallestClusterPtr->primitives.insert(prim_id);

					// update the prim to be in the new chosen cluster
					LODClusterMeshTree.primToCluster.at(prim_id).insert(chosenCluster);
					LODClusterMeshTree.clusterTypePrimitivePool.at(newType).at(prim_id) = smallestClusterPtr->id;

					// updating the edges
					for (int i = 0; i < 3; i++)
					{
						// adding edges to the parent cluster
						smallestClusterPtr->edges.insert(LODClusterMeshTree.primitivePool.at(prim_id)->edges[i]);
					}

				}

			}

			// removing the small cluster
			LODClusterMeshTree.clusters.erase(parent_id);
			LODClusterMeshTree.clusterTypeMap[parent_ptr->type].erase(parent_id);

			engine::cluster::computeClusterAABB(smallestClusterPtr, LODClusterMeshTree, newType);
			engine::cluster::outlineCluster(LODClusterMeshTree, newType);
		}
	}
}

std::unordered_map<engineID_t, engine::math::Mat4f>
LODGenarator::computeQEM_Quadrics_V2(engine::cluster::Cluster* cluster,
	engine::cluster::LODClusterMeshTree& LODClusterMeshTree,
	engine::cluster::HierarchyLevel level)
{
	using engine::math::Mat4f;
	using engine::math::Vec4f;

	std::unordered_map<engineID_t, Mat4f> Q_mxMap;
	std::unordered_set<engineID_t> computedVertices;

	auto accumulate_Q_for_vertex = [&](engineID_t vertexID)
		{
			if (computedVertices.find(vertexID) != computedVertices.end())
				return;

#ifndef ENGINE_MATH_SIMD
			Mat4f Q_sum(0.0f);

			const auto& listOfPrim = LODClusterMeshTree.vertexToPrimMap.at(level).at(vertexID);
			for (engineID_t p_id : listOfPrim)
			{
				const auto p0 = *LODClusterMeshTree.vertexPool.at(LODClusterMeshTree.primitivePool.at(p_id)->vertices[0]);
				const auto p1 = *LODClusterMeshTree.vertexPool.at(LODClusterMeshTree.primitivePool.at(p_id)->vertices[1]);
				const auto p2 = *LODClusterMeshTree.vertexPool.at(LODClusterMeshTree.primitivePool.at(p_id)->vertices[2]);

				Vec4f n = calculate_primitive_normal(p0.coords, p1.coords, p2.coords);
				if (engine::math::length(n) < std::numeric_limits<float>::epsilon())
					continue;

				const float d = -(n.dot(p0.coords));
				const float a = n.x, b = n.y, c = n.z;

				// Outer product P * P^T, P = [a b c d]
				Mat4f K(0.0f);
				K.row[0] = { a * a, a * b, a * c, a * d };
				K.row[1] = { b * a, b * b, b * c, b * d };
				K.row[2] = { c * a, c * b, c * c, c * d };
				K.row[3] = { d * a, d * b, d * c, d * d };

				Q_sum = Q_sum + K;
			}

			Q_mxMap[vertexID] = Q_sum;

#else
			// SIMD accumulation of Q = sum_i (P_i * P_i^T)
			// Uses SSE2 baseline; if FMA is available, fused multiply - add is used for accumulation.
			__m128 sum0 = _mm_setzero_ps(); // row 0
			__m128 sum1 = _mm_setzero_ps(); // row 1
			__m128 sum2 = _mm_setzero_ps(); // row 2
			__m128 sum3 = _mm_setzero_ps(); // row 3

			const auto& listOfPrim = LODClusterMeshTree.vertexToPrimMap.at(level).at(vertexID);
			for (engineID_t p_id : listOfPrim)
			{
				const auto p0 = *LODClusterMeshTree.vertexPool.at(LODClusterMeshTree.primitivePool.at(p_id)->vertices[0]);
				const auto p1 = *LODClusterMeshTree.vertexPool.at(LODClusterMeshTree.primitivePool.at(p_id)->vertices[1]);
				const auto p2 = *LODClusterMeshTree.vertexPool.at(LODClusterMeshTree.primitivePool.at(p_id)->vertices[2]);

				Vec4f n = calculate_primitive_normal(p0.coords, p1.coords, p2.coords);
				if (engine::math::length(n) < std::numeric_limits<float>::epsilon())
					continue;

				const float d = -(n.dot(p0.coords));
				const __m128 P = _mm_setr_ps(n.x, n.y, n.z, d); // [a b c d]

				const __m128 ax = _mm_shuffle_ps(P, P, _MM_SHUFFLE(0, 0, 0, 0)); // aaaa
				const __m128 by = _mm_shuffle_ps(P, P, _MM_SHUFFLE(1, 1, 1, 1)); // bbbb
				const __m128 cz = _mm_shuffle_ps(P, P, _MM_SHUFFLE(2, 2, 2, 2)); // cccc
				const __m128 dw = _mm_shuffle_ps(P, P, _MM_SHUFFLE(3, 3, 3, 3)); // dddd

#if defined(__FMA__) ||  defined(__AVX2__)
				sum0 = _mm_fmadd_ps(ax, P, sum0); // sum0 += a * P
				sum1 = _mm_fmadd_ps(by, P, sum1); // sum1 += b * P
				sum2 = _mm_fmadd_ps(cz, P, sum2); // sum2 += c * P
				sum3 = _mm_fmadd_ps(dw, P, sum3); // sum3 += d * P
#else
				sum0 = _mm_add_ps(sum0, _mm_mul_ps(ax, P));
				sum1 = _mm_add_ps(sum1, _mm_mul_ps(by, P));
				sum2 = _mm_add_ps(sum2, _mm_mul_ps(cz, P));
				sum3 = _mm_add_ps(sum3, _mm_mul_ps(dw, P));
#endif
			}

			alignas(16) float r0[4], r1[4], r2[4], r3[4];
			_mm_storeu_ps(r0, sum0);
			_mm_storeu_ps(r1, sum1);
			_mm_storeu_ps(r2, sum2);
			_mm_storeu_ps(r3, sum3);

			Mat4f Q_sum(0.0f);
			Q_sum.row[0] = { r0[0], r0[1], r0[2], r0[3] };
			Q_sum.row[1] = { r1[0], r1[1], r1[2], r1[3] };
			Q_sum.row[2] = { r2[0], r2[1], r2[2], r2[3] };
			Q_sum.row[3] = { r3[0], r3[1], r3[2], r3[3] };

			Q_mxMap[vertexID] = Q_sum;
#endif

			computedVertices.insert(vertexID);
		};

	for (auto& e_id : cluster->innerEdge)
	{
		auto& e_ptr = LODClusterMeshTree.edgePool.at(e_id);
		accumulate_Q_for_vertex(e_ptr->vertices[0]);
		accumulate_Q_for_vertex(e_ptr->vertices[1]);
	}

	return Q_mxMap;
}

engine::lod::EdgeToCollapseCanidate LODGenarator::computeQEM_EdgeCost_V2(std::unordered_map<engineID_t, engine::math::Mat4f>& Q_mxMap,
	engine::mesh::Edge* edge,
	engine::cluster::Cluster* cluster,
	engine::cluster::LODClusterMeshTree& LODClusterMeshTree,
	engine::cluster::HierarchyLevel level)
{
	// the two incident vertices must be in Q
	auto it0 = Q_mxMap.find(edge->vertices[0]);
	auto it1 = Q_mxMap.find(edge->vertices[1]);

	// Add a function to print your matrix

	// (b) no quadric
	if (it0 == Q_mxMap.end() || it1 == Q_mxMap.end())
	{
		++g_rej.noQ;
		return { edge->id, FLT_MAX, {} };
	}

	// (c) boundarary vertices
	if (cluster->boundaryVertices.count(edge->vertices[0]) && cluster->boundaryVertices.count(edge->vertices[1]))
	{
		return { edge->id, FLT_MAX, {} };
	}
	engine::math::Mat4f Q1 = Q_mxMap.at(edge->vertices[0]);
	engine::math::Mat4f Q2 = Q_mxMap.at(edge->vertices[1]);

	engine::math::Mat4f Q_pair = Q1 + Q2; // Combined Quadric

	// --- Find Optimal v Position ---
	// Extract 3x3 A and 3x1 b from Q_pair
	engine::math::Mat3f A =
	{
		{Q_pair.row[0]._a, Q_pair.row[0]._b, Q_pair.row[0]._c},
		{Q_pair.row[1]._a, Q_pair.row[1]._b, Q_pair.row[1]._c},
		{Q_pair.row[2]._a, Q_pair.row[2]._b, Q_pair.row[2]._c}
	};

	engine::math::Vec3f b_neg = // -b from Av = -b
	{
		-Q_pair.row[0]._d,
		-Q_pair.row[1]._d,
		-Q_pair.row[2]._d
	};

	float detA = engine::math::det(A);

	engine::math::Vec4f optimal_v;
	float current_error;
	const float DET_EPSILON = 1e-8f; // For determinant check

	if (std::abs(detA) < DET_EPSILON) // Determinant is zero or too small, fall back
	{

		// Calculate error for the fallback position
		const engineID_t uID = edge->vertices[0];
		const engineID_t vID = edge->vertices[1];

		const engine::math::Vec4f Pu = LODClusterMeshTree.vertexPool.at(uID)->coords;
		const engine::math::Vec4f Pv = LODClusterMeshTree.vertexPool.at(vID)->coords;

		engine::math::Vec4f Pm = (Pu + Pv) * 0.5f;
		Pm.w = 1.0f; // be explicit

		// v^T * Q * v with v = [x y z 1]; use the same component family consistently (_a,_b,_c,_d)
		auto evalQ = [&](const engine::math::Vec4f& P) -> float {
			engine::math::Vec4f v_h{ P.x, P.y, P.z, 1.0f };
			float r0 = Q_pair.row[0]._a * v_h._a + Q_pair.row[0]._b * v_h._b + Q_pair.row[0]._c * v_h._c + Q_pair.row[0]._d * v_h._d;
			float r1 = Q_pair.row[1]._a * v_h._a + Q_pair.row[1]._b * v_h._b + Q_pair.row[1]._c * v_h._c + Q_pair.row[1]._d * v_h._d;
			float r2 = Q_pair.row[2]._a * v_h._a + Q_pair.row[2]._b * v_h._b + Q_pair.row[2]._c * v_h._c + Q_pair.row[2]._d * v_h._d;
			float r3 = Q_pair.row[3]._a * v_h._a + Q_pair.row[3]._b * v_h._b + Q_pair.row[3]._c * v_h._c + Q_pair.row[3]._d * v_h._d;
			return v_h._a * r0 + v_h._b * r1 + v_h._c * r2 + v_h._d * r3;
			};

		const float eU = evalQ(Pu);
		const float eV = evalQ(Pv);
		const float eM = evalQ(Pm);

		if (eU <= eV && eU <= eM) { optimal_v = Pu; current_error = eU; }
		else if (eV <= eM) { optimal_v = Pv; current_error = eV; }
		else { optimal_v = Pm; current_error = eM; }

		// ensure homogeneous intent
		optimal_v.w = 1.0f;
	}
	else // Determinant is good, proceed with inverse
	{
		auto inv = engine::math::inverse(A);

		// Calculate optimal_v = invA * b_neg (matrix-vector multiplication)
		optimal_v.x = inv.row[0].dot(b_neg); // Using vec3.dot for row-vector dot product
		optimal_v.y = inv.row[1].dot(b_neg);
		optimal_v.z = inv.row[2].dot(b_neg);
		optimal_v.w = 1.0f;

		// Calculate error = v_h^T * Q_pair * v_h  (SIMD-accelerated)
		engine::math::Vec4f v_h = { optimal_v.x, optimal_v.y, optimal_v.z, optimal_v.w };

#ifndef ENGINE_MATH_SIMD
		{
			const float r0 = Q_pair.row[0]._a * v_h._a + Q_pair.row[0]._b * v_h._b + Q_pair.row[0]._c * v_h._c + Q_pair.row[0]._d * v_h._d;
			const float r1 = Q_pair.row[1]._a * v_h._a + Q_pair.row[1]._b * v_h._b + Q_pair.row[1]._c * v_h._c + Q_pair.row[1]._d * v_h._d;
			const float r2 = Q_pair.row[2]._a * v_h._a + Q_pair.row[2]._b * v_h._b + Q_pair.row[2]._c * v_h._c + Q_pair.row[2]._d * v_h._d;
			const float r3 = Q_pair.row[3]._a * v_h._a + Q_pair.row[3]._b * v_h._b + Q_pair.row[3]._c * v_h._c + Q_pair.row[3]._d * v_h._d;

			current_error = v_h._a * r0 + v_h._b * r1 + v_h._c * r2 + v_h._d * r3;
		}
#else
		{
			const __m128 v = _mm_setr_ps(v_h._a, v_h._b, v_h._c, v_h._d);
			const __m128 row0 = _mm_setr_ps(Q_pair.row[0]._a, Q_pair.row[0]._b, Q_pair.row[0]._c, Q_pair.row[0]._d);
			const __m128 row1 = _mm_setr_ps(Q_pair.row[1]._a, Q_pair.row[1]._b, Q_pair.row[1]._c, Q_pair.row[1]._d);
			const __m128 row2 = _mm_setr_ps(Q_pair.row[2]._a, Q_pair.row[2]._b, Q_pair.row[2]._c, Q_pair.row[2]._d);
			const __m128 row3 = _mm_setr_ps(Q_pair.row[3]._a, Q_pair.row[3]._b, Q_pair.row[3]._c, Q_pair.row[3]._d);

#if defined(__SSE4_1__) || defined(__AVX2__)
			const __m128 d0 = _mm_dp_ps(row0, v, 0xF1); // dot4 -> X
			const __m128 d1 = _mm_dp_ps(row1, v, 0xF1);
			const __m128 d2 = _mm_dp_ps(row2, v, 0xF1);
			const __m128 d3 = _mm_dp_ps(row3, v, 0xF1);

			const __m128 rr = _mm_setr_ps(_mm_cvtss_f32(d0),
				_mm_cvtss_f32(d1),
				_mm_cvtss_f32(d2),
				_mm_cvtss_f32(d3));

			current_error = _mm_cvtss_f32(_mm_dp_ps(v, rr, 0xF1));
#else
			auto dot4 = [](__m128 a, __m128 b) -> float {
				__m128 m = _mm_mul_ps(a, b);
				__m128 shuf = _mm_shuffle_ps(m, m, _MM_SHUFFLE(2, 1, 0, 3));
				__m128 sum = _mm_add_ps(m, shuf);
				shuf = _mm_shuffle_ps(sum, sum, _MM_SHUFFLE(1, 0, 3, 2));
				sum = _mm_add_ps(sum, shuf);
				return _mm_cvtss_f32(sum);
				};

			const float r0 = dot4(row0, v);
			const float r1 = dot4(row1, v);
			const float r2 = dot4(row2, v);
			const float r3 = dot4(row3, v);

			const __m128 rr = _mm_setr_ps(r0, r1, r2, r3);
			current_error = dot4(v, rr);
#endif
		}
#endif



	} // End of optimal_v calculation

	// ---------- triangle flip / degeneracy guard -------------------------
	// Collect candidates to rewire (faces incident to u or v) and drop the two removed.
	std::unordered_set<engineID_t> toCheck;

	size_t reserveSize = LODClusterMeshTree.vertexToPrimMap.at(level).at(edge->vertices[0]).size() + LODClusterMeshTree.vertexToPrimMap.at(level).at(edge->vertices[1]).size();
	toCheck.reserve(reserveSize);

	auto addFaces = [&](engineID_t vid)
		{
			for (engineID_t t : LODClusterMeshTree.vertexToPrimMap.at(level).at(vid))
			{
				if (cluster->primitives.count(t) && (t != edge->prims.at(level)[0] && t != edge->prims.at(level)[1]))
				{
					toCheck.insert(t);
				}
			}
		};

	addFaces(edge->vertices[0]);
	addFaces(edge->vertices[1]);

	const engineID_t u = edge->vertices[0];
	const engineID_t v = edge->vertices[1];
	const float uvLen2 = engine::math::len2_xyz(LODClusterMeshTree.vertexPool.at(u)->coords,
		LODClusterMeshTree.vertexPool.at(v)->coords);

	for (engineID_t pid : toCheck) {
		auto* tri = LODClusterMeshTree.primitivePool.at(pid);

		// Original vertices (IDs + coords)
		engineID_t ids[3] = { tri->vertices[0], tri->vertices[1], tri->vertices[2] };
		engine::math::Vec4f A0 = LODClusterMeshTree.vertexPool.at(ids[0])->coords;
		engine::math::Vec4f B0 = LODClusterMeshTree.vertexPool.at(ids[1])->coords;
		engine::math::Vec4f C0 = LODClusterMeshTree.vertexPool.at(ids[2])->coords;

		// Rewired triangle (replace occurrences of u or v by optimal_v)
		engine::math::Vec4f A = (ids[0] == u || ids[0] == v) ? optimal_v : A0;
		engine::math::Vec4f B = (ids[1] == u || ids[1] == v) ? optimal_v : B0;
		engine::math::Vec4f C = (ids[2] == u || ids[2] == v) ? optimal_v : C0;

		// Degeneracy: zero/really small area after rewire
		float A2 = engine::math::area2_sq(A, B, C);                      // (2*area)^2  ~ length^4
		// Use a scale from the triangle itself (sum of edge^2), then square it to match units.
		float s = engine::math::len2_xyz(A, B) + engine::math::len2_xyz(B, C) + engine::math::len2_xyz(C, A); // length^2
		float s2 = s * s;

		if (A2 <= _kAreaRelEps * s2)                   // robust sliver/degenerate test
		{
			++g_rej.area;
			return { edge->id, FLT_MAX, {} };
		}

		// Normal flip guard
		auto n0 = engine::math::unitNormal_xyz(A0, B0, C0);
		auto n1 = engine::math::unitNormal_xyz(A, B, C);
		//float dotn = n0.x * n1.x + n0.y * n1.y + n0.z * n1.z;
		float dotn = engine::math::dot3_xyz(n0,n1);

		if (dotn < _kCosMaxFlip)
		{
			++g_rej.flip;
			return { edge->id, FLT_MAX, {} };
		}
		else
		{
		}
	}

	return {
		edge->id,
		current_error,
		optimal_v
	};
}

CollapseResult LODGenarator::collapseEdge_v2(engine::lod::EdgeToCollapseCanidate& canidate,
	engine::cluster::Cluster* clusterPtr,
	engine::cluster::LODClusterMeshTree& LODClusterMeshTree,
	engine::cluster::HierarchyLevel clusterType,
	engine::cluster::HierarchyLevel prevClusterType)
{

	CollapseResult collapseResult;
	auto& canidateEdgeID = *LODClusterMeshTree.edgeMap.at(clusterType).find(canidate.edge);
	auto& canidateEdgePtr = LODClusterMeshTree.edgePool.at(canidateEdgeID);

	auto canidateVextex_1 = LODClusterMeshTree.vertexPool.at(canidateEdgePtr->vertices[0]);
	auto canidateVextex_2 = LODClusterMeshTree.vertexPool.at(canidateEdgePtr->vertices[1]);

	bool connected_boundary_v1 = clusterPtr->boundaryVertices.count(canidateVextex_1->id) > 0;
	bool connected_boundary_v2 = clusterPtr->boundaryVertices.count(canidateVextex_2->id) > 0;

	engine::mesh::Vertex* keepV;      // the vertex that will survive
	engine::mesh::Vertex* discardV_1 = nullptr;   // the one we will remove
	engine::mesh::Vertex* discardV_2 = nullptr;   // the one we will remove; this one will be used if the optimal_v is used

	bool isVertexSpanningPass = false;

	// check if the edge is connected to a boundary vertex
	if (connected_boundary_v1) // if v1 is connected to a boudary vertex
	{
		keepV = canidateVextex_1;
		discardV_1 = canidateVextex_2;
		isVertexSpanningPass = true;
	}
	else if (connected_boundary_v2) // if v2 is connected to a boudary vertex
	{
		keepV = canidateVextex_2;
		discardV_1 = canidateVextex_1;
		isVertexSpanningPass = true;
	}
	else // not connected to a boundary vertex
	{

		float d_v0 = engine::math::length(canidate.optimal - canidateVextex_1->coords);
		float d_v1 = engine::math::length(canidate.optimal - canidateVextex_2->coords);

		float edge_len = engine::math::length(canidateVextex_1->coords - canidateVextex_2->coords);

		const float kSnapFactor = 0.25f;      // 25% of the edge length
		const float t = kSnapFactor * edge_len;


		if (d_v0 <= t) // optimal_v very close to v0
		{
			keepV = canidateVextex_1;
			discardV_1 = canidateVextex_2;
			isVertexSpanningPass = true;
		}
		else if (d_v1 <= t) // optimal_v very close to v1
		{
			keepV = canidateVextex_2;
			discardV_1 = canidateVextex_1;
			isVertexSpanningPass = true;
		}
		else
		{
			engine::mesh::Vertex* newVertex = new engine::mesh::Vertex();      // the vertex that will survive
			newVertex->coords = canidate.optimal;
			newVertex->id = LODClusterMeshTree.nextAvailableVertexID++;

			keepV = newVertex;
			discardV_1 = canidateVextex_1;
			discardV_2 = canidateVextex_2;

			// adding optimimal to the pool
			LODClusterMeshTree.vertexPool[newVertex->id] = newVertex;
			LODClusterMeshTree.vertexToPrimMap.at(clusterType)[newVertex->id] = {};
		}
	}


	collapseResult.w = keepV->id;
	collapseResult.u = canidateEdgePtr->vertices[0];
	collapseResult.v = canidateEdgePtr->vertices[1];

	// marking triangle to be removed
	std::unordered_set<engineID_t> toRemovePrims;

	size_t reserveSize = 2;

	if (discardV_1)
		reserveSize += LODClusterMeshTree.vertexToPrimMap.at(clusterType).at(discardV_1->id).size();

	if (discardV_2)
		reserveSize += LODClusterMeshTree.vertexToPrimMap.at(clusterType).at(discardV_2->id).size();

	toRemovePrims.reserve(reserveSize);

	toRemovePrims.insert(canidateEdgePtr->prims.at(clusterPtr->type)[0]);
	toRemovePrims.insert(canidateEdgePtr->prims.at(clusterPtr->type)[1]);


	// collect all tris incident to discarded vertices, minus to be removed triangle)
	std::unordered_set<engineID_t> toRewire;

	auto collect = [&](engineID_t vid) {
		for (engineID_t t : LODClusterMeshTree.vertexToPrimMap.at(clusterType).at(vid)) 
		{
			if (!toRemovePrims.count(t) && clusterPtr->primitives.count(t)) 
			{	
				toRewire.insert(t);
			}
		}
	};

	if (discardV_1) 
		collect(discardV_1->id);

	if (discardV_2) 
		collect(discardV_2->id);


	auto capacityCheck = [&]() -> bool
	{
			std::unordered_set<engine::mesh::EdgeKey, engine::mesh::EdgeHashByVertexId, engine::mesh::EdgeKeyEq>  innerEdgeKeyset;

			for (auto toRewirePrimID : toRewire)
			{
				const auto toRewirePrimPtr = LODClusterMeshTree.primitivePool.at(toRewirePrimID);

				// pre creating the edges
				engine::utils::array<engineID_t, 3> vertices;


				// getting the new coords 
				for (int i = 0; i < 3; i++)
				{
					engineID_t v = (toRewirePrimPtr->vertices[i] == canidateVextex_1->id || toRewirePrimPtr->vertices[i] == canidateVextex_2->id) ? keepV->id : toRewirePrimPtr->vertices[i];
					vertices.add(v);
				}

				// check if inner edge in this patch
				// first pair of edge [0,1]
				if ((canidateVextex_1->id == vertices[0] || canidateVextex_2->id == vertices[0])
					|| (canidateVextex_1->id == vertices[1] || canidateVextex_2->id == vertices[1])) // also check in reverse
				{
					engine::mesh::EdgeKey edgekey_1 = engine::mesh::EdgeKey{ vertices[0],vertices[1] };
					innerEdgeKeyset.insert(edgekey_1);

				}

				// first pair of edge [1,2]
				if ((canidateVextex_1->id == vertices[1] || canidateVextex_2->id == vertices[1])
					|| (canidateVextex_1->id == vertices[2] || canidateVextex_2->id == vertices[2])) // also check in reverse
				{
					engine::mesh::EdgeKey edgekey_2 = engine::mesh::EdgeKey{ vertices[1],vertices[2] };
					innerEdgeKeyset.insert(edgekey_2);
				}

				// first pair of edge [2,0]
				if ((canidateVextex_1->id == vertices[2] || canidateVextex_2->id == vertices[2])
					|| (canidateVextex_1->id == vertices[0] || canidateVextex_2->id == vertices[0])) // also check in reverse
				{
					engine::mesh::EdgeKey edgekey_3 = engine::mesh::EdgeKey{ vertices[2],vertices[0] };
					innerEdgeKeyset.insert(edgekey_3);
				}

			}


			// check if edge is rewireable
			for (auto& edgeKey : innerEdgeKeyset)
			{
				if (LODClusterMeshTree.edgeKeyToEdgeMap.count(edgeKey))
				{
					auto edgeID = LODClusterMeshTree.edgeKeyToEdgeMap.at(edgeKey);
					auto edgePtr = LODClusterMeshTree.edgePool.at(edgeID);

					// if the edge does not contain the edge to be rewired 
					// then this collapse would lead to a douplicate edge creation

					auto refPrim1 = edgePtr->prims.at(clusterType)[0];
					auto refPrim2 = edgePtr->prims.at(clusterType)[1];

					bool prim1Exist = toRewire.count(refPrim1);
					bool prim2Exist = toRewire.count(refPrim2);

					prim1Exist += toRemovePrims.count(refPrim1);
					prim2Exist += toRemovePrims.count(refPrim2);

					if (!prim1Exist && !prim2Exist)
						return true;

				}
			}
			return false;
	};

	if (capacityCheck())
		return { 0,{},{} };

	std::unordered_map<engineID_t,engineID_t> OldPrimToOuterEdge;
	OldPrimToOuterEdge.reserve(toRemovePrims.size());

	std::unordered_set<engineID_t> outerEdges;
	outerEdges.reserve(toRemovePrims.size());

	// debug
	std::unordered_map<engineID_t, engine::mesh::Edge*> visitedEdges;

	auto primitiveRemoval = [&](engineID_t p_id)
	{
			// 1. remove from cluster/level sets
			clusterPtr->primitives.erase(p_id);
			LODClusterMeshTree.primitiveMap.at(clusterType).erase(p_id);
			LODClusterMeshTree.clusterTypePrimitivePool.at(clusterType).erase(p_id);

			// 2. unlink from vertices (level-scoped)
			auto* p = LODClusterMeshTree.primitivePool.at(p_id);
			for (int i = 0; i < 3; ++i) 
			{
				engineID_t v = p->vertices[i];
				LODClusterMeshTree.vertexToPrimMap.at(clusterType).at(v).erase(p_id);
			}

			// 3. unlink from edges at this level; remove empty edges at this level
			for (int i = 0; i < 3; ++i) 
			{
				engineID_t e_id = p->edges[i];
				auto* e = LODClusterMeshTree.edgePool.at(e_id);

				// edge must have a slot for this level if p lives here
				auto& slots = e->prims.at(clusterType);   // {a,b} engineID_t
				if (slots[0] == p_id) 
				{
					slots[0] = 0;
				}

				if (slots[1] == p_id) 
				{
					slots[1] = 0;
				}

				const bool emptyLevel = (slots[0] == 0 && slots[1] == 0);

				if (emptyLevel) 
				{
					e->prims.erase(clusterType);
					LODClusterMeshTree.edgeMap.at(clusterType).erase(e_id);

					clusterPtr->edges.erase(e_id);
					// if no level references this edge any more, purge global edge & key map
					if (e->prims.empty()) 
					{
						// remove key -> id mapping using the canonical key
						engine::mesh::EdgeKey k{ e->vertices[0], e->vertices[1] };
						LODClusterMeshTree.edgeKeyToEdgeMap.erase(k);

						// finally drop the edge object
						LODClusterMeshTree.edgePool.erase(e_id);
					}
				}



				// check if outer edge of the patch
				if ((canidateVextex_1->id != e->vertices[0] && canidateVextex_2->id != e->vertices[1])
					&& (canidateVextex_1->id != e->vertices[1] && canidateVextex_2->id != e->vertices[0])) // also check in reverse
				{
					OldPrimToOuterEdge[p_id] = e_id;
					outerEdges.insert(e_id);
				}

				visitedEdges[e_id] = e;
			}
	};


	for (auto& p_id : toRemovePrims)
	{
		primitiveRemoval(p_id);
	}


	for (auto& p_id : toRewire)
	{
		primitiveRemoval(p_id);
	}

	// Begin rewiring 
	
	// beginning with creating the new prims to replace the to rewired prim that has been preamtively removed
	std::unordered_map<engineID_t, engine::mesh::Primitive*> newPrimsMap;
	std::unordered_map<engineID_t, engineID_t> oldPrimToNewPrimMap;

	for (auto toRewirePrimID : toRewire)
	{
		auto toRewirePrimPtr = LODClusterMeshTree.primitivePool.at(toRewirePrimID);
		engine::utils::array<engineID_t,3> vertices;
		// getting the new coords 
		for (int i = 0; i < 3; i++)
		{
			engineID_t v = (toRewirePrimPtr->vertices[i] == canidateVextex_1->id || toRewirePrimPtr->vertices[i] == canidateVextex_2->id) ? keepV->id : toRewirePrimPtr->vertices[i];
			vertices.add(v);
		}

		engine::mesh::PrimKey primKey = engine::mesh::PrimKey{ vertices[0],vertices[1] ,vertices[2] };

		if (LODClusterMeshTree.primKeyToPrimMap.count(primKey) >= 1)
		{
			// prim already exists
			auto primID = LODClusterMeshTree.primKeyToPrimMap.at(primKey);
			auto primPtr = LODClusterMeshTree.primitivePool.at(primID);

			oldPrimToNewPrimMap[toRewirePrimPtr->id] = primID;
			newPrimsMap[primID] = primPtr;
		}
		else
		{
			// prim does not exist 

			// creating the new prim
			engine::mesh::Primitive* newPrim = new engine::mesh::Primitive();
			newPrim->id = LODClusterMeshTree.nextAvailablePrimitiveID++;
			newPrim->vertices = vertices;
			LODClusterMeshTree.clusterTypePrimitivePool.at(clusterType)[newPrim->id] = clusterPtr->id;
			LODClusterMeshTree.vertexToPrimMap.at(clusterType).at(newPrim->vertices[0]).insert(newPrim->id); // book keeping
			LODClusterMeshTree.vertexToPrimMap.at(clusterType).at(newPrim->vertices[1]).insert(newPrim->id); // book keeping
			LODClusterMeshTree.vertexToPrimMap.at(clusterType).at(newPrim->vertices[2]).insert(newPrim->id); // book keeping
			//  centroid calculation
			auto v0 = LODClusterMeshTree.vertexPool.at(newPrim->vertices[0]);
			auto v1 = LODClusterMeshTree.vertexPool.at(newPrim->vertices[1]);
			auto v2 = LODClusterMeshTree.vertexPool.at(newPrim->vertices[2]);


			newPrim->centroid = {
				((v0->coords.x + v1->coords.x + v2->coords.x) / 3),
				((v0->coords.y + v1->coords.y + v2->coords.y) / 3),
				((v0->coords.z + v1->coords.z + v2->coords.z) / 3)
			};


			// update the global pools
			LODClusterMeshTree.primitiveMap.at(clusterType).insert(newPrim->id);
			LODClusterMeshTree.primitivePool[newPrim->id] = newPrim;
			LODClusterMeshTree.primKeyToPrimMap[primKey] = newPrim->id;
			LODClusterMeshTree.primToCluster[newPrim->id].insert(clusterPtr->id);

			clusterPtr->primitives.insert(newPrim->id);

			oldPrimToNewPrimMap[toRewirePrimPtr->id] = newPrim->id;
			newPrimsMap[newPrim->id] = newPrim;

		}
	}

	// handling edges
	// assigning back outer edges
	for (auto& [oldPrimID,newPrimID ] : oldPrimToNewPrimMap)
	{
		auto outerEdgePtr = LODClusterMeshTree.edgePool.at(OldPrimToOuterEdge.at(oldPrimID));
		// inserting the newPrim into the empty slot of the edge
		auto& slots = outerEdgePtr->prims.at(clusterType);   // {a,b} engineID_t
		if (slots[0] == 0 )
		{
			slots[0] = newPrimID; 
		}
		if (slots[1] == 0)
		{
			slots[1] = newPrimID;
		}

		auto newPrimPtr = LODClusterMeshTree.primitivePool.at(newPrimID);
		newPrimPtr->edges.add(outerEdgePtr->id); 
	}


	std::unordered_map<engineID_t, std::unordered_set<engine::mesh::EdgeKey, engine::mesh::EdgeHashByVertexId, engine::mesh::EdgeKeyEq>>  primToEdgeKey;
	primToEdgeKey.reserve(newPrimsMap.size());

	std::unordered_map<engine::mesh::EdgeKey, std::unordered_set<engineID_t>, engine::mesh::EdgeHashByVertexId, engine::mesh::EdgeKeyEq>  edgeKeyToPrim;
	edgeKeyToPrim.reserve(newPrimsMap.size() * 2);

	// precreate all the edge key from the new prims
	for (auto& [p_id, p_ptr] : newPrimsMap)
	{

		// creating all edges of the prim
		engine::mesh::EdgeKey edgekey_1 = engine::mesh::EdgeKey{ p_ptr->vertices[0],p_ptr->vertices[1] };
		engine::mesh::EdgeKey edgekey_2 = engine::mesh::EdgeKey{ p_ptr->vertices[1],p_ptr->vertices[2] };
		engine::mesh::EdgeKey edgekey_3 = engine::mesh::EdgeKey{ p_ptr->vertices[2],p_ptr->vertices[0] };

		primToEdgeKey[p_id].insert(edgekey_1);
		primToEdgeKey[p_id].insert(edgekey_2);
		primToEdgeKey[p_id].insert(edgekey_3);

		edgeKeyToPrim[edgekey_1].insert(p_id);
		edgeKeyToPrim[edgekey_2].insert(p_id);
		edgeKeyToPrim[edgekey_3].insert(p_id);
 	}


	// check edge existence
	// Note: 2 cases!
	// As for existing edges
	// case 1: in case where is collapsing pass is snapping to a optimal_v (w = optimal_v)
	// the only edges that exists are the outer patch edges.
	// The outer egdes are already handled and are already assigned.
	// Since we pre create all edges for the new prims ONLY 1 edge will be
	// FOUND but it will skiped.

	// case 2: in case where is collapsing pass is snapping to boundary/V1/V2 (w = u/v)
	// there WILL two edges that already exists. With ONE of its side being empty (0,a)
	// in this case only filling the empty slot.

	// case boundary/V1/V2 (w = u/v)
	//std::cout << "Beginnging Edge asssignments\n";
	//logfile << "Beginnging Edge asssignments\n";

	if (isVertexSpanningPass)
	{
		for (auto& [edgeKey, newPrimIDs] : edgeKeyToPrim)
		{

			if (LODClusterMeshTree.edgeKeyToEdgeMap.count(edgeKey) > 0)
			{
				// edge exists
				auto eID = LODClusterMeshTree.edgeKeyToEdgeMap.at(edgeKey);
				auto ePtr = LODClusterMeshTree.edgePool.at(eID);

				// if outer edge then skipp
				// since its already handled above
				if (outerEdges.count(eID) > 0)
				{	
					continue;
				}


				// the size of this MUST be 1 as well
				// just to be sure we check it for debugging reason
				//if (newPrimIDs.size() > 1)
				//	std::cout << ""; //break  if needed should idealy never enter

				// get the first element
				auto primID = *newPrimIDs.begin();

				auto newPrimPtr = LODClusterMeshTree.primitivePool.at(primID);

				// update/insert the primitive to the empty slot
				auto& slots = ePtr->prims.at(clusterType);   // {a,b} engineID_t
				if (slots[0] == 0) 
				{
					slots[0] = primID;

				}
				if (slots[1] == 0) 
				{
					slots[1] = primID;
				}

				newPrimPtr->edges.add(eID);

			}
			else // edge does not exist
			{

				// creates new an edge using a the edge key
				engine::mesh::Edge* newEdge = new engine::mesh::Edge{ edgeKey.a,edgeKey.b };
				newEdge->id = LODClusterMeshTree.nextAvailableEdgeID++;
				newEdge->prims[clusterType]; // creates an empty at the is level 
				collapseResult.incidentEdges.push_back(newEdge->id);


				LODClusterMeshTree.edgeKeyToEdgeMap[edgeKey] = newEdge->id;
				LODClusterMeshTree.edgeMap.at(clusterType).insert(newEdge->id);
				LODClusterMeshTree.edgePool[newEdge->id] = newEdge;

				clusterPtr->edges.insert(newEdge->id);
				for (auto& p_id : newPrimIDs)
				{
					engine::mesh::Primitive* newPrimPtr = LODClusterMeshTree.primitivePool.at(p_id);
					newPrimPtr->edges.add(newEdge->id);

					newEdge->prims[clusterType].add(newPrimPtr->id);
				}

			}
		}
	}
	else // case (w = optimal_v)
	{

		for (auto& [edgeKey, newPrimIDs] : edgeKeyToPrim)
		{
			if (LODClusterMeshTree.edgeKeyToEdgeMap.count(edgeKey) > 0)
			{
				continue; // if exist skip
			}

			// creates new an edge using a the edge key
			engine::mesh::Edge* newEdge = new engine::mesh::Edge{ edgeKey.a,edgeKey.b };
			newEdge->id = LODClusterMeshTree.nextAvailableEdgeID++;
			newEdge->prims[clusterType]; // creates an empty at the is level 
			collapseResult.incidentEdges.push_back(newEdge->id);


			LODClusterMeshTree.edgeKeyToEdgeMap[edgeKey] = newEdge->id;
			LODClusterMeshTree.edgeMap.at(clusterType).insert(newEdge->id);
			LODClusterMeshTree.edgePool[newEdge->id] = newEdge;

			for (auto& p_id : newPrimIDs)
			{
				engine::mesh::Primitive* newPrimPtr = LODClusterMeshTree.primitivePool.at(p_id);
				newPrimPtr->edges.add(newEdge->id);
				newEdge->prims[clusterType].add(newPrimPtr->id);
			}

		}
	}

	return collapseResult;
}

engine::math::Mat4f LODGenarator::computeQ(engineID_t v_id, engine::cluster::LODClusterMeshTree& LODClusterMeshTree, engine::cluster::HierarchyLevel level)
{

	const std::pmr::unordered_set<engineID_t>& listOfPrim = LODClusterMeshTree.vertexToPrimMap.at(level).at(v_id);
	std::vector<engine::math::Mat4f> Kp_mx; // Kp for primitives connected to v

	/* compute K matrix for each connected Prims */
	for (engineID_t p_id : listOfPrim)
	{
		const auto p_pos_0 = *LODClusterMeshTree.vertexPool.at(LODClusterMeshTree.primitivePool.at(p_id)->vertices[0]);
		const auto p_pos_1 = *LODClusterMeshTree.vertexPool.at(LODClusterMeshTree.primitivePool.at(p_id)->vertices[1]);
		const auto p_pos_2 = *LODClusterMeshTree.vertexPool.at(LODClusterMeshTree.primitivePool.at(p_id)->vertices[2]);

		// Calculate normal using vec3 operations
		engine::math::Vec4f normal = calculate_primitive_normal(p_pos_0.coords, p_pos_1.coords, p_pos_2.coords);

		// Handle degenerate primitives (zero normal length)
		if (engine::math::length(normal) < std::numeric_limits<float>::epsilon()) {
			continue; // Skip degenerate primitives
		}

		// compute d component
		float d = -(normal.dot(p_pos_0.coords)); // Dot product

		// create P Component (homogeneous plane equation coefficients)
		engine::math::Vec4f P = { normal.x, normal.y, normal.z, d };

		//Compute K Quadratic Matrix (P * P^T)
		engine::math::Mat4f K_mx4;
		K_mx4.row[0] = { P._a * P._a, P._a * P._b , P._a * P._c, P._a * P._d };
		K_mx4.row[1] = { P._a * P._b, P._b * P._b , P._b * P._c, P._b * P._d };
		K_mx4.row[2] = { P._a * P._c, P._b * P._c , P._c * P._c, P._c * P._d };
		K_mx4.row[3] = { P._a * P._d, P._b * P._d , P._c * P._d, P._d * P._d };
		Kp_mx.push_back(K_mx4);
	}

	// Compute Q (sum of Kp for connected primitives)
	engine::math::Mat4f Q_sum = {}; // Initialize to zero matrix

	for (const auto& Kp : Kp_mx)
	{
		Q_sum = Q_sum + Kp; // Use overloaded Mx4 + operator
	}

	return Q_sum;

}
