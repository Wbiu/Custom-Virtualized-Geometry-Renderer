#pragma once
#include <stdexcept> 
#include "engine/utils/utils.h"
#include "engine/clustering/AABB.h"
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <memory_resource>


namespace engine::cluster {
	struct Cluster;  // Forward declaration

	enum HierarchyLevel
	{
		MODEL_LOADING_LEVEL,
		INITIAL_LEVEL,
		LOD0,
		LOD1,
		LOD2,
		LOD3,
	};
}

namespace engine::mesh
{

	enum LOD
	{
		LOD0,
		LOD1,
		LOD2,
		LOD3,
	};

	struct Vertex_
	{
		engine::math::Vec4f coords{};
		engine::math::Vec4f color{ 0,1.0f,0,1.0f };
		engine::math::Vec4f normal{};
	};


	struct alignas(16) Vertex
	{

		engine::math::Vec4f coords{};
		engine::math::Vec4f color {0,1.0f,0,1.0f};
		engine::math::Vec4f normal{};
		engineID_t id = 0;
		uint32_t  _pad[3]; // pad to 64 bytes

		inline bool operator==(const Vertex& other) {
			return coords.x == other.coords.x &&
				coords.y == other.coords.y &&
				coords.z == other.coords.z &&
				coords.w == other.coords.w &&
				color.r == other.color.r &&
				color.g == other.color.g &&
				color.b == other.color.b && 
				color.a == other.color.a;
		}

		std::vector<float> toFloats() const {
			return {
				coords.x, coords.y, coords.z, coords.w,
				color.r, color.g, color.b, color.a,
				normal.x, normal.y, normal.z, normal.w
			};
		}
	};

	inline bool operator==(const Vertex& a, const Vertex& b) {
		return a.coords.x == b.coords.x &&
			a.coords.y == b.coords.y &&
			a.coords.z == b.coords.z &&
			a.coords.w == b.coords.w &&
			a.color.r == b.color.r &&
			a.color.g == b.color.g &&
			a.color.b == b.color.b &&
			a.color.a == b.color.a;
	}

	struct VertexHash {
		std::size_t operator()(const Vertex& v) const noexcept {
			std::hash<float> hasher;
			std::size_t hx = hasher(v.coords.x);
			std::size_t hy = hasher(v.coords.y);
			std::size_t hz = hasher(v.coords.z);
			return hx ^ (hy << 1) ^ (hz << 2);
		}
	};


	// forward declaration Primitive
	struct Primitive;

	struct Edge
	{
		engineID_t id = 0;
		engine::utils::array<engineID_t, 2> vertices;
		std::unordered_map<engine::cluster::HierarchyLevel,engine::utils::array<engineID_t, 2>> prims;

		Edge(Vertex* a, Vertex* b) {
			if (a->id <= b->id)
			{
				vertices[0] = a->id; vertices[1] = b->id; 
			}
			else 
			{
				vertices[0] = b->id; vertices[1] = a->id; 
			}
		}

		Edge(engineID_t v_id_a, engineID_t v_id_b) {
			if (v_id_a <= v_id_b) 
			{
				vertices[0] = v_id_a; vertices[1] = v_id_b; 
			}
			else
			{
				vertices[0] = v_id_a; vertices[1] = v_id_b; 
			}
		}

		Edge() : id(0), vertices{ 0,0 } {
			prims.insert({ engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL,{0,0} });
			}

		bool operator==(const Edge& o) const noexcept {
			return vertices[0] == o.vertices[0] &&
				vertices[1] == o.vertices[1];
		}
		
		bool operator<(const Edge& o) const noexcept {
			auto a0 = vertices[0], a1 = vertices[1];
			auto b0 = o.vertices[0], b1 = o.vertices[1];
			if (a0 > a1) std::swap(a0, a1);
			if (b0 > b1) std::swap(b0, b1);
			return (a0 < b0) || (a0 == b0 && a1 < b1);
		}
			
	};

	struct EdgeKey {
		engineID_t a, b;
		EdgeKey(Vertex* v1, Vertex* v2) { 
			if (v1->id <= v2->id) 
			{ 
				a = v1->id; b = v2->id; 
			} 
			else 
			{ 
				a = v2->id; b = v1->id; 
			} 
		}
		EdgeKey(engineID_t x, engineID_t y) { if (x <= y) { a = x; b = y; } else { a = y; b = x; } }
	};

	struct EdgeHashByVertexId {
		size_t operator()(const EdgeKey& k) const noexcept {
			engineID_t a = k.a, b = k.b;
			if (a > b) std::swap(a, b);
			uint64_t h = std::hash<uint64_t>{}(a);
			uint64_t k2 = std::hash<uint64_t>{}(b);
			h ^= k2 + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
			return static_cast<size_t>(h);
		}

	};

	struct EdgeKeyEq 
	{
		bool operator()(const EdgeKey& x, const EdgeKey& y) const noexcept {
			return x.a == y.a && x.b == y.b;
		}
	};

	struct Primitive
	{
		engineID_t id = 0;
		engine::math::Vec3f centroid{};
		engine::cluster::Cluster* cluster = nullptr;
		engine::utils::array<engineID_t, 3> vertices;
		engine::utils::array<engineID_t, 3> edges;

	};

	struct PrimKey {
		engineID_t a{ 0 }, b{ 0 }, c{ 0 }; // sorted, winding-invariant

		PrimKey() = default;

		// from three vertex IDs
		PrimKey(engineID_t i, engineID_t j, engineID_t k) noexcept { set(i, j, k); }

		// from three vertices
		PrimKey(const engine::mesh::Vertex* v1,
			const engine::mesh::Vertex* v2,
			const engine::mesh::Vertex* v3) noexcept {
			set(v1->id, v2->id, v3->id);
		}

		// canonicalize (a <= b <= c)
		void set(engineID_t i, engineID_t j, engineID_t k) noexcept {
			if (i > j) std::swap(i, j);
			if (j > k) std::swap(j, k);
			if (i > j) std::swap(i, j);
			a = i; b = j; c = k;
		}

		//  sanity check
		bool valid() const noexcept {
			return a && b && c && a != b && b != c && a != c;
		}

		bool operator==(const PrimKey& o) const noexcept {
			return a == o.a && b == o.b && c == o.c;
		}
	};

	// hash for unordered_map/set
	struct PrimKeyHash {
		size_t operator()(const PrimKey& k) const noexcept {
			size_t h = std::hash<engineID_t>{}(k.a);
			h ^= std::hash<engineID_t>{}(k.b) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
			h ^= std::hash<engineID_t>{}(k.c) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
			return h;
		}
	};

	struct PrimKeyEq {
		bool operator()(PrimKey const& x, PrimKey const& y) const noexcept {
			return x.a == y.a && x.b == y.b && x.c == y.c;
		}
	};

	struct Model
	{
		std::pmr::memory_resource* memoryPool{ std::pmr::get_default_resource() }; // 

		engineID_t nextAvailablePrimitiveID = 0;
		engineID_t nextAvailableVertexID = 0;
		engineID_t nextAvailableEdgeID = 0;

		int modelIndexElementCount = 0;
		int aabbIndexElementCount = 0;
		engine::mesh::AABB aabb;

		std::pmr::unordered_map<engineID_t, engine::mesh::Primitive*> primitivePool;
		std::pmr::unordered_map<engine::mesh::PrimKey,engineID_t,PrimKeyHash, PrimKeyEq> primKeyToPrimMap;
		std::pmr::unordered_map<engine::cluster::HierarchyLevel, std::pmr::unordered_set<engineID_t>> primitiveMap;

		std::pmr::unordered_map<engineID_t, engine::mesh::Edge*> edgePool;
		std::pmr::unordered_map<engine::cluster::HierarchyLevel, std::pmr::unordered_set<engineID_t>> edgeMap;
		std::pmr::unordered_map<engine::mesh::EdgeKey, engineID_t, engine::mesh::EdgeHashByVertexId, engine::mesh::EdgeKeyEq> edgeKeyToEdgeMap;

		std::pmr::unordered_map<engineID_t,engine::mesh::Vertex*> vertexPool;
		std::pmr::unordered_map<engine::cluster::HierarchyLevel, std::pmr::unordered_map<engineID_t,std::pmr::unordered_set<engineID_t>>> vertexToPrimMap;
		std::pmr::vector<engineID_t> vertexIndcicesBufferOrder;

		Model() = default;

		explicit Model(std::pmr::memory_resource* r) 
			: memoryPool(r),
			primitivePool(r), 
			primKeyToPrimMap(0, PrimKeyHash{}, PrimKeyEq{}, r),
			primitiveMap(r), 
			edgePool(r), 
			edgeMap(r), 
			edgeKeyToEdgeMap(static_cast<std::size_t>(0), engine::mesh::EdgeHashByVertexId{}, engine::mesh::EdgeKeyEq{}, r),
			vertexPool(r), 
			vertexToPrimMap(r), 
			vertexIndcicesBufferOrder(r) {}
	};

	struct ClusterDrawRange {
		uint32_t firstIndex;
		uint32_t indexCount;
		engineID_t clusterId;
		engine::cluster::HierarchyLevel level;
	};

	struct LinearizedModelData
	{
		std::vector<engine::mesh::Vertex> vertices;
		std::vector<engineID_t> indices;
		std::unordered_map<engineID_t, uint32_t> vertexIdToIdx;
		std::unordered_map<engine::cluster::HierarchyLevel, uint32_t> perLevelStartingVertexIdx;
		std::unordered_map<engine::cluster::HierarchyLevel, uint32_t> perLevelIndexCount;

		std::unordered_map<engineID_t, ClusterDrawRange> clusterDrawRange;
	};

	inline void dataPoolTransfer(mesh::Model& dest, mesh::Model& src)
	{
		dest = std::move(src);
	}

	inline void linearlizeVertices(std::vector<engine::mesh::Vertex>& vertices,
		std::unordered_map<engineID_t, engineID_t>& vertexIdToIdx,
		const std::unordered_map<engineID_t,engine::mesh::Vertex*> verticesMap)
	{
		// all IDs starts at 1 but ARRAY at zero
		for (engineID_t i = 1 ; i < (verticesMap.size() + 1) ; i++) // (verticesMap.size() + 1)  '+1' to include the element as well
		{
			vertexIdToIdx[verticesMap.at(i)->id] = static_cast<uint32_t>(vertices.size());
			vertices.emplace_back(*verticesMap.at(i));
		}
	}

	inline void linearlizeIndexElements(std::vector<unsigned int>& indices, 
		std::unordered_map<engineID_t, engineID_t>& vertexIdToIdx,
		std::unordered_map<engineID_t, engine::mesh::Primitive*> primitiveMap)
	{
		for (auto& [p_id, p_ptr] : primitiveMap)
		{
			for (int i = 0; i < 3; i++)
			{
				indices.emplace_back(vertexIdToIdx.at(p_ptr->vertices[i]));
			}
		}
	}

};