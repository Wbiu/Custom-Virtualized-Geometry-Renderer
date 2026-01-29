#pragma once
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <stdexcept>
#include <vector>
#include <iostream>
#include <optional>
#include <algorithm> //for std::clamp
#include <set>
#include <array>
#include <fstream>
#include <engine/mesh/mesh.h>
#include <chrono>
#include <functional>
#include <filesystem>

namespace engine::vk
{
	/* swap-chain struct */
	struct SurfaceSupportDetails {
		VkSurfaceCapabilitiesKHR capabilities;
		std::vector<VkSurfaceFormatKHR> formats;
		std::vector<VkPresentModeKHR> presentModes;
	};

	/* vk queue family*/
	struct QueueFamily {
		std::optional<uint32_t> graphicsFamily;
		std::optional<uint32_t> presentFamily;
		bool isComplete()
		{
			return graphicsFamily.has_value() && presentFamily.has_value();
		}
	};

	static VkVertexInputBindingDescription getVertexBindingDescription() {
		VkVertexInputBindingDescription vkBindingDescription{};

		vkBindingDescription.binding = 0;
		vkBindingDescription.stride = sizeof(mesh::Vertex);
		vkBindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

		return vkBindingDescription;
	}

	static std::array<VkVertexInputAttributeDescription, 3> getVertexAttributeDescriptions() {

		std::array<VkVertexInputAttributeDescription, 3> attributeDescriptions{};
		/* vertex position attribute*/
		attributeDescriptions[0].binding = 0;
		attributeDescriptions[0].location = 0;
		attributeDescriptions[0].format = VK_FORMAT_R32G32B32A32_SFLOAT;
		attributeDescriptions[0].offset = offsetof(mesh::Vertex , mesh::Vertex::coords);

		/* color value attribute */
		attributeDescriptions[1].binding = 0;
		attributeDescriptions[1].location = 1;
		attributeDescriptions[1].format = VK_FORMAT_R32G32B32A32_SFLOAT;
		attributeDescriptions[1].offset = offsetof(mesh::Vertex, mesh::Vertex::color);

		/* normal coord attribute */
		attributeDescriptions[2].binding = 0;
		attributeDescriptions[2].location = 2;
		attributeDescriptions[2].format = VK_FORMAT_R32G32B32A32_SFLOAT;
		attributeDescriptions[2].offset = offsetof(mesh::Vertex, mesh::Vertex::normal);

		return attributeDescriptions;
	}

	struct DrawItem { uint32_t firstIndex, indexCount; };

	struct IndirectDrawSet 
	{
		VkBuffer buf = VK_NULL_HANDLE;
		VkDeviceMemory mem = VK_NULL_HANDLE;
		void* mapped = nullptr; // persistently mapped (host visible+coherent)
		uint32_t     count = 0;
	};


	// CPU-side layout must match std430 (vec4 is 16B aligned). For cluster coloured draw
	struct PerDrawGPU {
		uint32_t clusterId;
		uint32_t lod;
		uint32_t pad0, pad1;   // pad to 16B before the vec4
		float    color[4];     // or pack to uint32_t rgba8 and unpack in shader
	};

	struct PerFrame {
		VkBuffer     ssbo;
		VkDeviceMemory mem;
		void* mapped;
		uint32_t     capacity; // max draws this buffer can hold
	};

	struct RtCameraUBO {
		engine::math::Mat4f invViewProj;  // inverse(proj * view) for RT
		engine::math::Vec3f camPos;       // camera position in world space
		float padding = 0.0f;             // keep std140 alignment happy
	};


	struct RtClusterBlas
	{
		VkAccelerationStructureKHR blas = VK_NULL_HANDLE;
		VkDeviceAddress deviceAddress = 0;

		VkBuffer blasBuffer = VK_NULL_HANDLE;
		VkDeviceMemory blasMemory = VK_NULL_HANDLE;

		uint32_t firstIndex = 0;   // offset into global index buffer (in indices, not bytes)
		uint32_t indexCount = 0;   // number of indices for this cluster
		engineID_t clusterId = 0;   // engine cluster ID, for mapping to your cluster system
	};


	struct RtClusterBuildInfo
	{
		uint32_t firstIndex;  // index into global indices[]
		uint32_t indexCount;  // number of indices for this cluster
		engineID_t clusterId;   // which engine cluster this corresponds to
	};

	struct RtInstanceData
	{
		uint32_t baseIndex;   // first index into global indices[]
		uint32_t clusterId;   // optional for debugging / future
		uint32_t lodLevel;    // optional, for LOD logic later
		uint32_t renderMode;		  // render view MODE   
	};


	struct RtSamples
	{
		//RTAO
		uint32_t aoSampleCnt = 0;
		float aoMaxDistance = 0.0f;

		// Shadow
		uint32_t shadowSampleCnt = 0;
		float shadowConeAngleDeg = 0.0f;
	};


}