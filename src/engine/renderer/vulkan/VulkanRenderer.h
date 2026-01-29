#pragma once
#include "vulkan_helper_extention.h"
#include <imgui/imgui_impl_vulkan.h>

class VulkanRenderer
{
private:

	// vars 

	int _WINDOW_WIDTH = 0;
	int _WINDOW_HEIGHT = 0;
	GLFWwindow* _GLFwindow;
	const char* _applicationName;

	/* flaged resized window */
	bool _framebufferResized = false;


	/* vk objects */
	VkInstance _vkInstance;
	VkSurfaceKHR _vkSurface;
	VkPhysicalDevice _vkPhysicalDevice = VK_NULL_HANDLE; //<-- the actual GPU physical decive
	VkDevice _vkDevice;		// logical device
	VkRenderPass _vkRenderPass;

	/* descriptors */
	VkDescriptorSetLayout _vkDescriptorSetLayout;
	VkDescriptorPool _vkDescriptorPool;
	std::vector<VkDescriptorSet> _vkDescriptorSets;

	/* descriptors pool for UI */
	VkDescriptorPool _vkDescriptorPoolUI;

	/* pipe line */
	VkPipelineLayout _vkPipelineLayout;
	VkPipeline _vkGraphicsPipeline;

	/* RT Graphics Pipeline */
	VkPipeline _vkRtGraphicsPipeline;

	/* RT Pipeline (ray tracing) */
	VkPipeline _vkRtPipeline;

	/* vk swap chain */
	VkSwapchainKHR _vkSwapChain;
	std::vector<VkImage> _vkSwapChainImages;
	std::vector<VkImageView> _vkSwapChainImageViews;
	VkFormat _vkSwapChainImageFormat;
	VkExtent2D _vkSwapChainExtent;
	std::vector<VkFramebuffer> _vkSwapChainFramebuffers;
	std::vector<VkImageLayout> _swapchainLayouts;


	/* vk commands */
	VkCommandPool _vkCommandPool;
	std::vector<VkCommandBuffer> _vkCommandBuffers;

	/* queues */
	VkQueue _vkGraphicsQueue;
	VkQueue _vkPresentationQueue;
	uint32_t _vkQueueFamilyIdx = 0;

	/*vk debug*/
	VkDebugUtilsMessengerEXT _vkDebugMessender;

	/* vk Multisampling */
	VkSampleCountFlagBits _vkMsaaSamples = VK_SAMPLE_COUNT_1_BIT;
	VkImageView _vkColorImageView;
	VkImage _vkColorImage;
	VkDeviceMemory _vkColorImageMemory;


	/* depth buffer */
	VkImage _vkDepthImage;
	VkDeviceMemory _vkDepthImageMemory;
	VkImageView _vkDepthImageView;

	/* rendering memory data (vertices/indices) */
	VkBuffer _vkVertexBuffer;
	VkDeviceMemory _vkVertexBufferMemory; //  allocated memory on the GPU

	VkBuffer _vkIndexBuffer;
	VkDeviceMemory _vkIndexBufferMemory;

	/* Ray tracing output */
	VkImage        _vkRtColorImage;
	VkDeviceMemory _vkRtColorImageMemory;
	VkImageView    _vkRtColorImageView;

	/* RT pipeline properties*/
	VkPhysicalDeviceRayTracingPipelinePropertiesKHR _vkRtPipelineProps{};

	/* RT Shader Binding Table (SBT) */
	VkBuffer _vkRtSbtBuffer;
	VkDeviceMemory _vkRtSbtMemory;

	VkStridedDeviceAddressRegionKHR _vkSbtRaygenRegion{};
	VkStridedDeviceAddressRegionKHR _vkSbtMissRegion{};
	VkStridedDeviceAddressRegionKHR _vkSbtHitRegion{};
	VkStridedDeviceAddressRegionKHR _vkSbtCallableRegion{};

	// Ray tracing function pointers to the device driver
	PFN_vkCreateRayTracingPipelinesKHR _vkRtCreateRayTracingPipelinesKHR_PFN = nullptr;
	PFN_vkGetRayTracingShaderGroupHandlesKHR _vlRtGetRayTracingShaderGroupHandlesKHR_PFN = nullptr;
	PFN_vkCmdTraceRaysKHR _vkRtCmdTraceRaysKHR_PFN = nullptr;

	PFN_vkCreateAccelerationStructureKHR _vkCreateAccelerationStructureKHR_PFN = nullptr;
	PFN_vkDestroyAccelerationStructureKHR _vkDestroyAccelerationStructureKHR_PFN = nullptr;
	PFN_vkGetAccelerationStructureBuildSizesKHR _vkGetAccelerationStructureBuildSizesKHR_PFN = nullptr;
	PFN_vkGetAccelerationStructureDeviceAddressKHR _vkGetAccelerationStructureDeviceAddressKHR_PFN = nullptr;

	//PFN_vkBuildAccelerationStructuresKHR _vkBuildAccelerationStructuresKHR_PFN = nullptr;
	PFN_vkCmdBuildAccelerationStructuresKHR _vkCmdBuildAccelerationStructuresKHR_PFN = nullptr;

	// Test ray tracing acceleration structures 
	VkAccelerationStructureKHR _vkTestBlas = VK_NULL_HANDLE;
	VkAccelerationStructureKHR _vkTestTlas = VK_NULL_HANDLE;

	// Backing buffers for the AS objects
	VkBuffer _vkTestBlasBuffer = VK_NULL_HANDLE;
	VkDeviceMemory _vkTestBlasMemory = VK_NULL_HANDLE;

	VkBuffer _vkTestTlasBuffer = VK_NULL_HANDLE;
	VkDeviceMemory _vkTestTlasMemory = VK_NULL_HANDLE;

	// Scratch buffer used during build (we’ll allocate/free it around builds)
	VkBuffer _vkAsScratchBuffer = VK_NULL_HANDLE;
	VkDeviceMemory _vkAsScratchMemory = VK_NULL_HANDLE;


	VkDeviceAddress _vkTestBlasDeviceAddress = 0;
	VkDeviceAddress _vkTestTlasDeviceAddress = 0;

	/* RT Structes */
	VkAccelerationStructureKHR _vkMeshBlas = VK_NULL_HANDLE;
	VkBuffer _vkMeshBlasBuffer = VK_NULL_HANDLE;
	VkDeviceMemory _vkMeshBlasMemory = VK_NULL_HANDLE;
	VkDeviceAddress _vkMeshBlasDeviceAddress = 0;

	VkAccelerationStructureKHR _vkMeshTlas = VK_NULL_HANDLE;
	VkBuffer _vkMeshTlasBuffer = VK_NULL_HANDLE;
	VkDeviceMemory _vkMeshTlasMemory = VK_NULL_HANDLE;
	VkDeviceAddress _vkMeshTlasDeviceAddress = 0;

	// RT vertex/index buffers (separated from raster)
	VkBuffer _vkRtVertexBuffer = VK_NULL_HANDLE;
	VkDeviceMemory _vkRtVertexBufferMemory = VK_NULL_HANDLE;
	VkBuffer _vkRtIndexBuffer = VK_NULL_HANDLE;
	VkDeviceMemory _vkRtIndexBufferMemory = VK_NULL_HANDLE;

	/* RT Camera */
	engine::vk::RtCameraUBO _rtCameraHost{};

	std::vector<VkBuffer> _vkRtCameraBuffers;
	std::vector<VkDeviceMemory> _vkRtCameraBuffersMemory;
	std::vector<void*> _vkRtCameraBuffersMapped;

	/* uniform buffers */
	std::vector<VkBuffer> _vkUniformBuffers;
	std::vector<VkDeviceMemory> _vkUniformBuffersMemory;
	std::vector<void*> _vkUniformBuffersMapped;
	engine::math::Mat4f* _mat4Uniform;

	engine::vk::RtSamples* _rtSamples;
	std::vector<VkBuffer> _vkRtSampleUniform;
	std::vector<VkDeviceMemory> _vkRtUniformBuffersMemory;
	std::vector<void*> _vkRtUniformBuffersMapped;


	/* vk sync controlls*/
	std::vector<VkSemaphore> _vkImageAvailableSemaphores;
	std::vector<VkSemaphore> _vkRenderFinishedSemaphores;
	std::vector<VkFence> _vkInFlightFences;
	std::vector<VkFence> _vkImagesInFlight;

	std::vector <engine::vk::RtClusterBlas > _rtClusterBlases;


	uint32_t _vkRtVertexCount = 0;
	uint32_t _vkRtIndexCount = 0;

	// Cluster-based top-level AS (TLAS) built from _rtClusterBlases
	VkAccelerationStructureKHR _vkClusterTlas = VK_NULL_HANDLE;
	VkBuffer _vkClusterTlasBuffer = VK_NULL_HANDLE;
	VkDeviceMemory _vkClusterTlasMemory = VK_NULL_HANDLE;


	VkBuffer _vkRtInstanceBuffer = VK_NULL_HANDLE;
	VkDeviceMemory _vkRtInstanceBufferMemory = VK_NULL_HANDLE;


	/* vk required device extentions */
	const std::vector<const char*> _requiredDeviceExtensions = {
		VK_KHR_SWAPCHAIN_EXTENSION_NAME,
		VK_KHR_DYNAMIC_RENDERING_EXTENSION_NAME,
		VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME, // RT Extension
		VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME, // RT Extension
		VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME, // RT Extension
		VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME // RT Extension
	};

	/*vk validation layer */
	const std::vector<const char*> _validationLayers = {
		"VK_LAYER_KHRONOS_validation"
	};

	/* max precompute frames (CPU) */
	const static int _MAX_FRAMES_IN_FLIGHT = 2;

	/* track frame */
	uint32_t _currentFrame = 0;

	std::vector<engine::vk::DrawItem> _frameDrawItems;

	/* indirect draws */
	std::vector<engine::vk::IndirectDrawSet> _indirectPerFrame; // size = _MAX_FRAMES_IN_FLIGHT

	const uint32_t _MAX_INDIRECT_DRAWS = 65536;
	const VkDeviceSize _INDIRECT_BYTES =
		_MAX_INDIRECT_DRAWS * sizeof(VkDrawIndexedIndirectCommand);
	
	engine::vk::PerFrame _perDraw[_MAX_FRAMES_IN_FLIGHT];


	// GPU timing
	VkQueryPool _vkGpuTimestampQueryPool = VK_NULL_HANDLE;
	float       _vkTimestampPeriod = 0.0f;  
	double      _gpuLastFrameMs = 0.0;  
	std::array<bool, _MAX_FRAMES_IN_FLIGHT> _gpuTimestampReady{};

#ifdef NDEBUG
	const bool _enableValidationLayers = false;
#else
	const bool _enableValidationLayers = true;
#endif // NDEBUG


#ifdef LINEDRAWMODE
	bool _drawLineMode = true;
#else
	bool _drawLineMode = false;
#endif // LINEDRAWMODE





	// end vars


	// functions
	void initWindow();
	void drawFrame(bool drawScene);
	void recreateSwapChain();
	void recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex,bool drawScene);
	void updateUniformBuffer(uint32_t currentImage);
	void initVk();
	void createGpuTimestampQueryPool();
	void createVkInstance();
	bool checkValidationLayerSupport();
	void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo);
	std::vector<const char*> getRequiredInstanceExtensions();
	void setUpVkDebugMessenger();
	void createSurface();
	void selectPhysicalDevice();
	bool checkPhysicalDevices(VkPhysicalDevice& physicalDevice);
	engine::vk::QueueFamily getQueueFamilies(VkPhysicalDevice physicalDevice);
	bool checkDeviceExtensionSupport(VkPhysicalDevice physicalDevice);
	engine::vk::SurfaceSupportDetails querySwapChainSupport(VkPhysicalDevice physicalDevice);
	VkSampleCountFlagBits getMaxUsableSampleCount();
	void createLogicalDevice();
	void createSwapChain();
	VkSurfaceFormatKHR selectSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats);
	VkPresentModeKHR selectSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes);
	VkExtent2D selectSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities);
	void createImageViews();
	VkImageView createImageView(VkImage image, VkFormat format, VkImageAspectFlags aspectFlags, uint32_t mipLevels);
	VkFormat findDepthFormat();
	VkFormat findSupportedFormat(const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features);
	void createDescriptorSetLayout();
	void createGraphicsPipeline();
	VkResult createDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo,
		const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pDebugMessenger);
	static std::vector<char> readFile(const std::string& filename);
	VkShaderModule createVKShaderModule(const std::vector<char>& shaderCode);
	void createCommandPool();
	void createColorResources();
	void createImage(uint32_t width, uint32_t height, uint32_t mipLevels, VkSampleCountFlagBits numSamples, VkFormat format, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImage& image, VkDeviceMemory& imageMemory);
	uint32_t getMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);
	void createDepthResources();
	void transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout, uint32_t mipLevels);
	VkCommandBuffer beginSingleTimeCommands();
	bool hasStencilComponent(VkFormat format);
	void endSingleTimeCommands(VkCommandBuffer commandBuffer);
	void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& bufferMemory);
	void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);
	void createUniformBuffers();
	void createDescriptorPool();
	void createUIDescriptorPool();
	void createDescriptorSets();
	void createCommandBuffers();
	void createSyncObjects();
	void createIndirectBuffers();
	void createSSBO();
	void createFramebuffers();
	void updateSSBO(uint32_t currentImage);
	static void framebufferResizeCallback(GLFWwindow* window, int width, int height);
	static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
		VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
		VkDebugUtilsMessageTypeFlagsEXT messageType,
		const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
		void* pUserData);
	void cleanUpSwapChain();
	void destroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator);
	std::filesystem::path findShaderPath(const char* path);

	// RT functions
	void createRtColourResources();
	void createRtGraphicsPipeline();
	void createRayTracingPipeline();
	void queryRayTracingProperties();
	void createRayTracingSBT();

	void createRtCameraBuffers();

	void createTestBlas();
	void createTestTlas();

	// end functions
public:
	VulkanRenderer(int windowWidth, int windowHight, const char* applicationName);
	~VulkanRenderer();

	// call back to UI
	std::function<void(VkCommandBuffer)> callBackUI;
	std::function<void(uint32_t)> callBackOnSwapchainRecreateUI;

	void init();
	void cleanUp();
	void waitIdle();
	bool shouldCloseWindow();
	void submitDrawList(const std::vector<engine::mesh::ClusterDrawRange>& list, int renderMode);
	GLFWwindow* getWindow() { return _GLFwindow; };
	VkSurfaceKHR getSurface() { return _vkSurface; };
	void submitRenderData(const std::vector<engine::mesh::Vertex>& vertices, const std::vector<unsigned int>& indices);
	void submitUniform(engine::math::Mat4f* mat4, engine::vk::RtSamples* rtSamples);
	void draw(bool drawScene);
	void pollWindowInputEvents();

	void submitRtCameraData(const engine::vk::RtCameraUBO& data);
	void submitRTRenderData(const std::vector<engine::mesh::Vertex>& vertices, const std::vector<unsigned int>& indices);

	void buildClusterBlases(const std::vector<engine::vk::RtClusterBuildInfo>& clusters);

	void initImGUI_Info(ImGui_ImplVulkan_InitInfo* info);

	void buildClusterTlasAll();
	
	void buildClusterTlasVisible(const std::vector<engineID_t>& visibleClusters, uint32_t renderMode);

	double getGpuFrameTimeMs() const { return _gpuLastFrameMs; }

};
