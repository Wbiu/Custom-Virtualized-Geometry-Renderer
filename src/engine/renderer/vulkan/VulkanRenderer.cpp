#include "VulkanRenderer.h"


VulkanRenderer::VulkanRenderer(int windowWidth, int windowHight, const char* applicationName)
	: _WINDOW_WIDTH(windowWidth), _WINDOW_HEIGHT(windowHight), _applicationName(applicationName)
{
}

VulkanRenderer::~VulkanRenderer()
{
}

void VulkanRenderer::init()
{
	initWindow();
	initVk();
}

void VulkanRenderer::cleanUp()
{
	cleanUpSwapChain();

	for (size_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; i++) {
		vkDestroyBuffer(_vkDevice, _vkUniformBuffers[i], nullptr);
		vkFreeMemory(_vkDevice, _vkUniformBuffersMemory[i], nullptr);
	}

	for (size_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; ++i) {
		vkDestroyBuffer(_vkDevice, _vkRtCameraBuffers[i], nullptr);
		vkFreeMemory(_vkDevice, _vkRtCameraBuffersMemory[i], nullptr);
	}


	for (size_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; ++i) {
		vkDestroyBuffer(_vkDevice, _vkRtSampleUniform[i], nullptr);
		vkFreeMemory(_vkDevice, _vkRtUniformBuffersMemory[i], nullptr);
	}


	vkDestroyDescriptorPool(_vkDevice, _vkDescriptorPool, nullptr);
	vkDestroyDescriptorPool(_vkDevice, _vkDescriptorPoolUI, nullptr);
	vkDestroyDescriptorSetLayout(_vkDevice, _vkDescriptorSetLayout, nullptr);

	vkDestroyBuffer(_vkDevice, _vkIndexBuffer, nullptr);
	vkFreeMemory(_vkDevice, _vkIndexBufferMemory, nullptr);

	vkDestroyBuffer(_vkDevice, _vkVertexBuffer, nullptr);
	vkFreeMemory(_vkDevice, _vkVertexBufferMemory, nullptr);

	for (size_t i = 0; i < _vkImageAvailableSemaphores.size(); ++i) {
		vkDestroySemaphore(_vkDevice, _vkImageAvailableSemaphores[i], nullptr);
	}
	for (size_t i = 0; i < _vkRenderFinishedSemaphores.size(); ++i) {
		vkDestroySemaphore(_vkDevice, _vkRenderFinishedSemaphores[i], nullptr);
	}
	for (size_t i = 0; i < _vkInFlightFences.size(); ++i) {
		vkDestroyFence(_vkDevice, _vkInFlightFences[i], nullptr);
	}


	vkDestroyCommandPool(_vkDevice, _vkCommandPool, nullptr);

	vkDestroyPipeline(_vkDevice, _vkGraphicsPipeline, nullptr);
	vkDestroyPipelineLayout(_vkDevice, _vkPipelineLayout, nullptr);
	vkDestroyRenderPass(_vkDevice, _vkRenderPass, nullptr);

	vkDestroyPipeline(_vkDevice, _vkRtGraphicsPipeline, nullptr);
	vkDestroyPipeline(_vkDevice, _vkRtPipeline, nullptr);
	vkDestroyBuffer(_vkDevice, _vkRtSbtBuffer, nullptr);
	vkFreeMemory(_vkDevice, _vkRtSbtMemory, nullptr);

	/* removed after the test buffer are not needed */
	if (_vkTestBlas != VK_NULL_HANDLE) {
		_vkDestroyAccelerationStructureKHR_PFN(_vkDevice, _vkTestBlas, nullptr);
		_vkTestBlas = VK_NULL_HANDLE;
	}
	if (_vkTestBlasBuffer != VK_NULL_HANDLE) {
		vkDestroyBuffer(_vkDevice, _vkTestBlasBuffer, nullptr);
		_vkTestBlasBuffer = VK_NULL_HANDLE;
	}
	if (_vkTestBlasMemory != VK_NULL_HANDLE) {
		vkFreeMemory(_vkDevice, _vkTestBlasMemory, nullptr);
		_vkTestBlasMemory = VK_NULL_HANDLE;
	}

	if (_vkTestTlas != VK_NULL_HANDLE) {
		_vkDestroyAccelerationStructureKHR_PFN(_vkDevice, _vkTestTlas, nullptr);
		_vkTestTlas = VK_NULL_HANDLE;
	}
	if (_vkTestTlasBuffer != VK_NULL_HANDLE) {
		vkDestroyBuffer(_vkDevice, _vkTestTlasBuffer, nullptr);
		_vkTestTlasBuffer = VK_NULL_HANDLE;
	}
	if (_vkTestTlasMemory != VK_NULL_HANDLE) {
		vkFreeMemory(_vkDevice, _vkTestTlasMemory, nullptr);
		_vkTestTlasMemory = VK_NULL_HANDLE;
	}



	// RT vertex/index buffers cleanup
	if (_vkRtVertexBuffer != VK_NULL_HANDLE) {
		vkDestroyBuffer(_vkDevice, _vkRtVertexBuffer, nullptr);
		_vkRtVertexBuffer = VK_NULL_HANDLE;
	}
	if (_vkRtVertexBufferMemory != VK_NULL_HANDLE) {
		vkFreeMemory(_vkDevice, _vkRtVertexBufferMemory, nullptr);
		_vkRtVertexBufferMemory = VK_NULL_HANDLE;
	}

	if (_vkRtIndexBuffer != VK_NULL_HANDLE) {
		vkDestroyBuffer(_vkDevice, _vkRtIndexBuffer, nullptr);
		_vkRtIndexBuffer = VK_NULL_HANDLE;
	}
	if (_vkRtIndexBufferMemory != VK_NULL_HANDLE) {
		vkFreeMemory(_vkDevice, _vkRtIndexBufferMemory, nullptr);
		_vkRtIndexBufferMemory = VK_NULL_HANDLE;
	}

	if (_vkMeshBlas != VK_NULL_HANDLE) {
		_vkDestroyAccelerationStructureKHR_PFN(_vkDevice, _vkMeshBlas, nullptr);
		_vkMeshBlas = VK_NULL_HANDLE;
	}
	if (_vkMeshBlasBuffer != VK_NULL_HANDLE) {
		vkDestroyBuffer(_vkDevice, _vkMeshBlasBuffer, nullptr);
		_vkMeshBlasBuffer = VK_NULL_HANDLE;
	}
	if (_vkMeshBlasMemory != VK_NULL_HANDLE) {
		vkFreeMemory(_vkDevice, _vkMeshBlasMemory, nullptr);
		_vkMeshBlasMemory = VK_NULL_HANDLE;
	}



	if (_vkMeshTlas != VK_NULL_HANDLE) {
		_vkDestroyAccelerationStructureKHR_PFN(_vkDevice, _vkMeshTlas, nullptr);
		_vkMeshTlas = VK_NULL_HANDLE;
	}
	if (_vkMeshTlasBuffer != VK_NULL_HANDLE) {
		vkDestroyBuffer(_vkDevice, _vkMeshTlasBuffer, nullptr);
		_vkMeshTlasBuffer = VK_NULL_HANDLE;
	}
	if (_vkMeshTlasMemory != VK_NULL_HANDLE) {
		vkFreeMemory(_vkDevice, _vkMeshTlasMemory, nullptr);
		_vkMeshTlasMemory = VK_NULL_HANDLE;
	}


	// cluster BLAS cleanup 
	for (auto& c : _rtClusterBlases)
	{
		if (c.blas != VK_NULL_HANDLE)
		{
			_vkDestroyAccelerationStructureKHR_PFN(_vkDevice, c.blas, nullptr);
			c.blas = VK_NULL_HANDLE;
		}
		if (c.blasBuffer != VK_NULL_HANDLE)
		{
			vkDestroyBuffer(_vkDevice, c.blasBuffer, nullptr);
			c.blasBuffer = VK_NULL_HANDLE;
		}
		if (c.blasMemory != VK_NULL_HANDLE)
		{
			vkFreeMemory(_vkDevice, c.blasMemory, nullptr);
			c.blasMemory = VK_NULL_HANDLE;
		}
	}
	_rtClusterBlases.clear();


	if (_vkClusterTlas != VK_NULL_HANDLE)
	{
		_vkDestroyAccelerationStructureKHR_PFN(_vkDevice, _vkClusterTlas, nullptr);
		_vkClusterTlas = VK_NULL_HANDLE;
	}
	if (_vkClusterTlasBuffer != VK_NULL_HANDLE)
	{
		vkDestroyBuffer(_vkDevice, _vkClusterTlasBuffer, nullptr);
		_vkClusterTlasBuffer = VK_NULL_HANDLE;
	}
	if (_vkClusterTlasMemory != VK_NULL_HANDLE)
	{
		vkFreeMemory(_vkDevice, _vkClusterTlasMemory, nullptr);
		_vkClusterTlasMemory = VK_NULL_HANDLE;
	}


	if (_vkRtInstanceBuffer != VK_NULL_HANDLE) {
		vkDestroyBuffer(_vkDevice, _vkRtInstanceBuffer, nullptr);
		_vkRtInstanceBuffer = VK_NULL_HANDLE;
	}
	if (_vkRtInstanceBufferMemory != VK_NULL_HANDLE) {
		vkFreeMemory(_vkDevice, _vkRtInstanceBufferMemory, nullptr);
		_vkRtInstanceBufferMemory = VK_NULL_HANDLE;
	}

	if (_enableValidationLayers) 
	{
		destroyDebugUtilsMessengerEXT(_vkInstance, _vkDebugMessender, nullptr);
	}

	/* cleaning up the indirect frames */
	for (auto& ind : _indirectPerFrame) {
		if (ind.mapped) {
			vkUnmapMemory(_vkDevice, ind.mem);
			ind.mapped = nullptr;
		}
		if (ind.buf) {
			vkDestroyBuffer(_vkDevice, ind.buf, nullptr);
			ind.buf = VK_NULL_HANDLE;
		}
		if (ind.mem) {
			vkFreeMemory(_vkDevice, ind.mem, nullptr);
			ind.mem = VK_NULL_HANDLE;
		}
	}

	for (uint32_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; ++i) {
		if (_perDraw[i].mapped) vkUnmapMemory(_vkDevice, _perDraw[i].mem);
		if (_perDraw[i].ssbo)   vkDestroyBuffer(_vkDevice, _perDraw[i].ssbo, nullptr);
		if (_perDraw[i].mem)    vkFreeMemory(_vkDevice, _perDraw[i].mem, nullptr);
	}

	_indirectPerFrame.clear();

	if (_vkGpuTimestampQueryPool != VK_NULL_HANDLE)
	{
		vkDestroyQueryPool(_vkDevice, _vkGpuTimestampQueryPool, nullptr);
		_vkGpuTimestampQueryPool = VK_NULL_HANDLE;
	}

	vkDestroySurfaceKHR(_vkInstance, _vkSurface, nullptr);
	vkDestroyDevice(_vkDevice, nullptr);
	vkDestroyInstance(_vkInstance, nullptr);
	glfwDestroyWindow(_GLFwindow);
	glfwTerminate();
}

void VulkanRenderer::waitIdle()
{
	// wait until the device is done with it work before clean up
	vkDeviceWaitIdle(_vkDevice);
}

void VulkanRenderer::cleanUpSwapChain()
{
	// Destroy RT color resources first (order not super critical, but keep it tidy)
	vkDestroyImageView(_vkDevice, _vkRtColorImageView, nullptr);
	vkDestroyImage(_vkDevice, _vkRtColorImage, nullptr);
	vkFreeMemory(_vkDevice, _vkRtColorImageMemory, nullptr);

	vkDestroyImageView(_vkDevice, _vkColorImageView, nullptr);
	vkDestroyImage(_vkDevice, _vkColorImage, nullptr);
	vkFreeMemory(_vkDevice, _vkColorImageMemory, nullptr);

	vkDestroyImageView(_vkDevice, _vkDepthImageView, nullptr);
	vkDestroyImage(_vkDevice, _vkDepthImage, nullptr);
	vkFreeMemory(_vkDevice, _vkDepthImageMemory, nullptr);


	for (size_t i = 0; i < _vkSwapChainFramebuffers.size(); i++) {
		vkDestroyFramebuffer(_vkDevice, _vkSwapChainFramebuffers[i], nullptr);
	}

	for (size_t i = 0; i < _vkSwapChainImageViews.size(); i++) {
		vkDestroyImageView(_vkDevice, _vkSwapChainImageViews[i], nullptr);
	}

	vkDestroySwapchainKHR(_vkDevice, _vkSwapChain, nullptr);
}

void VulkanRenderer::destroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator) {
	auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
	if (func != nullptr) {
		func(instance, debugMessenger, pAllocator);
	}
}

void VulkanRenderer::initWindow()
{
	if (glfwInit() == GLFW_FALSE)
		throw std::runtime_error("ERROR::GLFW::GLFW_INIT::FAILED_TO_INITIALIZE_GLFW\n");

	glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

	/*disable window resize*/
	//glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

	/* enable window resize*/
	glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

	/* create a glfw window */
	_GLFwindow = glfwCreateWindow(_WINDOW_WIDTH, _WINDOW_HEIGHT, _applicationName, nullptr, nullptr);
	glfwSetWindowUserPointer(_GLFwindow, this);
	glfwSetFramebufferSizeCallback(_GLFwindow, framebufferResizeCallback);
}

bool VulkanRenderer::shouldCloseWindow()
{
	return glfwWindowShouldClose(_GLFwindow);
}

void VulkanRenderer::submitDrawList(const std::vector<engine::mesh::ClusterDrawRange>& list, int renderMode) {
	
	/* per cluser colouring draw */
	auto& ind = _indirectPerFrame[_currentFrame];
	const uint32_t n = (uint32_t)list.size();
	auto* indir = reinterpret_cast<VkDrawIndexedIndirectCommand*>(ind.mapped);
	auto* perDraw = reinterpret_cast<engine::vk::PerDrawGPU*>(_perDraw[_currentFrame].mapped);

	for (uint32_t i = 0; i < n; ++i) {
		const auto& drawRange = list[i];

		// indirect command
		VkDrawIndexedIndirectCommand cmd{};
		cmd.indexCount = drawRange.indexCount;
		cmd.instanceCount = 1;
		cmd.firstIndex = drawRange.firstIndex;
		cmd.vertexOffset = 0;
		cmd.firstInstance = i; // matches SSBO index
		indir[i] = cmd;

		// SSBO record
		engine::vk::PerDrawGPU rec{};
		rec.clusterId = drawRange.clusterId;
		rec.pad0 = renderMode;
		rec.lod = (uint32_t)drawRange.level;

		// colouring the each cluster
		uint32_t h = rec.clusterId;
		rec.color[0] = (((h * 97) % 231) + 24) / 255.0f;
		rec.color[1] = (((h * 57) % 231) + 24) / 255.0f;
		rec.color[2] = (((h * 33) % 231) + 24) / 255.0f;
		rec.color[3] = 1.0f;

		perDraw[i] = rec;
	}
	ind.count = n;
}

void VulkanRenderer::draw(bool drawScene)
{
	drawFrame(drawScene);
}

void VulkanRenderer::pollWindowInputEvents()
{
	glfwPollEvents();
}

void VulkanRenderer::drawFrame(bool drawScene)
{
	vkWaitForFences(_vkDevice, 1, &_vkInFlightFences[_currentFrame], VK_TRUE, UINT64_MAX);

	if (_vkGpuTimestampQueryPool != VK_NULL_HANDLE && _vkTimestampPeriod > 0.0f && _gpuTimestampReady[_currentFrame])
	{
		uint32_t queryBase = _currentFrame * 2;
		uint64_t timestamps[2] = {};

		VkResult qr = vkGetQueryPoolResults(
			_vkDevice,
			_vkGpuTimestampQueryPool,
			queryBase,
			2,                             
			sizeof(timestamps),
			timestamps,
			sizeof(uint64_t),
			VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT
		);

		if (qr == VK_SUCCESS)
		{
			uint64_t dtTicks = timestamps[1] - timestamps[0];
			double dtNs = double(dtTicks) * double(_vkTimestampPeriod);
			_gpuLastFrameMs = dtNs / 1e6;  // nanoseconds -> milliseconds
		}
	}

	uint32_t imageIndex;
	VkResult vkAcquireNextImageResult = vkAcquireNextImageKHR(
		_vkDevice,
		_vkSwapChain,
		UINT64_MAX,
		_vkImageAvailableSemaphores[_currentFrame],  
		VK_NULL_HANDLE,
		&imageIndex);

	if (vkAcquireNextImageResult == VK_ERROR_OUT_OF_DATE_KHR) {
		recreateSwapChain();
		return;
	}
	else if (vkAcquireNextImageResult != VK_SUCCESS && vkAcquireNextImageResult != VK_SUBOPTIMAL_KHR) {
		throw std::runtime_error("ERROR::VULKAN::DRAW_CALL::FAILED_TO_ACQUIRE_SWAPCHAIN_IMAGE\n");
	}

	if (_vkImagesInFlight[imageIndex] != VK_NULL_HANDLE) {
		vkWaitForFences(_vkDevice, 1, &_vkImagesInFlight[imageIndex], VK_TRUE, UINT64_MAX);
	}
	_vkImagesInFlight[imageIndex] = _vkInFlightFences[_currentFrame];

	vkResetFences(_vkDevice, 1, &_vkInFlightFences[_currentFrame]);

	vkResetCommandBuffer(_vkCommandBuffers[_currentFrame], 0);
	recordCommandBuffer(_vkCommandBuffers[_currentFrame], imageIndex, drawScene);

	_gpuTimestampReady[_currentFrame] = true;

	if (drawScene)
		updateUniformBuffer(_currentFrame);

	VkSemaphore vkWaitSemaphores[] = { _vkImageAvailableSemaphores[_currentFrame] };
	VkSemaphore vkSignalSemaphores[] = { _vkRenderFinishedSemaphores[imageIndex] };
	VkPipelineStageFlags vkWaitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };

	VkSubmitInfo vkSubmitInfo{};
	vkSubmitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	vkSubmitInfo.waitSemaphoreCount = 1;
	vkSubmitInfo.pWaitSemaphores = vkWaitSemaphores;
	vkSubmitInfo.pWaitDstStageMask = vkWaitStages;
	vkSubmitInfo.commandBufferCount = 1;
	vkSubmitInfo.pCommandBuffers = &_vkCommandBuffers[_currentFrame];
	vkSubmitInfo.signalSemaphoreCount = 1;
	vkSubmitInfo.pSignalSemaphores = vkSignalSemaphores;

	VkResult submitRes = vkQueueSubmit(
		_vkGraphicsQueue,
		1,
		&vkSubmitInfo,
		_vkInFlightFences[_currentFrame]
	);

	if (submitRes != VK_SUCCESS) {
		std::cerr << "vkQueueSubmit (drawFrame) failed with VkResult = " << submitRes << std::endl;

		switch (submitRes) {
		case VK_ERROR_DEVICE_LOST:
			std::cerr << " -> VK_ERROR_DEVICE_LOST\n";
			break;
		case VK_ERROR_OUT_OF_DEVICE_MEMORY:
			std::cerr << " -> VK_ERROR_OUT_OF_DEVICE_MEMORY\n";
			break;
		case VK_ERROR_OUT_OF_HOST_MEMORY:
			std::cerr << " -> VK_ERROR_OUT_OF_HOST_MEMORY\n";
			break;
		default:
			break;
		}
		throw std::runtime_error("ERROR::VULKAN::DRAW_CALL::COMMAND_BUFFER::FAILED_TO_SUBMIT_COMMAND_BUFFER");
	}

	VkPresentInfoKHR vkPresentInfo{};
	vkPresentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
	vkPresentInfo.waitSemaphoreCount = 1;
	vkPresentInfo.pWaitSemaphores = vkSignalSemaphores;

	VkSwapchainKHR vkSwapChains[] = { _vkSwapChain };
	vkPresentInfo.swapchainCount = 1;
	vkPresentInfo.pSwapchains = vkSwapChains;
	vkPresentInfo.pImageIndices = &imageIndex;

	VkResult vkQueuePresentResult = vkQueuePresentKHR(_vkPresentationQueue, &vkPresentInfo);
	if (vkQueuePresentResult == VK_ERROR_OUT_OF_DATE_KHR || vkQueuePresentResult == VK_SUBOPTIMAL_KHR) {
		_framebufferResized = false;
		recreateSwapChain();
	}
	else if (vkQueuePresentResult != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::DRAW_CALL::FAILED_TO_PRESENT_SWAPCHAIN_IMAGE\n");
	}
	_currentFrame = (_currentFrame + 1) % _MAX_FRAMES_IN_FLIGHT;
}

/*
	This function will be called when window surface is no longer compatible.
	Such as in an event where the windw size is changing.
*/
void VulkanRenderer::recreateSwapChain()
{
	int width = 0, height = 0;
	glfwGetFramebufferSize(_GLFwindow, &width, &height);

	while (width == 0 || height == 0) {
		glfwGetFramebufferSize(_GLFwindow, &width, &height);
		glfwWaitEvents();
	}

	vkDeviceWaitIdle(_vkDevice);

	cleanUpSwapChain();
	createSwapChain();
	createImageViews();
	createColorResources();
	createDepthResources();
	createRtColourResources();
	createFramebuffers();

	//swapchain image count may have changed
	_vkImagesInFlight.assign(_vkSwapChainImages.size(), VK_NULL_HANDLE);
}

void VulkanRenderer::recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex, bool drawScene)
{
	VkCommandBufferBeginInfo vkBeginInfo{};
	vkBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	vkBeginInfo.flags = 0; // Optional
	vkBeginInfo.pInheritanceInfo = nullptr; // Optional

	if (vkBeginCommandBuffer(commandBuffer, &vkBeginInfo) != VK_SUCCESS)
	{
		throw std::runtime_error("ERROR::VULKAN::COMMANDBUFFER_BEGIN_INFO::FAILED_TO_CREATE_COMMANDBUFFER_BEGIN_INFO\n");
	}

	if (_vkGpuTimestampQueryPool != VK_NULL_HANDLE)
	{
		uint32_t queryBase = _currentFrame * 2; 
		vkCmdResetQueryPool(
			commandBuffer,
			_vkGpuTimestampQueryPool,
			queryBase,
			2   
		);

		vkCmdWriteTimestamp(
			commandBuffer,
			VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
			_vkGpuTimestampQueryPool,
			queryBase + 0
		);
	}

	const VkImage swapImg = _vkSwapChainImages[imageIndex];
	VkImageLayout old = _swapchainLayouts[imageIndex];

	VkImageMemoryBarrier toColor{};
	toColor.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	toColor.image = swapImg;
	toColor.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
	toColor.oldLayout = old;
	toColor.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	toColor.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	toColor.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

	// Stages/masks based on old layout:
	VkPipelineStageFlags srcStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;

	toColor.srcAccessMask = 0;
	if (old == VK_IMAGE_LAYOUT_PRESENT_SRC_KHR) {
		srcStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		toColor.srcAccessMask = 0;
	}

	VkPipelineStageFlags dstStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	toColor.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

	vkCmdPipelineBarrier(commandBuffer,
		srcStage, dstStage,
		0, 0, nullptr, 0, nullptr, 1, &toColor);

	_swapchainLayouts[imageIndex] = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;


	if(drawScene)
	{
		// 1) Trace rays to fill _vkRtColorImage
		vkCmdBindPipeline(
			commandBuffer,
			VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
			_vkRtPipeline);

		// same descriptor set as graphics: set=0 with rtImage at binding 3
		vkCmdBindDescriptorSets(
			commandBuffer,
			VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
			_vkPipelineLayout,
			0,
			1,
			&_vkDescriptorSets[_currentFrame],
			0,
			nullptr);

		// launch raygen over the whole viewport
		_vkRtCmdTraceRaysKHR_PFN(
			commandBuffer,
			&_vkSbtRaygenRegion,    // raygen SBT
			&_vkSbtMissRegion,      // miss SBT
			&_vkSbtHitRegion,       // hit SBT (currently empty)
			&_vkSbtCallableRegion,  // callable SBT (empty)
			_vkSwapChainExtent.width,
			_vkSwapChainExtent.height,
			1);

		// 2) Barrier: make rt image writes visible to the fullscreen fragment shader
		VkImageMemoryBarrier rtBarrier{};
		rtBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		rtBarrier.image = _vkRtColorImage;
		rtBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		rtBarrier.subresourceRange.baseMipLevel = 0;
		rtBarrier.subresourceRange.levelCount = 1;
		rtBarrier.subresourceRange.baseArrayLayer = 0;
		rtBarrier.subresourceRange.layerCount = 1;
		rtBarrier.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
		rtBarrier.newLayout = VK_IMAGE_LAYOUT_GENERAL; // keep as GENERAL
		rtBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		rtBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		rtBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
		rtBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

		vkCmdPipelineBarrier(
			commandBuffer,
			VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR,
			VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
			0,
			0, nullptr,
			0, nullptr,
			1, &rtBarrier);

		// 3) Fullscreen triangle that samples rtImage and writes to swapchain
		VkRenderingAttachmentInfo colorAtt{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO_KHR };
		colorAtt.imageView = _vkSwapChainImageViews[imageIndex];
		colorAtt.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		colorAtt.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		colorAtt.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		VkClearValue clearSwap{};
		clearSwap.color = { {0.f, 0.f, 0.f, 1.f} };
		colorAtt.clearValue = clearSwap;

		VkRenderingInfo renderingInfo{ VK_STRUCTURE_TYPE_RENDERING_INFO_KHR };
		renderingInfo.renderArea = { {0,0}, _vkSwapChainExtent };
		renderingInfo.layerCount = 1;
		renderingInfo.colorAttachmentCount = 1;
		renderingInfo.pColorAttachments = &colorAtt;
		renderingInfo.pDepthAttachment = nullptr; // no depth

		vkCmdBeginRendering(commandBuffer, &renderingInfo);

		// viewport & scissor
		VkViewport vp{};
		vp.x = 0.0f;
		vp.y = 0.0f;
		vp.width = static_cast<float>(_vkSwapChainExtent.width);
		vp.height = static_cast<float>(_vkSwapChainExtent.height);
		vp.minDepth = 0.0f;
		vp.maxDepth = 1.0f;
		vkCmdSetViewport(commandBuffer, 0, 1, &vp);

		VkRect2D sc{};
		sc.offset = { 0, 0 };
		sc.extent = _vkSwapChainExtent;
		vkCmdSetScissor(commandBuffer, 0, 1, &sc);

		// bind RT fullscreen graphics pipeline
		vkCmdBindPipeline(
			commandBuffer,
			VK_PIPELINE_BIND_POINT_GRAPHICS,
			_vkRtGraphicsPipeline);

		// same descriptor set: fragment shader reads rtImage at binding=3
		vkCmdBindDescriptorSets(
			commandBuffer,
			VK_PIPELINE_BIND_POINT_GRAPHICS,
			_vkPipelineLayout,
			0,
			1,
			&_vkDescriptorSets[_currentFrame],
			0,
			nullptr);

		// fullscreen triangle (3 vertices, no vertex buffer)
		vkCmdDraw(commandBuffer, 3, 1, 0, 0);

		vkCmdEndRendering(commandBuffer);

		// 4) Barrier between scene and UI (unchanged)
		VkImageMemoryBarrier between{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
		between.image = swapImg;
		between.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
		between.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		between.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		between.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		between.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
		VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		between.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		between.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

		vkCmdPipelineBarrier(
			commandBuffer,
			VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
			VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
			0,
			0, nullptr,
			0, nullptr,
			1, &between);
	}

	// ============= UI Pass ================
	VkRenderingAttachmentInfo uiColorAtt{ VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO_KHR };
	uiColorAtt.imageView = _vkSwapChainImageViews[imageIndex];
	uiColorAtt.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	uiColorAtt.loadOp =  drawScene ?  VK_ATTACHMENT_LOAD_OP_LOAD : VK_ATTACHMENT_LOAD_OP_CLEAR;
	uiColorAtt.storeOp = VK_ATTACHMENT_STORE_OP_STORE; // present reads it
	uiColorAtt.resolveMode = VK_RESOLVE_MODE_NONE;

	VkRenderingInfo uiRenderInfo{ VK_STRUCTURE_TYPE_RENDERING_INFO_KHR };
	uiRenderInfo.renderArea = { {0,0}, _vkSwapChainExtent };
	uiRenderInfo.layerCount = 1;
	uiRenderInfo.colorAttachmentCount = 1;
	uiRenderInfo.pColorAttachments = &uiColorAtt;

	vkCmdBeginRendering(commandBuffer, &uiRenderInfo);

	VkViewport vp{ 0,0,(float)_vkSwapChainExtent.width,(float)_vkSwapChainExtent.height,0.f,1.f };
	vkCmdSetViewport(commandBuffer, 0, 1, &vp);

	VkRect2D sc{ {0,0}, _vkSwapChainExtent };
	vkCmdSetScissor(commandBuffer, 0, 1, &sc);

	callBackUI(commandBuffer);  // ImGui_ImplVulkan_RenderDrawData(drawData, cmd)

	vkCmdEndRendering(commandBuffer);

	// ============= END UI Pass ============



	// ---- COLOR_ATTACHMENT_OPTIMAL -> PRESENT ----
	// adding this for dynamic rendering
	VkImageMemoryBarrier toPresent{};
	toPresent.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	toPresent.image = swapImg;
	toPresent.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
	toPresent.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	toPresent.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
	toPresent.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	toPresent.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	toPresent.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
	toPresent.dstAccessMask = 0;

	vkCmdPipelineBarrier(commandBuffer,
		VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
		VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
		0, 0, nullptr, 0, nullptr, 1, &toPresent);

	// >>> Layout tracking: back to PRESENT
	_swapchainLayouts[imageIndex] = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

	if (_vkGpuTimestampQueryPool != VK_NULL_HANDLE)
	{
		uint32_t queryBase = _currentFrame * 2;
		vkCmdWriteTimestamp(
			commandBuffer,
			VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
			_vkGpuTimestampQueryPool,
			queryBase + 1
		);
	}

	if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::RECORD_COMMAND::FAILED_TO_RECORD_COMMAND_BUFFER");
	}
}

void VulkanRenderer::updateUniformBuffer(uint32_t currentImage)
{
	memcpy(_vkUniformBuffersMapped[currentImage], _mat4Uniform, sizeof(*_mat4Uniform));

	/* RT Camera */
	memcpy(_vkRtCameraBuffersMapped[currentImage],
		&_rtCameraHost,
		sizeof(engine::vk::RtCameraUBO));

	/* RT Ray Samples */
	memcpy(_vkRtUniformBuffersMapped[currentImage],
		_rtSamples,
		sizeof(engine::vk::RtSamples));

}

void VulkanRenderer::updateSSBO(uint32_t currentImage)
{
	memcpy(_perDraw[currentImage].mapped, _frameDrawItems.data(),
		_frameDrawItems.size() * sizeof(engine::vk::DrawItem));
}

void VulkanRenderer::submitUniform(engine::math::Mat4f* mat4, engine::vk::RtSamples* rtSamples )
{
	_mat4Uniform = mat4;
	_rtSamples = rtSamples;
}

void VulkanRenderer::initVk()
{
	createVkInstance();
	setUpVkDebugMessenger();
	createSurface();
	selectPhysicalDevice();

	createLogicalDevice();
	createSwapChain();
	createImageViews();


	createDescriptorSetLayout();
	createGraphicsPipeline();

	createRtGraphicsPipeline(); // RT
	createRayTracingPipeline(); // RT
	createRayTracingSBT(); // RT


	createCommandPool();
	createColorResources(); 
	createDepthResources();

	createRtColourResources(); // RT

	createIndirectBuffers(); 

	createUniformBuffers();

	createRtCameraBuffers(); // RT cam buffer

	createSSBO();

	createDescriptorPool();
	createUIDescriptorPool();

	createDescriptorSets();

	createCommandBuffers();
	createSyncObjects();

	createGpuTimestampQueryPool();
}

void VulkanRenderer::createGpuTimestampQueryPool()
{
	VkQueryPoolCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
	info.queryType = VK_QUERY_TYPE_TIMESTAMP;
	info.queryCount = 2 * _MAX_FRAMES_IN_FLIGHT; // begin + end per frame slot

	if (vkCreateQueryPool(_vkDevice, &info, nullptr, &_vkGpuTimestampQueryPool) != VK_SUCCESS)
	{
		throw std::runtime_error("ERROR::VULKAN::GPU_TIMING::FAILED_TO_CREATE_QUERY_POOL\n");
	}

}

void VulkanRenderer::initImGUI_Info(ImGui_ImplVulkan_InitInfo* info)
{
	info->Instance = _vkInstance;
	info->PhysicalDevice = _vkPhysicalDevice;
	info->Device = _vkDevice;
	info->QueueFamily = _vkQueueFamilyIdx;
	info->Queue = _vkGraphicsQueue;
	//info->PipelineCache = g_PipelineCache;	//optional
	info->DescriptorPool = _vkDescriptorPoolUI;
	info->UseDynamicRendering = true;
	//info->RenderPass = wd->RenderPass;		//ignored since dynamic rendering
	//info->Subpass = 0;						//optional
	info->MinImageCount = (uint32_t)_vkSwapChainImages.size();
	info->ImageCount = (uint32_t)_vkSwapChainImages.size();
	info->MSAASamples = VK_SAMPLE_COUNT_1_BIT;

	info->PipelineRenderingCreateInfo = {};
	info->PipelineRenderingCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO_KHR;
	info->PipelineRenderingCreateInfo.colorAttachmentCount = 1;
	info->PipelineRenderingCreateInfo.pColorAttachmentFormats = &_vkSwapChainImageFormat;
	info->PipelineRenderingCreateInfo.depthAttachmentFormat = VK_FORMAT_UNDEFINED;
	info->PipelineRenderingCreateInfo.stencilAttachmentFormat = VK_FORMAT_UNDEFINED;

	//info->Allocator = g_Allocator;	 //optional
	//info->CheckVkResultFn = check_vk_result;	//optional
}

void VulkanRenderer::createVkInstance()
{
	VkApplicationInfo vkAppInfo{};
	vkAppInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	vkAppInfo.pApplicationName = _applicationName;
	vkAppInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
	vkAppInfo.pEngineName = "Virtual Geometry Renderer";
	vkAppInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
	vkAppInfo.apiVersion = VK_API_VERSION_1_4;


	VkInstanceCreateInfo vkInstanceCreateInfo{};
	vkInstanceCreateInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	vkInstanceCreateInfo.pApplicationInfo = &vkAppInfo;


	/* validation layer */
	if (_enableValidationLayers && !checkValidationLayerSupport())
	{
		throw std::runtime_error("ERROR::VULKAN::VALIDATION_LAYER_REQUESTED::VALIDATION_LAYER_NOT_FOUND\n");
	}


	VkDebugUtilsMessengerCreateInfoEXT vkDebugMessengerCreateInfo{};

	if (_enableValidationLayers)
	{
		vkInstanceCreateInfo.enabledLayerCount = static_cast<uint32_t>(_validationLayers.size());
		vkInstanceCreateInfo.ppEnabledLayerNames = _validationLayers.data();
		populateDebugMessengerCreateInfo(vkDebugMessengerCreateInfo);
		vkInstanceCreateInfo.pNext = (VkDebugUtilsMessengerCreateInfoEXT*)&vkDebugMessengerCreateInfo;
	}
	else
	{
		vkInstanceCreateInfo.enabledLayerCount = 0;
		vkInstanceCreateInfo.pNext = nullptr;
	}

	/* adding extension for glfw (message call back )*/
	std::vector<const char*> extensions = getRequiredInstanceExtensions();
	vkInstanceCreateInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
	vkInstanceCreateInfo.ppEnabledExtensionNames = extensions.data();

	VkResult vkResult = vkCreateInstance(&vkInstanceCreateInfo, nullptr, &_vkInstance);

	/* could be extended */
	if (vkResult != VK_SUCCESS)
		throw std::runtime_error("ERROR::VULKAN::CREATE::VULKAN_INSTANCE_CREATION_FAILED\n");

}

bool VulkanRenderer::checkValidationLayerSupport()
{
	uint32_t layerCount;
	vkEnumerateInstanceLayerProperties(&layerCount, nullptr);
	std::vector<VkLayerProperties> availableLayers(layerCount);
	vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());


	for (const char* layerName : _validationLayers)
	{
		bool validationSupportFound = false;

		for (const VkLayerProperties& layerPropertie : availableLayers)
		{
			if (strcmp(layerName, layerPropertie.layerName) == 0)
			{
				validationSupportFound = true;
				return validationSupportFound;
			}
		}

	}

	return false;

}

void VulkanRenderer::populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo)
{
	createInfo = {};
	createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
	createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
	createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
	createInfo.pfnUserCallback = debugCallback;
}

VKAPI_ATTR VkBool32 VKAPI_CALL VulkanRenderer::debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity
	, VkDebugUtilsMessageTypeFlagsEXT messageType, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData)
{
	std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;
	return VK_FALSE;
}

void VulkanRenderer::framebufferResizeCallback(GLFWwindow* window, int width, int height)
{
	VulkanRenderer* renderer = reinterpret_cast<VulkanRenderer*>(glfwGetWindowUserPointer(window));
	renderer->_framebufferResized = true;
};

std::vector<const char*> VulkanRenderer::getRequiredInstanceExtensions()
{
	uint32_t extensionCount = 0;
	const char** glfwExtensions;

	glfwExtensions = glfwGetRequiredInstanceExtensions(&extensionCount);

	std::vector<const char*> extensions(glfwExtensions, glfwExtensions + extensionCount);

	if (_enableValidationLayers)
		extensions.emplace_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);

	return extensions;
}

void VulkanRenderer::setUpVkDebugMessenger()
{
	if (!_enableValidationLayers)
		return;

	VkDebugUtilsMessengerCreateInfoEXT vkDebugMessengerCreateInfo{};
	populateDebugMessengerCreateInfo(vkDebugMessengerCreateInfo);

	if (createDebugUtilsMessengerEXT(_vkInstance, &vkDebugMessengerCreateInfo, nullptr, &_vkDebugMessender) != VK_SUCCESS)
		throw std::runtime_error("ERROR::VULKAN::DEBUG::FAILED_TO_CREATE_DEBUG_MESSENGER\n");
}

VkResult VulkanRenderer::createDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo,
	const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pDebugMessenger)
{
	PFN_vkCreateDebugUtilsMessengerEXT func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
	if (func != nullptr) {
		return func(instance, pCreateInfo, pAllocator, pDebugMessenger);
	}
	else {
		return VK_ERROR_EXTENSION_NOT_PRESENT;
	}
}

void VulkanRenderer::createSurface()
{
	if (glfwCreateWindowSurface(_vkInstance, _GLFwindow, nullptr, &_vkSurface) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::GLFW::FIALED_TO_CREATE_A_SURFACE\n");
	}
}

void VulkanRenderer::selectPhysicalDevice()
{
	uint32_t deviceCount = 0;
	vkEnumeratePhysicalDevices(_vkInstance, &deviceCount, nullptr);
	if (deviceCount == 0)
		throw std::runtime_error("ERROR::VULKAN::PHYSICAL_DEVICE::PHYSICAL_DEVICE_DOES_NOT_SUPPORTS_VULKAN\n");
	std::vector<VkPhysicalDevice> physicalDivices(deviceCount);
	vkEnumeratePhysicalDevices(_vkInstance, &deviceCount, physicalDivices.data());

	for (VkPhysicalDevice physicalDevice : physicalDivices)
	{
		if (checkPhysicalDevices(physicalDevice))
		{
			_vkPhysicalDevice = physicalDevice;
			_vkMsaaSamples = getMaxUsableSampleCount();
			
			VkPhysicalDeviceProperties vkDevprops{};
			vkGetPhysicalDeviceProperties(_vkPhysicalDevice, &vkDevprops);
			_vkTimestampPeriod = vkDevprops.limits.timestampPeriod; 


			queryRayTracingProperties();


		}
	}

	if (_vkPhysicalDevice == VK_NULL_HANDLE)
		throw std::runtime_error("ERROR::VULKAN::PHYSICAL_DEVICE::DEVICE_DOES_NOT_MEET_THE_REQUIRMENT\n");
}

bool VulkanRenderer::checkPhysicalDevices(VkPhysicalDevice& physicalDevice)
{
	VkPhysicalDeviceProperties deviceProperties;
	VkPhysicalDeviceFeatures deviceFeatures;
	vkGetPhysicalDeviceProperties(physicalDevice, &deviceProperties);
	vkGetPhysicalDeviceFeatures(physicalDevice, &deviceFeatures);

	engine::vk::QueueFamily queueFamily = getQueueFamilies(physicalDevice);

	bool supportsGivenExtensions = checkDeviceExtensionSupport(physicalDevice);
	bool isSwapChainAdequate = false;

	if (supportsGivenExtensions) {
		engine::vk::SurfaceSupportDetails surfaceSupport = querySwapChainSupport(physicalDevice);
		isSwapChainAdequate = !surfaceSupport.formats.empty() && !surfaceSupport.presentModes.empty();
	}
	std::cout << "INFO::CHEKC_PHYSICAL_DEVICE::DEVICE_INFO::" << deviceProperties.deviceName << "::";

	if (deviceProperties.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU)
		std::cout << "VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU\n";

	if (deviceProperties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU)
		std::cout << "VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU\n";

	return (deviceProperties.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) || (deviceProperties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) &&
		deviceFeatures.geometryShader && queueFamily.isComplete() && supportsGivenExtensions && isSwapChainAdequate
		&& deviceFeatures.samplerAnisotropy;
}

engine::vk::QueueFamily VulkanRenderer::getQueueFamilies(VkPhysicalDevice physicalDevice)
{
	engine::vk::QueueFamily queueFamily;

	uint32_t queueFamilyCount = 0;

	vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, nullptr);
	std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
	vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, queueFamilies.data());

	int atIndex = 0;
	for (VkQueueFamilyProperties& queueFamilyProperty : queueFamilies)
	{
		if (queueFamilyProperty.queueFlags & VK_QUEUE_GRAPHICS_BIT)
			queueFamily.graphicsFamily = atIndex;

		VkBool32 presentSupport = false;
		vkGetPhysicalDeviceSurfaceSupportKHR(physicalDevice, atIndex, _vkSurface, &presentSupport);
		if (presentSupport)
			queueFamily.presentFamily = atIndex;

		if (queueFamily.isComplete())
			break;

		atIndex++;

	}
	_vkQueueFamilyIdx = atIndex;

	return queueFamily;
}

bool VulkanRenderer::checkDeviceExtensionSupport(VkPhysicalDevice physicalDevice)
{
	uint32_t extensionCount;
	vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &extensionCount, nullptr);
	std::vector<VkExtensionProperties> availableExtensions(extensionCount);
	vkEnumerateDeviceExtensionProperties(physicalDevice, nullptr, &extensionCount, availableExtensions.data());

	std::set<std::string> requiredExtensions(_requiredDeviceExtensions.begin(), _requiredDeviceExtensions.end());

	for (const VkExtensionProperties& extension : availableExtensions) {

		//std::cout << extension.extensionName << " " << extension.specVersion << std::endl;
		requiredExtensions.erase(extension.extensionName);
	}
	return requiredExtensions.empty();
}

engine::vk::SurfaceSupportDetails VulkanRenderer::querySwapChainSupport(VkPhysicalDevice physicalDevice)
{
	engine::vk::SurfaceSupportDetails surfaceDetails;
	vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, _vkSurface, &surfaceDetails.capabilities);

	uint32_t formatCount;
	vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, _vkSurface, &formatCount, nullptr);

	if (formatCount != 0) {
		surfaceDetails.formats.resize(formatCount);
		vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, _vkSurface, &formatCount, surfaceDetails.formats.data());
	}

	uint32_t presentModeCount;
	vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, _vkSurface, &presentModeCount, nullptr);

	if (presentModeCount != 0) {
		surfaceDetails.presentModes.resize(presentModeCount);
		vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, _vkSurface, &presentModeCount, surfaceDetails.presentModes.data());
	}

	return surfaceDetails;
}

VkSampleCountFlagBits VulkanRenderer::getMaxUsableSampleCount()
{
	VkPhysicalDeviceProperties physicalDeviceProperties;
	vkGetPhysicalDeviceProperties(_vkPhysicalDevice, &physicalDeviceProperties);

	VkSampleCountFlags counts = physicalDeviceProperties.limits.framebufferColorSampleCounts & physicalDeviceProperties.limits.framebufferDepthSampleCounts;
	if (counts & VK_SAMPLE_COUNT_64_BIT) { return VK_SAMPLE_COUNT_64_BIT; }
	if (counts & VK_SAMPLE_COUNT_32_BIT) { return VK_SAMPLE_COUNT_32_BIT; }
	if (counts & VK_SAMPLE_COUNT_16_BIT) { return VK_SAMPLE_COUNT_16_BIT; }
	if (counts & VK_SAMPLE_COUNT_8_BIT) { return VK_SAMPLE_COUNT_8_BIT; }
	if (counts & VK_SAMPLE_COUNT_4_BIT) { return VK_SAMPLE_COUNT_4_BIT; }
	if (counts & VK_SAMPLE_COUNT_2_BIT) { return VK_SAMPLE_COUNT_2_BIT; }

	return VK_SAMPLE_COUNT_1_BIT;
}

void VulkanRenderer::createLogicalDevice()
{
	engine::vk::QueueFamily queueFamily = getQueueFamilies(_vkPhysicalDevice);
	float queuePriority = 1.0f;

	VkPhysicalDeviceShaderDrawParametersFeatures drawFeat{};
	drawFeat.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_DRAW_PARAMETERS_FEATURES;
	drawFeat.shaderDrawParameters = VK_TRUE;

	// adding RT pipeline device feature
	VkPhysicalDeviceRayTracingPipelineFeaturesKHR vkRTPipelineFeature{};
	vkRTPipelineFeature.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR;
	vkRTPipelineFeature.rayTracingPipeline = VK_TRUE;

	VkPhysicalDeviceAccelerationStructureFeaturesKHR vkAcStructFeature{};
	vkAcStructFeature.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
	vkAcStructFeature.accelerationStructure = VK_TRUE;
	vkAcStructFeature.pNext = &vkRTPipelineFeature;

	VkPhysicalDeviceBufferDeviceAddressFeaturesKHR vkBufferDeviceAddressFeature{};
	vkBufferDeviceAddressFeature.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES_KHR;
	vkBufferDeviceAddressFeature.bufferDeviceAddress = VK_TRUE;
	vkBufferDeviceAddressFeature.pNext = &vkAcStructFeature;

	drawFeat.pNext = &vkBufferDeviceAddressFeature; 

	VkPhysicalDeviceFeatures vkPhysicalDeviceFeatures{};
	vkPhysicalDeviceFeatures.samplerAnisotropy = VK_FALSE;
	/*  enable sample shading feature for the device ( cost more performance ! )*/
	//vkPhysicalDeviceFeatures.sampleRateShading = VK_FALSE;
	vkPhysicalDeviceFeatures.sampleRateShading = VK_TRUE;
	vkPhysicalDeviceFeatures.geometryShader = VK_TRUE;
	vkPhysicalDeviceFeatures.multiDrawIndirect = VK_TRUE;
	vkPhysicalDeviceFeatures.drawIndirectFirstInstance = VK_TRUE;

	if (_drawLineMode)
		vkPhysicalDeviceFeatures.fillModeNonSolid = VK_TRUE;

	VkPhysicalDeviceDynamicRenderingFeatures dynamicRenderingFeature = {
		.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DYNAMIC_RENDERING_FEATURES,
		.pNext = &drawFeat,
		.dynamicRendering = VK_TRUE
	};

	VkDeviceCreateInfo vkDeviceCreateInfo{};
	vkDeviceCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	vkDeviceCreateInfo.pEnabledFeatures = &vkPhysicalDeviceFeatures;
	vkDeviceCreateInfo.pNext = &dynamicRenderingFeature;

	if (_enableValidationLayers) 
	{
		vkDeviceCreateInfo.enabledLayerCount = static_cast<uint32_t>(_validationLayers.size());
		vkDeviceCreateInfo.ppEnabledLayerNames = _validationLayers.data();
	}
	else 
	{
		vkDeviceCreateInfo.enabledLayerCount = 0;
	}

	std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
	std::set<uint32_t> uniqueQueueFamilies = { queueFamily.graphicsFamily.value(), queueFamily.presentFamily.value() };

	for (uint32_t queueFamily : uniqueQueueFamilies) {
		VkDeviceQueueCreateInfo queueCreateInfo{};
		queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
		queueCreateInfo.queueFamilyIndex = queueFamily;
		queueCreateInfo.queueCount = 1;
		queueCreateInfo.pQueuePriorities = &queuePriority;
		queueCreateInfos.push_back(queueCreateInfo);
	}

	vkDeviceCreateInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
	vkDeviceCreateInfo.pQueueCreateInfos = queueCreateInfos.data();

	vkDeviceCreateInfo.enabledExtensionCount = static_cast<uint32_t>(_requiredDeviceExtensions.size());
	vkDeviceCreateInfo.ppEnabledExtensionNames = _requiredDeviceExtensions.data();

	if (vkCreateDevice(_vkPhysicalDevice, &vkDeviceCreateInfo, nullptr, &_vkDevice) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::FAILED_TO_CREATE_A_LOGICAL_DEVICE\n");
	}

	vkGetDeviceQueue(_vkDevice, queueFamily.graphicsFamily.value(), 0, &_vkGraphicsQueue);
	vkGetDeviceQueue(_vkDevice, queueFamily.presentFamily.value(), 0, &_vkPresentationQueue);


	// Load ray tracing function pointers from device driver
	_vkRtCreateRayTracingPipelinesKHR_PFN =
		reinterpret_cast<PFN_vkCreateRayTracingPipelinesKHR>(
			vkGetDeviceProcAddr(_vkDevice, "vkCreateRayTracingPipelinesKHR"));

	_vlRtGetRayTracingShaderGroupHandlesKHR_PFN =
		reinterpret_cast<PFN_vkGetRayTracingShaderGroupHandlesKHR>(
			vkGetDeviceProcAddr(_vkDevice, "vkGetRayTracingShaderGroupHandlesKHR"));

	_vkRtCmdTraceRaysKHR_PFN =
		reinterpret_cast<PFN_vkCmdTraceRaysKHR>(
			vkGetDeviceProcAddr(_vkDevice, "vkCmdTraceRaysKHR"));

	_vkCreateAccelerationStructureKHR_PFN =
		reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(
			vkGetDeviceProcAddr(_vkDevice, "vkCreateAccelerationStructureKHR"));

	_vkDestroyAccelerationStructureKHR_PFN =
		reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(
			vkGetDeviceProcAddr(_vkDevice, "vkDestroyAccelerationStructureKHR"));

	_vkGetAccelerationStructureBuildSizesKHR_PFN =
		reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(
			vkGetDeviceProcAddr(_vkDevice, "vkGetAccelerationStructureBuildSizesKHR"));

	_vkGetAccelerationStructureDeviceAddressKHR_PFN =
		reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(
			vkGetDeviceProcAddr(_vkDevice, "vkGetAccelerationStructureDeviceAddressKHR"));

	//_vkBuildAccelerationStructuresKHR_PFN =
	//	reinterpret_cast<PFN_vkBuildAccelerationStructuresKHR>(
	//		vkGetDeviceProcAddr(_vkDevice, "vkBuildAccelerationStructuresKHR"));

	_vkCmdBuildAccelerationStructuresKHR_PFN =
		reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(
			vkGetDeviceProcAddr(_vkDevice, "vkCmdBuildAccelerationStructuresKHR"));


	if (!_vkRtCreateRayTracingPipelinesKHR_PFN ||
		!_vlRtGetRayTracingShaderGroupHandlesKHR_PFN ||
		!_vkRtCmdTraceRaysKHR_PFN ||
		!_vkCreateAccelerationStructureKHR_PFN ||
		!_vkDestroyAccelerationStructureKHR_PFN ||
		!_vkGetAccelerationStructureBuildSizesKHR_PFN ||
		!_vkGetAccelerationStructureDeviceAddressKHR_PFN ||
		//!_vkBuildAccelerationStructuresKHR_PFN ||
		!_vkCmdBuildAccelerationStructuresKHR_PFN)
	{
		throw std::runtime_error("ERROR::VULKAN::RT::FAILED_TO_LOAD_RAY_TRACING_FUNCTION_POINTERS\n");
	}

}

void VulkanRenderer::createSwapChain()
{
	engine::vk::SurfaceSupportDetails surfaceSupportDetail = querySwapChainSupport(_vkPhysicalDevice);
	VkSurfaceFormatKHR vkSurfaceFormat = selectSwapSurfaceFormat(surfaceSupportDetail.formats);
	VkPresentModeKHR vkPresentMode = selectSwapPresentMode(surfaceSupportDetail.presentModes);
	VkExtent2D vkExtent = selectSwapExtent(surfaceSupportDetail.capabilities);

	uint32_t imageCount = surfaceSupportDetail.capabilities.minImageCount + 1;
	if (surfaceSupportDetail.capabilities.maxImageCount > 0 && imageCount > surfaceSupportDetail.capabilities.maxImageCount) 
	{
		imageCount = surfaceSupportDetail.capabilities.maxImageCount;
	}


	VkSwapchainCreateInfoKHR vkSwapchainCreateInfoKHR{};
	vkSwapchainCreateInfoKHR.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
	vkSwapchainCreateInfoKHR.surface = _vkSurface;
	vkSwapchainCreateInfoKHR.minImageCount = imageCount;
	vkSwapchainCreateInfoKHR.imageFormat = vkSurfaceFormat.format;
	vkSwapchainCreateInfoKHR.imageColorSpace = vkSurfaceFormat.colorSpace;
	vkSwapchainCreateInfoKHR.imageExtent = vkExtent;
	vkSwapchainCreateInfoKHR.imageArrayLayers = 1;
	vkSwapchainCreateInfoKHR.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

	engine::vk::QueueFamily queueFamily = getQueueFamilies(_vkPhysicalDevice);
	uint32_t queueFamilyIndices[] = { queueFamily.graphicsFamily.value(), queueFamily.presentFamily.value() };

	if (queueFamily.graphicsFamily != queueFamily.presentFamily) {
		vkSwapchainCreateInfoKHR.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
		vkSwapchainCreateInfoKHR.queueFamilyIndexCount = 2;
		vkSwapchainCreateInfoKHR.pQueueFamilyIndices = queueFamilyIndices;
	}
	else {
		vkSwapchainCreateInfoKHR.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
		vkSwapchainCreateInfoKHR.queueFamilyIndexCount = 0; // Optional
		vkSwapchainCreateInfoKHR.pQueueFamilyIndices = nullptr; // Optional
	}

	vkSwapchainCreateInfoKHR.preTransform = surfaceSupportDetail.capabilities.currentTransform;
	vkSwapchainCreateInfoKHR.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;

	vkSwapchainCreateInfoKHR.presentMode = vkPresentMode;
	vkSwapchainCreateInfoKHR.clipped = VK_TRUE;

	vkSwapchainCreateInfoKHR.oldSwapchain = VK_NULL_HANDLE;

	if (vkCreateSwapchainKHR(_vkDevice, &vkSwapchainCreateInfoKHR, nullptr, &_vkSwapChain) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::SWAP_CHAIN::FAILED_TO_CREATE_SWAP_CHAIN");
	}

	vkGetSwapchainImagesKHR(_vkDevice, _vkSwapChain, &imageCount, nullptr);
	_vkSwapChainImages.resize(imageCount);
	vkGetSwapchainImagesKHR(_vkDevice, _vkSwapChain, &imageCount, _vkSwapChainImages.data());

	_vkSwapChainImageFormat = vkSurfaceFormat.format;
	_vkSwapChainExtent = vkExtent;

	// Track current layout per image
	_swapchainLayouts.assign(imageCount, VK_IMAGE_LAYOUT_UNDEFINED);

	if(callBackOnSwapchainRecreateUI) callBackOnSwapchainRecreateUI(imageCount);
}

VkSurfaceFormatKHR VulkanRenderer::selectSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& availableFormats)
{
	for (const auto& availableFormat : availableFormats) {
		if (availableFormat.format == VK_FORMAT_B8G8R8A8_SRGB && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
			return availableFormat;
		}
	}

	return availableFormats[0];
}

VkPresentModeKHR VulkanRenderer::selectSwapPresentMode(const std::vector<VkPresentModeKHR>& availablePresentModes)
{
	/*
	Available modes:
	VK_PRESENT_MODE_IMMEDIATE_KHR
	VK_PRESENT_MODE_FIFO_KHR
	VK_PRESENT_MODE_FIFO_RELAXED_KHR
	VK_PRESENT_MODE_MAILBOX_KHR
	*/

	for (const auto& availablePresentMode : availablePresentModes) {
		if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR) {
			return availablePresentMode;
		}
	}

	return VK_PRESENT_MODE_FIFO_KHR;
}

VkExtent2D VulkanRenderer::selectSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities)
{
	if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
		return capabilities.currentExtent;
	}
	else {
		int width, height;
		glfwGetFramebufferSize(_GLFwindow, &width, &height);

		VkExtent2D actualExtent = {
			static_cast<uint32_t>(width),
			static_cast<uint32_t>(height)
		};

		actualExtent.width = std::clamp(actualExtent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
		actualExtent.height = std::clamp(actualExtent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);

		return actualExtent;
	}

}

void VulkanRenderer::createImageViews()
{
	_vkSwapChainImageViews.resize(_vkSwapChainImages.size());

	for (int i = 0; i < _vkSwapChainImages.size(); i++)
	{
		_vkSwapChainImageViews[i] = createImageView(_vkSwapChainImages[i],
			_vkSwapChainImageFormat, VK_IMAGE_ASPECT_COLOR_BIT, 1);
	}
}

VkImageView VulkanRenderer::createImageView(VkImage image, VkFormat format, VkImageAspectFlags aspectFlags, uint32_t mipLevels)
{

	VkImageViewCreateInfo vkImageViewCreateInfo{};
	vkImageViewCreateInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	vkImageViewCreateInfo.image = image;
	vkImageViewCreateInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
	vkImageViewCreateInfo.format = format;
	vkImageViewCreateInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	vkImageViewCreateInfo.subresourceRange.baseMipLevel = 0;
	vkImageViewCreateInfo.subresourceRange.levelCount = 1;
	vkImageViewCreateInfo.subresourceRange.baseArrayLayer = 0;
	vkImageViewCreateInfo.subresourceRange.layerCount = 1;
	vkImageViewCreateInfo.subresourceRange.aspectMask = aspectFlags;
	vkImageViewCreateInfo.subresourceRange.levelCount = mipLevels;

	VkImageView vkImageView;
	if (vkCreateImageView(_vkDevice, &vkImageViewCreateInfo, nullptr, &vkImageView) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::IMAGE_TEXTURE_VIEW::FIALED_TO_CREATE_TEXTURE_IMAMGE_VIEW\n");
	}

	return vkImageView;
}

VkFormat VulkanRenderer::findDepthFormat()
{
	return findSupportedFormat(
		{ VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT },
		VK_IMAGE_TILING_OPTIMAL,
		VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT
	);
}

VkFormat VulkanRenderer::findSupportedFormat(const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features)
{

	for (VkFormat format : candidates) {
		VkFormatProperties props;
		vkGetPhysicalDeviceFormatProperties(_vkPhysicalDevice, format, &props);

		if (tiling == VK_IMAGE_TILING_LINEAR
			&& (props.linearTilingFeatures & features) == features) {
			return format;
		}
		else if (tiling == VK_IMAGE_TILING_OPTIMAL
			&& (props.optimalTilingFeatures & features) == features) {
			return format;
		}
	}
	throw std::runtime_error("ERROR::VULKAN::FAILED_TO_FIND_SUPPORTED_FORMAT\n");


}

void VulkanRenderer::createDescriptorSetLayout()
{
	VkDescriptorSetLayoutBinding vkUBOLayoutBinding{};
	vkUBOLayoutBinding.binding = 0;
	vkUBOLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	vkUBOLayoutBinding.descriptorCount = 1;
	vkUBOLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
	vkUBOLayoutBinding.pImmutableSamplers = nullptr; // Optional

	/* RT camera UBO */
	VkDescriptorSetLayoutBinding vkUBORtCameraBinding{};
	vkUBORtCameraBinding.binding = 1;
	vkUBORtCameraBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	vkUBORtCameraBinding.descriptorCount = 1;
	/* binds to RAY GEN Stage*/
	vkUBORtCameraBinding.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
	vkUBORtCameraBinding.pImmutableSamplers = nullptr;

	VkDescriptorSetLayoutBinding vkUBOLayoutBindingSSBO{};
	vkUBOLayoutBindingSSBO.binding = 2;
	vkUBOLayoutBindingSSBO.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	vkUBOLayoutBindingSSBO.descriptorCount = 1;
	vkUBOLayoutBindingSSBO.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
	vkUBOLayoutBindingSSBO.pImmutableSamplers = nullptr; // Optional
	
	// RT color image as storage image
	VkDescriptorSetLayoutBinding vkImageBindingRt{};
	vkImageBindingRt.binding = 3; // choose a free binding index
	vkImageBindingRt.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
	vkImageBindingRt.descriptorCount = 1;

	// Used by raygen/closest-hit/miss and fullscreen fragment (imageLoad)
	vkImageBindingRt.stageFlags =
		VK_SHADER_STAGE_RAYGEN_BIT_KHR |
		VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR |
		VK_SHADER_STAGE_MISS_BIT_KHR |
		VK_SHADER_STAGE_FRAGMENT_BIT;
	vkImageBindingRt.pImmutableSamplers = nullptr;


	// TLAS binding for ray tracing
	VkDescriptorSetLayoutBinding vkTlasBinding{};
	vkTlasBinding.binding = 4;
	vkTlasBinding.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
	vkTlasBinding.descriptorCount = 1;
	vkTlasBinding.stageFlags =
		VK_SHADER_STAGE_RAYGEN_BIT_KHR |
		VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR; // hit shaders will use it too later
	vkTlasBinding.pImmutableSamplers = nullptr;

	// Vertex buffer as storage buffer (for ray tracing hit shader)
	VkDescriptorSetLayoutBinding vkRtVertexBinding{};
	vkRtVertexBinding.binding = 5;
	vkRtVertexBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	vkRtVertexBinding.descriptorCount = 1;
	vkRtVertexBinding.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
	vkRtVertexBinding.pImmutableSamplers = nullptr;

	// Index buffer as storage buffer
	VkDescriptorSetLayoutBinding vkRtIndexBinding{};
	vkRtIndexBinding.binding = 6;
	vkRtIndexBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	vkRtIndexBinding.descriptorCount = 1;
	vkRtIndexBinding.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
	vkRtIndexBinding.pImmutableSamplers = nullptr;

	VkDescriptorSetLayoutBinding vkRtInstanceBinding{};
	vkRtInstanceBinding.binding = 7;
	vkRtInstanceBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	vkRtInstanceBinding.descriptorCount = 1;
	vkRtInstanceBinding.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
	vkRtInstanceBinding.pImmutableSamplers = nullptr;

	// Ray sample Settings
	VkDescriptorSetLayoutBinding vkRtRaySamaplesBinding{};
	vkRtRaySamaplesBinding.binding = 8;
	vkRtRaySamaplesBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	vkRtRaySamaplesBinding.descriptorCount = 1;
	vkRtRaySamaplesBinding.stageFlags = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
	vkRtRaySamaplesBinding.pImmutableSamplers = nullptr;

	std::array<VkDescriptorSetLayoutBinding, 9> bindings = { 
		vkUBOLayoutBinding, 
		vkUBORtCameraBinding, 
		vkUBOLayoutBindingSSBO, 
		vkImageBindingRt, 
		vkTlasBinding,
		vkRtVertexBinding,
		vkRtIndexBinding,
		vkRtInstanceBinding,
		vkRtRaySamaplesBinding
	};

	VkDescriptorSetLayoutCreateInfo vkDescriptorLayoutInfo{};
	vkDescriptorLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	vkDescriptorLayoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());;
	vkDescriptorLayoutInfo.pBindings = bindings.data();

	if (vkCreateDescriptorSetLayout(_vkDevice, &vkDescriptorLayoutInfo, nullptr, &_vkDescriptorSetLayout) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::DESCRIPTOR::FAILED_TO_CREATE_DESCRIPTOR_SET_LAYOUT");
	}
}

std::filesystem::path VulkanRenderer::findShaderPath(const char* path)
{
	// prefer project root when launched from VS
	if (std::filesystem::exists(path))
		return path;

	// otherwise use current path
	return std::filesystem::current_path() / path;
}

void VulkanRenderer::createGraphicsPipeline()
{
	std::vector<char> vertShaderCode = readFile(findShaderPath("src\\shaders\\vertex.spv").string());
	std::vector<char> fragShaderCode = readFile(findShaderPath("src\\shaders\\fragment.spv").string());

	VkShaderModule vkVertShaderModule = createVKShaderModule(vertShaderCode);
	VkShaderModule vkFragShaderModule = createVKShaderModule(fragShaderCode);

	VkPipelineShaderStageCreateInfo vkVertShaderStageInfo{};
	vkVertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	vkVertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
	vkVertShaderStageInfo.module = vkVertShaderModule;
	vkVertShaderStageInfo.pName = "main";

	VkPipelineShaderStageCreateInfo vkFragShaderStageInfo{};
	vkFragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	vkFragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
	vkFragShaderStageInfo.module = vkFragShaderModule;
	vkFragShaderStageInfo.pName = "main";

	VkPipelineShaderStageCreateInfo vkShaderStages[] = { vkVertShaderStageInfo, vkFragShaderStageInfo };

	VkVertexInputBindingDescription vkBindingDescription = engine::vk::getVertexBindingDescription();
	std::array<VkVertexInputAttributeDescription, 3> arrVkAttributeDescriptions = engine::vk::getVertexAttributeDescriptions();

	VkPipelineVertexInputStateCreateInfo vkVertexInputInfo{};
	vkVertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
	vkVertexInputInfo.vertexBindingDescriptionCount = 1;
	vkVertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(arrVkAttributeDescriptions.size());
	vkVertexInputInfo.pVertexBindingDescriptions = &vkBindingDescription;
	vkVertexInputInfo.pVertexAttributeDescriptions = arrVkAttributeDescriptions.data();

	VkPipelineInputAssemblyStateCreateInfo vkInputAssembly{};
	vkInputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
	vkInputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
	vkInputAssembly.primitiveRestartEnable = VK_FALSE;


	std::vector<VkDynamicState> vkDynamicStates = {
	VK_DYNAMIC_STATE_VIEWPORT,
	VK_DYNAMIC_STATE_SCISSOR
	};

	VkPipelineDynamicStateCreateInfo vkDynamicState{};
	vkDynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
	vkDynamicState.dynamicStateCount = static_cast<uint32_t>(vkDynamicStates.size());
	vkDynamicState.pDynamicStates = vkDynamicStates.data();

	VkViewport vkViewport{};
	vkViewport.x = 0.0f;
	vkViewport.y = 0.0f;
	vkViewport.width = (float)_vkSwapChainExtent.width;
	vkViewport.height = (float)_vkSwapChainExtent.height;
	vkViewport.minDepth = 0.0f;
	vkViewport.maxDepth = 1.0f;

	VkRect2D vkScissor{};
	vkScissor.offset = { 0, 0 };
	vkScissor.extent = _vkSwapChainExtent;

	VkPipelineViewportStateCreateInfo vkViewportState{};
	vkViewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
	vkViewportState.viewportCount = 1;
	vkViewportState.pViewports = &vkViewport;
	vkViewportState.scissorCount = 1;
	vkViewportState.pScissors = &vkScissor;


	VkPipelineRasterizationStateCreateInfo vkRasterizer{};
	vkRasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
	vkRasterizer.depthClampEnable = VK_FALSE;
	vkRasterizer.rasterizerDiscardEnable = VK_FALSE;
	vkRasterizer.polygonMode = VK_POLYGON_MODE_FILL;
	if (_drawLineMode)
		vkRasterizer.polygonMode = VK_POLYGON_MODE_LINE;
	vkRasterizer.lineWidth = 1.0f;
	/*
	Available polygonMode:
		VK_POLYGON_MODE_FILL: fill the area of the polygon with fragments
		VK_POLYGON_MODE_LINE: polygon edges are drawn as lines
		VK_POLYGON_MODE_POINT: polygon vertices are drawn as points
	*/
	vkRasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
	//vkRasterizer.cullMode = VK_CULL_MODE_NONE;
	//vkRasterizer.frontFace = VK_FRONT_FACE_CLOCKWISE;
	vkRasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE; // <- using this now because Y axis is fliped
	vkRasterizer.depthBiasEnable = VK_FALSE;
	vkRasterizer.depthBiasConstantFactor = 0.0f; // Optional
	vkRasterizer.depthBiasClamp = 0.0f; // Optional
	vkRasterizer.depthBiasSlopeFactor = 0.0f; // Optional


	VkPipelineMultisampleStateCreateInfo vkMultisampling{};
	vkMultisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
	//vkMultisampling.sampleShadingEnable = VK_FALSE;
	vkMultisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
	vkMultisampling.minSampleShading = 1.0f; // Optional
	vkMultisampling.pSampleMask = nullptr; // Optional
	vkMultisampling.alphaToCoverageEnable = VK_FALSE; // Optional
	vkMultisampling.alphaToOneEnable = VK_FALSE; // Optional
	vkMultisampling.rasterizationSamples = _vkMsaaSamples;
	/* cost more performance !! */
	vkMultisampling.sampleShadingEnable = VK_TRUE; // enable sample shading in the pipeline
	vkMultisampling.minSampleShading = .2f; // min fraction for sample shading; closer to one is smoother

	VkPipelineColorBlendAttachmentState vkColorBlendAttachment{};
	vkColorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
	//vkColorBlendAttachment.blendEnable = VK_FALSE;
	//vkColorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE; // Optional
	//vkColorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ZERO; // Optional
	//vkColorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD; // Optional
	//vkColorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE; // Optional
	//vkColorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO; // Optional
	//vkColorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD; // Optional

	vkColorBlendAttachment.blendEnable = VK_TRUE;
	vkColorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
	vkColorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
	vkColorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
	vkColorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
	vkColorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
	vkColorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

	VkPipelineColorBlendStateCreateInfo vkColorBlending{};
	vkColorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
	vkColorBlending.logicOpEnable = VK_FALSE;
	vkColorBlending.logicOp = VK_LOGIC_OP_COPY; // Optional
	vkColorBlending.attachmentCount = 1;
	vkColorBlending.pAttachments = &vkColorBlendAttachment;
	vkColorBlending.blendConstants[0] = 0.0f; // Optional
	vkColorBlending.blendConstants[1] = 0.0f; // Optional
	vkColorBlending.blendConstants[2] = 0.0f; // Optional
	vkColorBlending.blendConstants[3] = 0.0f; // Optional

	VkPipelineLayoutCreateInfo vkPipelineLayoutInfo{};
	vkPipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	vkPipelineLayoutInfo.setLayoutCount = 1; // Optional
	vkPipelineLayoutInfo.pSetLayouts = &_vkDescriptorSetLayout;
	vkPipelineLayoutInfo.pushConstantRangeCount = 0; // Optional
	vkPipelineLayoutInfo.pPushConstantRanges = nullptr; // Optional

	if (vkCreatePipelineLayout(_vkDevice, &vkPipelineLayoutInfo, nullptr, &_vkPipelineLayout) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::PIPELINE::FAILED_TO_CREATE_A_PIPELINE_LAYOUT\n");
	}


	/* dynamic rendering inserting formats here instead of in frame buffer */
	VkFormat colorFmt = _vkSwapChainImageFormat;
	VkFormat depthFmt = findDepthFormat();

	VkPipelineRenderingCreateInfo pipeRendering{};
	pipeRendering.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
	pipeRendering.colorAttachmentCount = 1;
	pipeRendering.pColorAttachmentFormats = &colorFmt;
	pipeRendering.depthAttachmentFormat = depthFmt;
	/* end */

	/* related to depth testing and depth buffer */
	VkPipelineDepthStencilStateCreateInfo vkPipelineDepthStencilStateCreateInfo{};
	vkPipelineDepthStencilStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	vkPipelineDepthStencilStateCreateInfo.depthTestEnable = VK_TRUE;
	vkPipelineDepthStencilStateCreateInfo.depthWriteEnable = VK_TRUE;
	vkPipelineDepthStencilStateCreateInfo.depthCompareOp = VK_COMPARE_OP_LESS;
	vkPipelineDepthStencilStateCreateInfo.depthBoundsTestEnable = VK_FALSE;
	vkPipelineDepthStencilStateCreateInfo.minDepthBounds = 0.0f; // Optional
	vkPipelineDepthStencilStateCreateInfo.maxDepthBounds = 1.0f; // Optional
	/* these for configuring stencil buffer operations*/
	vkPipelineDepthStencilStateCreateInfo.stencilTestEnable = VK_FALSE;
	vkPipelineDepthStencilStateCreateInfo.front = {}; // Optional
	vkPipelineDepthStencilStateCreateInfo.back = {}; // Optional

	VkGraphicsPipelineCreateInfo vkPipelineInfo{};
	vkPipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
	vkPipelineInfo.stageCount = 2;
	vkPipelineInfo.pStages = vkShaderStages;
	vkPipelineInfo.pVertexInputState = &vkVertexInputInfo;
	vkPipelineInfo.pInputAssemblyState = &vkInputAssembly;
	vkPipelineInfo.pViewportState = &vkViewportState;
	vkPipelineInfo.pRasterizationState = &vkRasterizer;
	vkPipelineInfo.pMultisampleState = &vkMultisampling;
	vkPipelineInfo.pDepthStencilState = &vkPipelineDepthStencilStateCreateInfo; // Optional
	vkPipelineInfo.pColorBlendState = &vkColorBlending;
	vkPipelineInfo.pDynamicState = &vkDynamicState;
	vkPipelineInfo.layout = _vkPipelineLayout;

	//vkPipelineInfo.renderPass = _vkRenderPass;
	vkPipelineInfo.renderPass = VK_NULL_HANDLE; // change to null handle / for dynamic rendering
	vkPipelineInfo.pNext = &pipeRendering; // and instead passing "VkPipelineRenderingCreateInfo"

	vkPipelineInfo.subpass = 0;

	vkPipelineInfo.basePipelineHandle = VK_NULL_HANDLE; // Optional
	vkPipelineInfo.basePipelineIndex = -1; // Optional


	if (vkCreateGraphicsPipelines(_vkDevice, VK_NULL_HANDLE, 1, &vkPipelineInfo, nullptr, &_vkGraphicsPipeline) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::PIPELINE::FIALED_TO_CREATE_GRAPHIS_PIPELINE\n");
	}

	vkDestroyShaderModule(_vkDevice, vkFragShaderModule, nullptr);
	vkDestroyShaderModule(_vkDevice, vkVertShaderModule, nullptr);
}

void VulkanRenderer::createRtGraphicsPipeline()
{

	std::vector<char> rt_vertShaderCode = readFile(findShaderPath("src\\shaders\\rt_vertex.spv").string());
	std::vector<char> rt_fragShaderCode = readFile(findShaderPath("src\\shaders\\rt_fragment.spv").string());

	VkShaderModule rt_vertShaderModule = createVKShaderModule(rt_vertShaderCode);
	VkShaderModule rt_fragShaderModule = createVKShaderModule(rt_fragShaderCode);

	VkPipelineShaderStageCreateInfo vkRtVertShaderStageInfo{};
	vkRtVertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	vkRtVertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
	vkRtVertShaderStageInfo.module = rt_vertShaderModule;
	vkRtVertShaderStageInfo.pName = "main";

	VkPipelineShaderStageCreateInfo vkRtFragShaderStageInfo{};
	vkRtFragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	vkRtFragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
	vkRtFragShaderStageInfo.module = rt_fragShaderModule;
	vkRtFragShaderStageInfo.pName = "main";

	VkPipelineShaderStageCreateInfo vkRtShaderStages[] = { vkRtVertShaderStageInfo, vkRtFragShaderStageInfo };

	// No vertex input (use gl_VertexIndex)
	VkPipelineVertexInputStateCreateInfo vkVertexInputInfo{};
	vkVertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
	vkVertexInputInfo.vertexBindingDescriptionCount = 0;
	vkVertexInputInfo.pVertexBindingDescriptions = nullptr;
	vkVertexInputInfo.vertexAttributeDescriptionCount = 0;
	vkVertexInputInfo.pVertexAttributeDescriptions = nullptr;

	VkPipelineInputAssemblyStateCreateInfo vkInputAssembly{};
	vkInputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
	vkInputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
	vkInputAssembly.primitiveRestartEnable = VK_FALSE;

	VkViewport vkViewport{};
	vkViewport.x = 0.0f;
	vkViewport.y = 0.0f;
	vkViewport.width = static_cast<float>(_vkSwapChainExtent.width);
	vkViewport.height = static_cast<float>(_vkSwapChainExtent.height);
	vkViewport.minDepth = 0.0f;
	vkViewport.maxDepth = 1.0f;

	VkRect2D vkScissor{};
	vkScissor.offset = { 0, 0 };
	vkScissor.extent = _vkSwapChainExtent;

	VkPipelineViewportStateCreateInfo vkViewportStateInfo{};
	vkViewportStateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
	vkViewportStateInfo.viewportCount = 1;
	vkViewportStateInfo.pViewports = &vkViewport;
	vkViewportStateInfo.scissorCount = 1;
	vkViewportStateInfo.pScissors = &vkScissor;

	VkPipelineRasterizationStateCreateInfo vkRasterizerInfo{};
	vkRasterizerInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
	vkRasterizerInfo.depthClampEnable = VK_FALSE;
	vkRasterizerInfo.rasterizerDiscardEnable = VK_FALSE;
	vkRasterizerInfo.polygonMode = VK_POLYGON_MODE_FILL;
	vkRasterizerInfo.cullMode = VK_CULL_MODE_NONE;
	vkRasterizerInfo.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
	vkRasterizerInfo.depthBiasEnable = VK_FALSE;
	vkRasterizerInfo.lineWidth = 1.0f;

	VkPipelineMultisampleStateCreateInfo vkMultisamplingInfo{};
	vkMultisamplingInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
	vkMultisamplingInfo.sampleShadingEnable = VK_FALSE;
	vkMultisamplingInfo.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

	VkPipelineColorBlendAttachmentState vkColorBlendAttachment{};
	vkColorBlendAttachment.colorWriteMask =
		VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
		VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
	vkColorBlendAttachment.blendEnable = VK_FALSE;

	VkPipelineColorBlendStateCreateInfo vkColorBlendingInfo{};
	vkColorBlendingInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
	vkColorBlendingInfo.logicOpEnable = VK_FALSE;
	vkColorBlendingInfo.attachmentCount = 1;
	vkColorBlendingInfo.pAttachments = &vkColorBlendAttachment;

	VkPipelineDepthStencilStateCreateInfo vkDepthStencilInfo{};
	vkDepthStencilInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	vkDepthStencilInfo.depthTestEnable = VK_FALSE;
	vkDepthStencilInfo.depthWriteEnable = VK_FALSE;

	VkPipelineRenderingCreateInfo vkRtPipeRendering{};
	vkRtPipeRendering.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
	vkRtPipeRendering.colorAttachmentCount = 1;
	vkRtPipeRendering.pColorAttachmentFormats = &_vkSwapChainImageFormat;
	vkRtPipeRendering.depthAttachmentFormat = VK_FORMAT_UNDEFINED; // no depth for fullscreen blit


	// Use existing pipeline layout (_vkPipelineLayout)
	VkGraphicsPipelineCreateInfo vkRtGraphicsPipelineInfo{};
	vkRtGraphicsPipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
	vkRtGraphicsPipelineInfo.stageCount = 2;
	vkRtGraphicsPipelineInfo.pStages = vkRtShaderStages;
	vkRtGraphicsPipelineInfo.pVertexInputState = &vkVertexInputInfo;
	vkRtGraphicsPipelineInfo.pInputAssemblyState = &vkInputAssembly;
	vkRtGraphicsPipelineInfo.pViewportState = &vkViewportStateInfo;
	vkRtGraphicsPipelineInfo.pRasterizationState = &vkRasterizerInfo;
	vkRtGraphicsPipelineInfo.pMultisampleState = &vkMultisamplingInfo;
	vkRtGraphicsPipelineInfo.pDepthStencilState = &vkDepthStencilInfo;
	vkRtGraphicsPipelineInfo.pColorBlendState = &vkColorBlendingInfo;
	vkRtGraphicsPipelineInfo.layout = _vkPipelineLayout;

	vkRtGraphicsPipelineInfo.renderPass = VK_NULL_HANDLE; 
	vkRtGraphicsPipelineInfo.pNext = &vkRtPipeRendering; // hook dynamic rendering

	if (vkCreateGraphicsPipelines(
		_vkDevice,
		VK_NULL_HANDLE,
		1,
		&vkRtGraphicsPipelineInfo,
		nullptr,
		&_vkRtGraphicsPipeline) != VK_SUCCESS)
	{
		throw std::runtime_error("ERROR::VULKAN::PIPELINE::FIALED_TO_CREATE_RT_GRAPHIS_PIPELINE\n");
	}

	vkDestroyShaderModule(_vkDevice, rt_vertShaderModule, nullptr);
	vkDestroyShaderModule(_vkDevice, rt_fragShaderModule, nullptr);
}

void VulkanRenderer::createRayTracingPipeline()
{
	std::vector<char> raygenCode = readFile(findShaderPath("src\\shaders\\rt_raygen.spv").string());
	std::vector<char> raymissCode = readFile(findShaderPath("src\\shaders\\rt_miss.spv").string());
	std::vector<char> rayhitCode = readFile(findShaderPath("src\\shaders\\rt_hit.spv").string());
	std::vector<char> rayshadowhitCode = readFile(findShaderPath("src\\shaders\\rt_shadow_hit.spv").string());
	std::vector<char> rayshadowmissCode = readFile(findShaderPath("src\\shaders\\rt_shadow_miss.spv").string());

	VkShaderModule vkRaygenModule = createVKShaderModule(raygenCode);
	VkShaderModule vkRaymissModule = createVKShaderModule(raymissCode);
	VkShaderModule vkRayhitModule = createVKShaderModule(rayhitCode);
	VkShaderModule vkRayShadowHitModule = createVKShaderModule(rayshadowhitCode);
	VkShaderModule vkRayShadowMissModule = createVKShaderModule(rayshadowmissCode);

	VkPipelineShaderStageCreateInfo stages[5]{};

	// Raygen stage
	stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	stages[0].stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
	stages[0].module = vkRaygenModule;
	stages[0].pName = "main";

	// Miss stage
	stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	stages[1].stage = VK_SHADER_STAGE_MISS_BIT_KHR;
	stages[1].module = vkRaymissModule;
	stages[1].pName = "main";

	// Closest-hit stage (index 2)
	stages[2].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	stages[2].stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
	stages[2].module = vkRayhitModule;
	stages[2].pName = "main";


	//  shadow miss
	stages[3].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	stages[3].stage = VK_SHADER_STAGE_MISS_BIT_KHR;
	stages[3].module = vkRayShadowMissModule;
	stages[3].pName = "main";

	//   shadow closest-hit
	stages[4].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	stages[4].stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
	stages[4].module = vkRayShadowHitModule;
	stages[4].pName = "main";

	VkRayTracingShaderGroupCreateInfoKHR vkRtShadergroups[5]{};

	// Group 0: raygen
	vkRtShadergroups[0].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
	vkRtShadergroups[0].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
	vkRtShadergroups[0].generalShader = 0; // stages[0] = rt_raygen
	vkRtShadergroups[0].closestHitShader = VK_SHADER_UNUSED_KHR;
	vkRtShadergroups[0].anyHitShader = VK_SHADER_UNUSED_KHR;
	vkRtShadergroups[0].intersectionShader = VK_SHADER_UNUSED_KHR;

	// Group 1: radiance miss
	vkRtShadergroups[1].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
	vkRtShadergroups[1].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
	vkRtShadergroups[1].generalShader = 1; // stages[1] = rt_miss
	vkRtShadergroups[1].closestHitShader = VK_SHADER_UNUSED_KHR;
	vkRtShadergroups[1].anyHitShader = VK_SHADER_UNUSED_KHR;
	vkRtShadergroups[1].intersectionShader = VK_SHADER_UNUSED_KHR;

	// Group 2: shadow miss  
	vkRtShadergroups[2].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
	vkRtShadergroups[2].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
	vkRtShadergroups[2].generalShader = 3; // stages[3] = rt_shadow_miss
	vkRtShadergroups[2].closestHitShader = VK_SHADER_UNUSED_KHR;
	vkRtShadergroups[2].anyHitShader = VK_SHADER_UNUSED_KHR;
	vkRtShadergroups[2].intersectionShader = VK_SHADER_UNUSED_KHR;

	// Group 3: radiance hit
	vkRtShadergroups[3].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
	vkRtShadergroups[3].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
	vkRtShadergroups[3].generalShader = VK_SHADER_UNUSED_KHR;
	vkRtShadergroups[3].closestHitShader = 2; // stages[2] = rt_hit.rhit
	vkRtShadergroups[3].anyHitShader = VK_SHADER_UNUSED_KHR;
	vkRtShadergroups[3].intersectionShader = VK_SHADER_UNUSED_KHR;

	// Group 4: shadow hit
	vkRtShadergroups[4].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
	vkRtShadergroups[4].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
	vkRtShadergroups[4].generalShader = VK_SHADER_UNUSED_KHR;
	vkRtShadergroups[4].closestHitShader = 4; // stages[4] = rt_shadow_hit.rshit
	vkRtShadergroups[4].anyHitShader = VK_SHADER_UNUSED_KHR;
	vkRtShadergroups[4].intersectionShader = VK_SHADER_UNUSED_KHR;


	// 3) Pipeline create info
	VkRayTracingPipelineCreateInfoKHR rtInfo{};
	rtInfo.sType = VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR;
	rtInfo.stageCount = 5;
	rtInfo.pStages = stages;
	rtInfo.groupCount = 5;
	rtInfo.pGroups = vkRtShadergroups;
	rtInfo.maxPipelineRayRecursionDepth = 2;
	// existing descriptor layout / pipeline layout
	rtInfo.layout = _vkPipelineLayout;

	VkResult res = _vkRtCreateRayTracingPipelinesKHR_PFN(
		_vkDevice,
		VK_NULL_HANDLE,            // deferred operation
		VK_NULL_HANDLE,            // pipeline cache
		1,
		&rtInfo,
		nullptr,
		&_vkRtPipeline
	);

	vkDestroyShaderModule(_vkDevice, vkRaygenModule, nullptr);
	vkDestroyShaderModule(_vkDevice, vkRaymissModule, nullptr);
	vkDestroyShaderModule(_vkDevice, vkRayhitModule, nullptr);
	vkDestroyShaderModule(_vkDevice, vkRayShadowHitModule, nullptr);
	vkDestroyShaderModule(_vkDevice, vkRayShadowMissModule, nullptr);

	if (res != VK_SUCCESS)
	{
		throw std::runtime_error("ERROR::VULKAN::RT::FAILED_TO_CREATE_RAY_TRACING_PIPELINE\n");
	}
}

std::vector<char> VulkanRenderer::readFile(const std::string& filename) 
{
	std::ifstream file(filename, std::ios::ate | std::ios::binary);

	if (!file.is_open()) {
		throw std::runtime_error("ERROR::VULKAN::SHADER::FIALED_LAODING_FILE\n");
	}

	size_t fileSize = (size_t)file.tellg();
	std::vector<char> buffer(fileSize);
	file.seekg(0);
	file.read(buffer.data(), fileSize);
	file.close();
	return buffer;
}

VkShaderModule VulkanRenderer::createVKShaderModule(const std::vector<char>& shaderCode)
{
	VkShaderModuleCreateInfo vkShaderModuleCreateInfo{};
	vkShaderModuleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	vkShaderModuleCreateInfo.codeSize = shaderCode.size();
	vkShaderModuleCreateInfo.pCode = reinterpret_cast<const uint32_t*>(shaderCode.data());


	VkShaderModule vkShaderModule;
	if (vkCreateShaderModule(_vkDevice, &vkShaderModuleCreateInfo, nullptr, &vkShaderModule) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::SHADER::FIALED_TO_CREATE_SHADER_MODUL\n");
	}

	return vkShaderModule;
}

void VulkanRenderer::createCommandPool()
{
	engine::vk::QueueFamily queueFamily = getQueueFamilies(_vkPhysicalDevice);

	VkCommandPoolCreateInfo vkPoolInfo{};
	vkPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	vkPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	vkPoolInfo.queueFamilyIndex = queueFamily.graphicsFamily.value();

	if (vkCreateCommandPool(_vkDevice, &vkPoolInfo, nullptr, &_vkCommandPool) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::COMMAND_POOL::FAILED_TO_CREATE_POOL\n");
	}
}

void VulkanRenderer::createColorResources()
{
	VkFormat vkColorFormat = _vkSwapChainImageFormat;

	createImage(_vkSwapChainExtent.width, _vkSwapChainExtent.height, 1
		, _vkMsaaSamples, vkColorFormat, VK_IMAGE_TILING_OPTIMAL
		, VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT
		, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, _vkColorImage, _vkColorImageMemory);

	_vkColorImageView = createImageView(_vkColorImage, vkColorFormat, VK_IMAGE_ASPECT_COLOR_BIT, 1);


	transitionImageLayout(
		_vkColorImage,
		_vkSwapChainImageFormat,
		VK_IMAGE_LAYOUT_UNDEFINED,
		VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
		/*mipLevels*/1
	);
}

void VulkanRenderer::createDepthResources()
{
	VkFormat vkDepthFormat = findDepthFormat();

	createImage(_vkSwapChainExtent.width, _vkSwapChainExtent.height, 1, _vkMsaaSamples, vkDepthFormat,
		VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, _vkDepthImage, _vkDepthImageMemory);

	_vkDepthImageView = createImageView(_vkDepthImage, vkDepthFormat, VK_IMAGE_ASPECT_DEPTH_BIT, 1);

	transitionImageLayout(_vkDepthImage, vkDepthFormat, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL, 1);
}

void VulkanRenderer::createRtColourResources()
{
	// using R8G8B8A8_UNORM to use VK_IMAGE_USAGE_STORAGE_BIT
	VkFormat rtFormat = VK_FORMAT_R8G8B8A8_UNORM;

	createImage(
		_vkSwapChainExtent.width,
		_vkSwapChainExtent.height,
		1,                             // mipLevels
		VK_SAMPLE_COUNT_1_BIT,         // numSamples
		rtFormat,
		VK_IMAGE_TILING_OPTIMAL,
		VK_IMAGE_USAGE_STORAGE_BIT |   // raygen  
		VK_IMAGE_USAGE_SAMPLED_BIT,    // fullscreen pass samples 
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
		_vkRtColorImage,
		_vkRtColorImageMemory
	);

	_vkRtColorImageView = createImageView(
		_vkRtColorImage,
		rtFormat,
		VK_IMAGE_ASPECT_COLOR_BIT,
		1
	);

	transitionImageLayout(
		_vkRtColorImage,
		rtFormat,
		VK_IMAGE_LAYOUT_UNDEFINED,
		VK_IMAGE_LAYOUT_GENERAL,
		1
	);
}

void VulkanRenderer::createImage(uint32_t width, uint32_t height, uint32_t mipLevels, VkSampleCountFlagBits numSamples, VkFormat format, VkImageTiling tiling
	, VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImage& image, VkDeviceMemory& imageMemory)
{
	VkImageCreateInfo vkImageInfo{};
	vkImageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	vkImageInfo.imageType = VK_IMAGE_TYPE_2D;
	vkImageInfo.extent.width = width;
	vkImageInfo.extent.height = height;
	vkImageInfo.extent.depth = 1;
	vkImageInfo.mipLevels = 1;
	vkImageInfo.arrayLayers = 1;
	vkImageInfo.format = format;
	vkImageInfo.tiling = tiling;
	vkImageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	vkImageInfo.usage = usage;
	vkImageInfo.samples = numSamples;
	vkImageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	vkImageInfo.mipLevels = mipLevels;


	if (vkCreateImage(_vkDevice, &vkImageInfo, nullptr, &image) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::IMAGE::FAILED_TO_CREATE_IMAMGE\n");
	}

	VkMemoryRequirements vkMemRequirements;
	vkGetImageMemoryRequirements(_vkDevice, image, &vkMemRequirements);

	VkMemoryAllocateInfo vkAllocInfo{};
	vkAllocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	vkAllocInfo.allocationSize = vkMemRequirements.size;
	vkAllocInfo.memoryTypeIndex = getMemoryType(vkMemRequirements.memoryTypeBits, properties);

	if (vkAllocateMemory(_vkDevice, &vkAllocInfo, nullptr, &imageMemory) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::ALLOCATE_IMAGE::FIALED_ALLOCATE_IMAGE_MEMORY\n");
	}

	vkBindImageMemory(_vkDevice, image, imageMemory, 0);


}

uint32_t VulkanRenderer::getMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties)
{
	VkPhysicalDeviceMemoryProperties vkMemProperties;
	vkGetPhysicalDeviceMemoryProperties(_vkPhysicalDevice, &vkMemProperties);

	for (uint32_t i = 0; i < vkMemProperties.memoryTypeCount; i++) {
		if ((typeFilter & (1 << i)) &&
			(vkMemProperties.memoryTypes[i].propertyFlags & properties) == properties) {

			return i;
		}
	}

	throw std::runtime_error("ERROR::VULKAN::MEMORY::GET_MEMORY_TYPE::FIALED_TO_FIND_SUITABLE_MEMORY_TYPE\n");

	return 0;
}

void VulkanRenderer::transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout, uint32_t mipLevels)
{
	VkCommandBuffer vkCommandBuffer = beginSingleTimeCommands();

	VkImageMemoryBarrier vkImageMemoryBarrier{};
	vkImageMemoryBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
	vkImageMemoryBarrier.oldLayout = oldLayout;
	vkImageMemoryBarrier.newLayout = newLayout;

	vkImageMemoryBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	vkImageMemoryBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

	vkImageMemoryBarrier.image = image;
	vkImageMemoryBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	vkImageMemoryBarrier.subresourceRange.baseMipLevel = 0;
	vkImageMemoryBarrier.subresourceRange.levelCount = 1;
	vkImageMemoryBarrier.subresourceRange.baseArrayLayer = 0;
	vkImageMemoryBarrier.subresourceRange.layerCount = 1;

	vkImageMemoryBarrier.srcAccessMask = 0; // TODO
	vkImageMemoryBarrier.dstAccessMask = 0; // TODO

	vkImageMemoryBarrier.subresourceRange.levelCount = mipLevels;


	VkPipelineStageFlags vkSourceStage;
	VkPipelineStageFlags vkDestinationStage;

	if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
		vkImageMemoryBarrier.srcAccessMask = 0;
		vkImageMemoryBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

		vkSourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
		vkDestinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
	}
	else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {
		vkImageMemoryBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
		vkImageMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

		vkSourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
		vkDestinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
	}
	else if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) {
		vkImageMemoryBarrier.srcAccessMask = 0;
		vkImageMemoryBarrier.dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

		vkSourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
		vkDestinationStage = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
	}
	else if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED &&
		newLayout == VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL)
	{
		vkImageMemoryBarrier.srcAccessMask = 0;
		vkImageMemoryBarrier.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

		vkSourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
		vkDestinationStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	}
	else if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED &&
		newLayout == VK_IMAGE_LAYOUT_GENERAL)
	{
		vkImageMemoryBarrier.srcAccessMask = 0;
		vkImageMemoryBarrier.dstAccessMask =
			VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;

		vkSourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
		vkDestinationStage =
			VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT |
			VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR;
	}
	else {
		throw std::invalid_argument("WARNING::INVALID_ARGUMENT::VULAKN::UNSUPPORTED_LAYOUT_TRANSITION\n");
	}

	if (newLayout == VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) {
		vkImageMemoryBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;

		if (hasStencilComponent(format)) {
			vkImageMemoryBarrier.subresourceRange.aspectMask |= VK_IMAGE_ASPECT_STENCIL_BIT;
		}
	}
	else {
		vkImageMemoryBarrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	}


	vkCmdPipelineBarrier(
		vkCommandBuffer,
		vkSourceStage, vkDestinationStage,
		0,
		0, nullptr,
		0, nullptr,
		1, &vkImageMemoryBarrier
	);

	endSingleTimeCommands(vkCommandBuffer);
}

VkCommandBuffer VulkanRenderer::beginSingleTimeCommands()
{
	VkCommandBufferAllocateInfo vkCommandBufferAllocateInfo{};
	vkCommandBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	vkCommandBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	vkCommandBufferAllocateInfo.commandPool = _vkCommandPool;
	vkCommandBufferAllocateInfo.commandBufferCount = 1;

	VkCommandBuffer vkCommandBuffer;
	vkAllocateCommandBuffers(_vkDevice, &vkCommandBufferAllocateInfo, &vkCommandBuffer);

	VkCommandBufferBeginInfo vkCommandBufferBeginInfo{};
	vkCommandBufferBeginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	vkCommandBufferBeginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

	vkBeginCommandBuffer(vkCommandBuffer, &vkCommandBufferBeginInfo);

	return vkCommandBuffer;
}

bool VulkanRenderer::hasStencilComponent(VkFormat format)
{
	return format == VK_FORMAT_D32_SFLOAT_S8_UINT || format == VK_FORMAT_D24_UNORM_S8_UINT;
}

void VulkanRenderer::endSingleTimeCommands(VkCommandBuffer commandBuffer)
{
	//vkEndCommandBuffer(commandBuffer);


	VkResult res = vkEndCommandBuffer(commandBuffer);
	if (res != VK_SUCCESS) {
		std::cerr << "endSingleTimeCommands: vkEndCommandBuffer failed, VkResult = " << res << "\n";
	}


	VkSubmitInfo vkSubmitInfo{};
	vkSubmitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	vkSubmitInfo.commandBufferCount = 1;
	vkSubmitInfo.pCommandBuffers = &commandBuffer;

	//vkQueueSubmit(_vkGraphicsQueue, 1, &vkSubmitInfo, VK_NULL_HANDLE);


	res = vkQueueSubmit(_vkGraphicsQueue, 1, &vkSubmitInfo, VK_NULL_HANDLE);
	if (res != VK_SUCCESS) {
		std::cerr << "endSingleTimeCommands: vkQueueSubmit failed, VkResult = " << res << "\n";
	}

	//vkQueueWaitIdle(_vkGraphicsQueue);


	res = vkQueueWaitIdle(_vkGraphicsQueue);
	if (res != VK_SUCCESS) {
		std::cerr << "endSingleTimeCommands: vkQueueWaitIdle failed, VkResult = " << res << "\n";
	}

	vkFreeCommandBuffers(_vkDevice, _vkCommandPool, 1, &commandBuffer);
}

void VulkanRenderer::createFramebuffers()
{
	_vkSwapChainFramebuffers.resize(_vkSwapChainImageViews.size());

	for (size_t i = 0; i < _vkSwapChainImageViews.size(); i++) 
	{

		std::array<VkImageView, 3> attachments = {
			_vkColorImageView,
			_vkDepthImageView,
			_vkSwapChainImageViews[i]
		};


		VkFramebufferCreateInfo vkFramebufferInfo{};
		vkFramebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		vkFramebufferInfo.renderPass = _vkRenderPass;
		vkFramebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());;
		vkFramebufferInfo.pAttachments = attachments.data();
		vkFramebufferInfo.width = _vkSwapChainExtent.width;
		vkFramebufferInfo.height = _vkSwapChainExtent.height;
		vkFramebufferInfo.layers = 1;

		if (vkCreateFramebuffer(_vkDevice, &vkFramebufferInfo, nullptr, &_vkSwapChainFramebuffers[i]) != VK_SUCCESS) {
			throw std::runtime_error("ERROR::VULKAN::FRAMEBUFFER::FAILED_TO_CREATE_FRAMEBUFFER\n");
		}

	}
}

void VulkanRenderer::submitRenderData(const std::vector<engine::mesh::Vertex>& vertices, const std::vector<unsigned int>& indices)
{
	/* vertices */
	VkDeviceSize vkVertexBufferSize = sizeof(engine::mesh::Vertex) * vertices.size();

	VkBuffer vkVertexStagingBuffer;
	VkDeviceMemory vkVertexDeviceStagingBufferMemory;

	createBuffer(vkVertexBufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
		, vkVertexStagingBuffer, vkVertexDeviceStagingBufferMemory);

	/* filling */
	void* vertexData;
	vkMapMemory(_vkDevice, vkVertexDeviceStagingBufferMemory, 0, vkVertexBufferSize, 0, &vertexData);
	memcpy(vertexData, vertices.data(), (size_t)vkVertexBufferSize);
	vkUnmapMemory(_vkDevice, vkVertexDeviceStagingBufferMemory);

	createBuffer(vkVertexBufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
		, _vkVertexBuffer, _vkVertexBufferMemory);

	copyBuffer(vkVertexStagingBuffer, _vkVertexBuffer, vkVertexBufferSize);

	vkDestroyBuffer(_vkDevice, vkVertexStagingBuffer, nullptr);
	vkFreeMemory(_vkDevice, vkVertexDeviceStagingBufferMemory, nullptr);

	/* indices */
	VkDeviceSize vkIndexBufferSize = sizeof(unsigned int) * indices.size();

	VkBuffer vkIndexStagingBuffer;
	VkDeviceMemory vkIndexDeviceStagingBufferMemory;

	createBuffer(vkIndexBufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
		, vkIndexStagingBuffer, vkIndexDeviceStagingBufferMemory);

	void* indexData;
	vkMapMemory(_vkDevice, vkIndexDeviceStagingBufferMemory, 0, vkIndexBufferSize, 0, &indexData);
	memcpy(indexData, indices.data(), (size_t)vkIndexBufferSize);
	vkUnmapMemory(_vkDevice, vkIndexDeviceStagingBufferMemory);

	createBuffer(vkIndexBufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
		, _vkIndexBuffer, _vkIndexBufferMemory);

	copyBuffer(vkIndexStagingBuffer, _vkIndexBuffer, vkIndexBufferSize);

	vkDestroyBuffer(_vkDevice, vkIndexStagingBuffer, nullptr);
	vkFreeMemory(_vkDevice, vkIndexDeviceStagingBufferMemory, nullptr);
}

void VulkanRenderer::submitRTRenderData(const std::vector<engine::mesh::Vertex>& vertices, const std::vector<unsigned int>& indices)
{

	vkDeviceWaitIdle(_vkDevice);

	/* vertices */
	VkDeviceSize vkVertexBufferSize =
		sizeof(engine::mesh::Vertex) * vertices.size();

	_vkRtVertexCount = static_cast<uint32_t>(vertices.size());
	_vkRtIndexCount = static_cast<uint32_t>(indices.size());

	// Destroy previous RT vertex buffer if any
	if (_vkRtVertexBuffer != VK_NULL_HANDLE) {
		vkDestroyBuffer(_vkDevice, _vkRtVertexBuffer, nullptr);
		vkFreeMemory(_vkDevice, _vkRtVertexBufferMemory, nullptr);
		_vkRtVertexBuffer = VK_NULL_HANDLE;
		_vkRtVertexBufferMemory = VK_NULL_HANDLE;
	}

	VkBuffer vkVertexStagingBuffer;
	VkDeviceMemory vkVertexDeviceStagingBufferMemory;

	createBuffer(
		vkVertexBufferSize,
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		vkVertexStagingBuffer,
		vkVertexDeviceStagingBufferMemory);

	/* filling */
	void* vertexData;
	vkMapMemory(_vkDevice, vkVertexDeviceStagingBufferMemory, 0,
		vkVertexBufferSize, 0, &vertexData);
	memcpy(vertexData, vertices.data(), (size_t)vkVertexBufferSize);
	vkUnmapMemory(_vkDevice, vkVertexDeviceStagingBufferMemory);

	createBuffer(
		vkVertexBufferSize,
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
		_vkRtVertexBuffer,
		_vkRtVertexBufferMemory);

	copyBuffer(vkVertexStagingBuffer, _vkRtVertexBuffer, vkVertexBufferSize);

	vkDestroyBuffer(_vkDevice, vkVertexStagingBuffer, nullptr);
	vkFreeMemory(_vkDevice, vkVertexDeviceStagingBufferMemory, nullptr);

	/* indices */
	VkDeviceSize vkIndexBufferSize =
		sizeof(unsigned int) * indices.size();

	// Destroy previous RT index buffer if any
	if (_vkRtIndexBuffer != VK_NULL_HANDLE) {
		vkDestroyBuffer(_vkDevice, _vkRtIndexBuffer, nullptr);
		vkFreeMemory(_vkDevice, _vkRtIndexBufferMemory, nullptr);
		_vkRtIndexBuffer = VK_NULL_HANDLE;
		_vkRtIndexBufferMemory = VK_NULL_HANDLE;
	}

	VkBuffer vkIndexStagingBuffer;
	VkDeviceMemory vkIndexDeviceStagingBufferMemory;

	createBuffer(
		vkIndexBufferSize,
		VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		vkIndexStagingBuffer,
		vkIndexDeviceStagingBufferMemory);

	void* indexData;
	vkMapMemory(_vkDevice, vkIndexDeviceStagingBufferMemory, 0,
		vkIndexBufferSize, 0, &indexData);
	memcpy(indexData, indices.data(), (size_t)vkIndexBufferSize);
	vkUnmapMemory(_vkDevice, vkIndexDeviceStagingBufferMemory);

	createBuffer(
		vkIndexBufferSize,
		VK_BUFFER_USAGE_TRANSFER_DST_BIT |
		VK_BUFFER_USAGE_INDEX_BUFFER_BIT |
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
		_vkRtIndexBuffer,
		_vkRtIndexBufferMemory);

	copyBuffer(vkIndexStagingBuffer, _vkRtIndexBuffer, vkIndexBufferSize);

	vkDestroyBuffer(_vkDevice, vkIndexStagingBuffer, nullptr);
	vkFreeMemory(_vkDevice, vkIndexDeviceStagingBufferMemory, nullptr);

	// make sure nothing is using the old RT buffers
	vkDeviceWaitIdle(_vkDevice);


	//destroy previous mesh BLAS if any
	if (_vkMeshBlas != VK_NULL_HANDLE) {
		_vkDestroyAccelerationStructureKHR_PFN(_vkDevice, _vkMeshBlas, nullptr);
		_vkMeshBlas = VK_NULL_HANDLE;
	}
	if (_vkMeshBlasBuffer != VK_NULL_HANDLE) {
		vkDestroyBuffer(_vkDevice, _vkMeshBlasBuffer, nullptr);
		_vkMeshBlasBuffer = VK_NULL_HANDLE;
	}
	if (_vkMeshBlasMemory != VK_NULL_HANDLE) {
		vkFreeMemory(_vkDevice, _vkMeshBlasMemory, nullptr);
		_vkMeshBlasMemory = VK_NULL_HANDLE;
	}

	const uint32_t vertexCount = static_cast<uint32_t>(vertices.size());
	const uint32_t indexCount = static_cast<uint32_t>(indices.size());
	const uint32_t primitiveCount = indexCount / 3; // triangles

	// Device addresses for the GPU vertex/index buffers
	VkBufferDeviceAddressInfo addrInfo{};
	addrInfo.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;

	addrInfo.buffer = _vkRtVertexBuffer;
	VkDeviceAddress vertexAddress = vkGetBufferDeviceAddress(_vkDevice, &addrInfo);

	addrInfo.buffer = _vkRtIndexBuffer;
	VkDeviceAddress indexAddress = vkGetBufferDeviceAddress(_vkDevice, &addrInfo);

	//Describe triangle geometry for BLAS build
	VkAccelerationStructureGeometryTrianglesDataKHR triangles{};
	triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
	triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;         // position format
	triangles.vertexData.deviceAddress = vertexAddress;
	triangles.vertexStride = sizeof(engine::mesh::Vertex);        // assumes position at start
	triangles.maxVertex = vertexCount;
	triangles.indexType = VK_INDEX_TYPE_UINT32;
	triangles.indexData.deviceAddress = indexAddress;
	triangles.transformData.deviceAddress = 0;                    // no per-primitive transform

	VkAccelerationStructureGeometryKHR asGeom{};
	asGeom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
	asGeom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
	asGeom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
	asGeom.geometry.triangles = triangles;

	VkAccelerationStructureBuildGeometryInfoKHR buildInfo{};
	buildInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
	buildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
	buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
	buildInfo.geometryCount = 1;
	buildInfo.pGeometries = &asGeom;

	VkAccelerationStructureBuildSizesInfoKHR sizeInfo{};
	sizeInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;

	_vkGetAccelerationStructureBuildSizesKHR_PFN(
		_vkDevice,
		VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
		&buildInfo,
		&primitiveCount,
		&sizeInfo);

	// Allocate buffer for the BLAS itself
	createBuffer(
		sizeInfo.accelerationStructureSize,
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
		_vkMeshBlasBuffer,
		_vkMeshBlasMemory);

	VkAccelerationStructureCreateInfoKHR asCreate{};
	asCreate.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
	asCreate.buffer = _vkMeshBlasBuffer;
	asCreate.offset = 0;
	asCreate.size = sizeInfo.accelerationStructureSize;
	asCreate.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;

	if (_vkCreateAccelerationStructureKHR_PFN(
		_vkDevice, &asCreate, nullptr, &_vkMeshBlas) != VK_SUCCESS)
	{
		throw std::runtime_error("ERROR::VULKAN::RT::FAILED_TO_CREATE_MESH_BLAS");
	}

	// Scratch buffer for BLAS build
	VkBuffer scratchBuffer;
	VkDeviceMemory scratchMemory;

	createBuffer(
		sizeInfo.buildScratchSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
		scratchBuffer,
		scratchMemory);

	addrInfo.buffer = scratchBuffer;
	VkDeviceAddress scratchAddress = vkGetBufferDeviceAddress(_vkDevice, &addrInfo);

	/// Build BLAS on the device
	buildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
	buildInfo.dstAccelerationStructure = _vkMeshBlas;
	buildInfo.scratchData.deviceAddress = scratchAddress;

	VkAccelerationStructureBuildRangeInfoKHR rangeInfo{};
	rangeInfo.primitiveCount = primitiveCount;
	rangeInfo.primitiveOffset = 0;
	rangeInfo.firstVertex = 0;
	rangeInfo.transformOffset = 0;

	VkAccelerationStructureBuildRangeInfoKHR* pRangeInfo = &rangeInfo;

	VkCommandBuffer cmd_blas = beginSingleTimeCommands();

	_vkCmdBuildAccelerationStructuresKHR_PFN(
		cmd_blas,
		1,
		&buildInfo,
		&pRangeInfo);

	endSingleTimeCommands(cmd_blas);

	//Get BLAS device address (for TLAS instances later)
	VkAccelerationStructureDeviceAddressInfoKHR asAddr{};
	asAddr.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
	asAddr.accelerationStructure = _vkMeshBlas;

	_vkMeshBlasDeviceAddress =
		_vkGetAccelerationStructureDeviceAddressKHR_PFN(_vkDevice, &asAddr);

	// Free scratch buffer (BLAS is now self-contained)
	vkDestroyBuffer(_vkDevice, scratchBuffer, nullptr);
	vkFreeMemory(_vkDevice, scratchMemory, nullptr);

	// === Build a TLAS that references the mesh BLAS ===

	// Destroy previous mesh TLAS if any
	if (_vkMeshTlas != VK_NULL_HANDLE) {
		_vkDestroyAccelerationStructureKHR_PFN(_vkDevice, _vkMeshTlas, nullptr);
		_vkMeshTlas = VK_NULL_HANDLE;
	}
	if (_vkMeshTlasBuffer != VK_NULL_HANDLE) {
		vkDestroyBuffer(_vkDevice, _vkMeshTlasBuffer, nullptr);
		_vkMeshTlasBuffer = VK_NULL_HANDLE;
	}
	if (_vkMeshTlasMemory != VK_NULL_HANDLE) {
		vkFreeMemory(_vkDevice, _vkMeshTlasMemory, nullptr);
		_vkMeshTlasMemory = VK_NULL_HANDLE;
	}

	// Create TLAS instance referencing _vkMeshBlas
	VkAccelerationStructureInstanceKHR instance{};
	// identity transform, row-major 3x4
	instance.transform.matrix[0][0] = 1.0f;
	instance.transform.matrix[0][1] = 0.0f;
	instance.transform.matrix[0][2] = 0.0f;
	instance.transform.matrix[0][3] = 0.0f;

	instance.transform.matrix[1][0] = 0.0f;
	instance.transform.matrix[1][1] = 1.0f;
	instance.transform.matrix[1][2] = 0.0f;
	instance.transform.matrix[1][3] = 0.0f;

	instance.transform.matrix[2][0] = 0.0f;
	instance.transform.matrix[2][1] = 0.0f;
	instance.transform.matrix[2][2] = 1.0f;
	instance.transform.matrix[2][3] = 0.0f;

	instance.instanceCustomIndex = 0;
	instance.mask = 0xFF;
	instance.instanceShaderBindingTableRecordOffset = 0;
	instance.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
	instance.accelerationStructureReference = _vkMeshBlasDeviceAddress;

	// Instance buffer
	VkBuffer       instanceBuffer;
	VkDeviceMemory instanceMemory;
	VkDeviceSize   instanceSize = sizeof(VkAccelerationStructureInstanceKHR);

	createBuffer(
		instanceSize,
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		instanceBuffer,
		instanceMemory
	);

	void* mapped = nullptr;
	vkMapMemory(_vkDevice, instanceMemory, 0, instanceSize, 0, &mapped);
	memcpy(mapped, &instance, sizeof(instance));
	vkUnmapMemory(_vkDevice, instanceMemory);

	//Get device address of the instance buffer
	VkBufferDeviceAddressInfo addrInfoTLAS{};
	addrInfoTLAS.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
	addrInfoTLAS.buffer = instanceBuffer;
	VkDeviceAddress instanceAddress = vkGetBufferDeviceAddress(_vkDevice, &addrInfoTLAS);

	// Describe TLAS geometry (instances)
	VkAccelerationStructureGeometryInstancesDataKHR instData{};
	instData.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
	instData.arrayOfPointers = VK_FALSE;
	instData.data.deviceAddress = instanceAddress;

	VkAccelerationStructureGeometryKHR tlasGeom{};
	tlasGeom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
	tlasGeom.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
	tlasGeom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
	tlasGeom.geometry.instances = instData;

	VkAccelerationStructureBuildGeometryInfoKHR tlasBuildInfo{};
	tlasBuildInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
	tlasBuildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
	tlasBuildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
	tlasBuildInfo.geometryCount = 1;
	tlasBuildInfo.pGeometries = &tlasGeom;

	uint32_t tlasPrimitiveCount = 1; // one instance

	VkAccelerationStructureBuildSizesInfoKHR tlasSize{};
	tlasSize.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;

	_vkGetAccelerationStructureBuildSizesKHR_PFN(
		_vkDevice,
		VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
		&tlasBuildInfo,
		&tlasPrimitiveCount,
		&tlasSize
	);

	/// Create TLAS buffer + TLAS object
	createBuffer(
		tlasSize.accelerationStructureSize,
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
		_vkMeshTlasBuffer,
		_vkMeshTlasMemory
	);

	VkAccelerationStructureCreateInfoKHR tlasCreate{};
	tlasCreate.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
	tlasCreate.buffer = _vkMeshTlasBuffer;
	tlasCreate.offset = 0;
	tlasCreate.size = tlasSize.accelerationStructureSize;
	tlasCreate.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;

	if (_vkCreateAccelerationStructureKHR_PFN(
		_vkDevice, &tlasCreate, nullptr, &_vkMeshTlas) != VK_SUCCESS)
	{
		throw std::runtime_error("ERROR::VULKAN::RT::FAILED_TO_CREATE_MESH_TLAS");
	}

	//Scratch buffer for TLAS build
	VkBuffer       tlasScratchBuffer;
	VkDeviceMemory tlasScratchMemory;

	createBuffer(
		tlasSize.buildScratchSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
		tlasScratchBuffer,
		tlasScratchMemory
	);

	VkBufferDeviceAddressInfo addrInfoScratch{};
	addrInfoScratch.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
	addrInfoScratch.buffer = tlasScratchBuffer;
	VkDeviceAddress tlasScratchAddress =
		vkGetBufferDeviceAddress(_vkDevice, &addrInfoScratch);

	tlasBuildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
	tlasBuildInfo.dstAccelerationStructure = _vkMeshTlas;
	tlasBuildInfo.scratchData.deviceAddress = tlasScratchAddress;

	VkAccelerationStructureBuildRangeInfoKHR tlasRange{};
	tlasRange.primitiveCount = tlasPrimitiveCount;
	tlasRange.primitiveOffset = 0;
	tlasRange.firstVertex = 0;
	tlasRange.transformOffset = 0;

	VkAccelerationStructureBuildRangeInfoKHR* pTlasRange = &tlasRange;

	VkCommandBuffer cmd_tlas = beginSingleTimeCommands();

	_vkCmdBuildAccelerationStructuresKHR_PFN(
		cmd_tlas,
		1,
		&tlasBuildInfo,
		&pTlasRange
	);

	endSingleTimeCommands(cmd_tlas);

	//Get TLAS device address (optional but nice)
	VkAccelerationStructureDeviceAddressInfoKHR tlasAddr{};
	tlasAddr.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
	tlasAddr.accelerationStructure = _vkMeshTlas;

	_vkMeshTlasDeviceAddress =
		_vkGetAccelerationStructureDeviceAddressKHR_PFN(_vkDevice, &tlasAddr);

	//Clean up TLAS scratch + instance buffer
	vkDestroyBuffer(_vkDevice, tlasScratchBuffer, nullptr);
	vkFreeMemory(_vkDevice, tlasScratchMemory, nullptr);

	vkDestroyBuffer(_vkDevice, instanceBuffer, nullptr);
	vkFreeMemory(_vkDevice, instanceMemory, nullptr);

}

void VulkanRenderer::createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
	VkMemoryPropertyFlags properties, VkBuffer& buffer, VkDeviceMemory& bufferMemory)
{

	VkBufferCreateInfo bufferInfo{};
	bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferInfo.size = size;
	bufferInfo.usage = usage;
	bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

	if (vkCreateBuffer(_vkDevice, &bufferInfo, nullptr, &buffer) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::VERTEX_BUFFER::FIALED_TO_CREATE_VERTEX_BUFFER\n");

	}

	VkMemoryRequirements memRequirements;
	vkGetBufferMemoryRequirements(_vkDevice, buffer, &memRequirements);

	VkMemoryAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.allocationSize = memRequirements.size;
	allocInfo.memoryTypeIndex = getMemoryType(memRequirements.memoryTypeBits, properties);

	VkMemoryAllocateFlagsInfo allocFlags{};
	if (usage & VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) {
		allocFlags.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
		allocFlags.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT;
		allocFlags.deviceMask = 0x1; // single-GPU

		allocInfo.pNext = &allocFlags;
	}
	else {
		allocInfo.pNext = nullptr;
	}

	if (vkAllocateMemory(_vkDevice, &allocInfo, nullptr, &bufferMemory) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::MEMORY::FAILED_TO_ALOCATE_VERTEX_BUFFER_MEMORY\n");
	}
	/*bind memory to use */
	vkBindBufferMemory(_vkDevice, buffer, bufferMemory, 0);
}

void VulkanRenderer::copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size)
{
	VkCommandBuffer vkCommandBuffer = beginSingleTimeCommands();

	VkBufferCopy vkCopyRegion{};
	vkCopyRegion.size = size;
	vkCmdCopyBuffer(vkCommandBuffer, srcBuffer, dstBuffer, 1, &vkCopyRegion);

	endSingleTimeCommands(vkCommandBuffer);

}

void VulkanRenderer::createUniformBuffers()
{
	VkDeviceSize vkBufferSize = sizeof(engine::math::Mat4f);
	VkDeviceSize vkRtBufferSize = sizeof(engine::vk::RtSamples);

	_vkUniformBuffers.resize(_MAX_FRAMES_IN_FLIGHT);
	_vkUniformBuffersMemory.resize(_MAX_FRAMES_IN_FLIGHT);
	_vkUniformBuffersMapped.resize(_MAX_FRAMES_IN_FLIGHT);

	_vkRtSampleUniform.resize(_MAX_FRAMES_IN_FLIGHT);
	_vkRtUniformBuffersMemory.resize(_MAX_FRAMES_IN_FLIGHT);
	_vkRtUniformBuffersMapped.resize(_MAX_FRAMES_IN_FLIGHT);

	for (size_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; i++) 
	{
		createBuffer(vkBufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
			, _vkUniformBuffers[i], _vkUniformBuffersMemory[i]);
		vkMapMemory(_vkDevice, _vkUniformBuffersMemory[i], 0, vkBufferSize, 0, &_vkUniformBuffersMapped[i]);


		createBuffer(vkRtBufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
			, _vkRtSampleUniform[i], _vkRtUniformBuffersMemory[i]);
		vkMapMemory(_vkDevice, _vkRtUniformBuffersMemory[i], 0, vkRtBufferSize, 0, &_vkRtUniformBuffersMapped[i]);
	}
}

void VulkanRenderer::createDescriptorPool()
{
	std::array<VkDescriptorPoolSize, 6> vkDescriptorPoolSize{};

	vkDescriptorPoolSize[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	vkDescriptorPoolSize[0].descriptorCount = static_cast<uint32_t>(_MAX_FRAMES_IN_FLIGHT);

	//  RT camera UBO
	vkDescriptorPoolSize[1].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	vkDescriptorPoolSize[1].descriptorCount = _MAX_FRAMES_IN_FLIGHT;

	vkDescriptorPoolSize[2].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	
	// Increase Storage buffer for RT (bindings 2, 5, 6 ,7)
	vkDescriptorPoolSize[2].descriptorCount = static_cast<uint32_t>(4 * _MAX_FRAMES_IN_FLIGHT);

	// RT storage image for rtColor
	vkDescriptorPoolSize[3].type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
	vkDescriptorPoolSize[3].descriptorCount = static_cast<uint32_t>(_MAX_FRAMES_IN_FLIGHT);
	
	// for TLAS
	vkDescriptorPoolSize[4].type = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
	vkDescriptorPoolSize[4].descriptorCount = static_cast<uint32_t>(_MAX_FRAMES_IN_FLIGHT);

	// ray samples
	vkDescriptorPoolSize[5].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	vkDescriptorPoolSize[5].descriptorCount = static_cast<uint32_t>(_MAX_FRAMES_IN_FLIGHT);


	VkDescriptorPoolCreateInfo vkDescriptorPoolCreateInfo{};
	vkDescriptorPoolCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	vkDescriptorPoolCreateInfo.poolSizeCount = static_cast<uint32_t>(vkDescriptorPoolSize.size());
	vkDescriptorPoolCreateInfo.pPoolSizes = vkDescriptorPoolSize.data();
	vkDescriptorPoolCreateInfo.maxSets = static_cast<uint32_t>(_MAX_FRAMES_IN_FLIGHT);


	if (vkCreateDescriptorPool(_vkDevice, &vkDescriptorPoolCreateInfo, nullptr, &_vkDescriptorPool) != VK_SUCCESS)
	{
		throw std::runtime_error("ERROR::VULKAN::DESCRIPTOR::FAILED_TO_CREATE_DESCRIPTOR_POOL\n");
	}
}

void VulkanRenderer::createUIDescriptorPool()
{
	std::array<VkDescriptorPoolSize, 1> vkDescriptorPoolSize{};

	vkDescriptorPoolSize[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
	vkDescriptorPoolSize[0].descriptorCount = static_cast<uint32_t>(_MAX_FRAMES_IN_FLIGHT);


	VkDescriptorPoolCreateInfo vkDescriptorPoolCreateInfo{};
	vkDescriptorPoolCreateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	vkDescriptorPoolCreateInfo.poolSizeCount = static_cast<uint32_t>(vkDescriptorPoolSize.size());
	vkDescriptorPoolCreateInfo.pPoolSizes = vkDescriptorPoolSize.data();
	vkDescriptorPoolCreateInfo.maxSets = static_cast<uint32_t>(_MAX_FRAMES_IN_FLIGHT);
	vkDescriptorPoolCreateInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;

	if (vkCreateDescriptorPool(_vkDevice, &vkDescriptorPoolCreateInfo, nullptr, &_vkDescriptorPoolUI) != VK_SUCCESS)
	{
		throw std::runtime_error("ERROR::VULKAN::DESCRIPTOR::FAILED_TO_CREATE_DESCRIPTOR_POOL_UI\n");
	}
}

void VulkanRenderer::createDescriptorSets()
{
	std::vector<VkDescriptorSetLayout> vDescriptorSetLayout(_MAX_FRAMES_IN_FLIGHT, _vkDescriptorSetLayout);

	VkDescriptorSetAllocateInfo vkDescriptorSetAllocateInfo{};
	vkDescriptorSetAllocateInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	vkDescriptorSetAllocateInfo.descriptorPool = _vkDescriptorPool;
	vkDescriptorSetAllocateInfo.descriptorSetCount = static_cast<uint32_t>(_MAX_FRAMES_IN_FLIGHT);
	vkDescriptorSetAllocateInfo.pSetLayouts = vDescriptorSetLayout.data();

	_vkDescriptorSets.resize(_MAX_FRAMES_IN_FLIGHT);

	if (vkAllocateDescriptorSets(_vkDevice, &vkDescriptorSetAllocateInfo, _vkDescriptorSets.data()) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::DESCRIPTOR::FAILED_TO_ALLOCATE_DESCRIPTOR_SETS\n");
	}

	for (size_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; i++) {

		VkDescriptorBufferInfo vkDescriptorBufferInfo{};
		vkDescriptorBufferInfo.buffer = _vkUniformBuffers[i];
		vkDescriptorBufferInfo.offset = 0;
		vkDescriptorBufferInfo.range = sizeof(engine::math::Mat4f);

		//  RT camera buffer info
		VkDescriptorBufferInfo vkDescriptorRtCameraBufferInfo{};
		vkDescriptorRtCameraBufferInfo.buffer = _vkRtCameraBuffers[i];
		vkDescriptorRtCameraBufferInfo.offset = 0;
		vkDescriptorRtCameraBufferInfo.range = sizeof(engine::vk::RtCameraUBO);

		VkDescriptorBufferInfo vkDescriptorBufferInfoSSBO{};
		vkDescriptorBufferInfoSSBO.buffer = _perDraw[i].ssbo;
		vkDescriptorBufferInfoSSBO.offset = 0;
		vkDescriptorBufferInfoSSBO.range = VK_WHOLE_SIZE;

		//  image info for RT Color
		VkDescriptorImageInfo  vkDescriptorBufferInfoRt{};
		vkDescriptorBufferInfoRt.imageLayout = VK_IMAGE_LAYOUT_GENERAL; // use it as storage image
		vkDescriptorBufferInfoRt.imageView = _vkRtColorImageView;
		vkDescriptorBufferInfoRt.sampler = VK_NULL_HANDLE; 

		// Acceleration structure info (TLAS)
		VkWriteDescriptorSetAccelerationStructureKHR vkAccelInfo{};
		vkAccelInfo.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR;
		vkAccelInfo.accelerationStructureCount = 1;
		vkAccelInfo.pAccelerationStructures = &_vkTestTlas;

		//  RT Samples
		VkDescriptorBufferInfo vkDescriptorRTSamples{};
		vkDescriptorRTSamples.buffer = _vkRtSampleUniform[i];
		vkDescriptorRTSamples.offset = 0;
		vkDescriptorRTSamples.range = sizeof(engine::vk::RtSamples);

		std::array<VkWriteDescriptorSet, 6> vVkDescriptorWrites{};

		vVkDescriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		vVkDescriptorWrites[0].dstSet = _vkDescriptorSets[i];
		vVkDescriptorWrites[0].dstBinding = 0;
		vVkDescriptorWrites[0].dstArrayElement = 0;
		vVkDescriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		vVkDescriptorWrites[0].descriptorCount = 1;
		vVkDescriptorWrites[0].pBufferInfo = &vkDescriptorBufferInfo;


		// binding 1: RT camera UBO
		vVkDescriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		vVkDescriptorWrites[1].dstSet = _vkDescriptorSets[i];
		vVkDescriptorWrites[1].dstBinding = 1;
		vVkDescriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		vVkDescriptorWrites[1].descriptorCount = 1;
		vVkDescriptorWrites[1].pBufferInfo = &vkDescriptorRtCameraBufferInfo;

		vVkDescriptorWrites[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		vVkDescriptorWrites[2].dstSet = _vkDescriptorSets[i];
		vVkDescriptorWrites[2].dstBinding = 2; // <<< binding 2, matches shader
		vVkDescriptorWrites[2].dstArrayElement = 0;
		vVkDescriptorWrites[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		vVkDescriptorWrites[2].descriptorCount = 1;
		vVkDescriptorWrites[2].pBufferInfo = &vkDescriptorBufferInfoSSBO;

		// binding 3: RT storage image
		vVkDescriptorWrites[3].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		vVkDescriptorWrites[3].dstSet = _vkDescriptorSets[i];
		vVkDescriptorWrites[3].dstBinding = 3;
		vVkDescriptorWrites[3].dstArrayElement = 0;
		vVkDescriptorWrites[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
		vVkDescriptorWrites[3].descriptorCount = 1;
		vVkDescriptorWrites[3].pImageInfo = &vkDescriptorBufferInfoRt;

		// binding 4: TLAS
		vVkDescriptorWrites[4].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		vVkDescriptorWrites[4].pNext = &vkAccelInfo; // hook AS info
		vVkDescriptorWrites[4].dstSet = _vkDescriptorSets[i];
		vVkDescriptorWrites[4].dstBinding = 4;
		vVkDescriptorWrites[4].dstArrayElement = 0;
		vVkDescriptorWrites[4].descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
		vVkDescriptorWrites[4].descriptorCount = 1;
		vVkDescriptorWrites[4].pImageInfo = nullptr;
		vVkDescriptorWrites[4].pBufferInfo = nullptr;
		vVkDescriptorWrites[4].pTexelBufferView = nullptr;

		// binding 5: RT Sample
		vVkDescriptorWrites[5].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		vVkDescriptorWrites[5].pNext = nullptr; // hook AS info
		vVkDescriptorWrites[5].dstSet = _vkDescriptorSets[i];
		vVkDescriptorWrites[5].dstBinding = 8;
		vVkDescriptorWrites[5].dstArrayElement = 0;
		vVkDescriptorWrites[5].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		vVkDescriptorWrites[5].descriptorCount = 1;
		vVkDescriptorWrites[5].pImageInfo = nullptr;
		vVkDescriptorWrites[5].pBufferInfo = &vkDescriptorRTSamples;
		vVkDescriptorWrites[5].pTexelBufferView = nullptr;


		vkUpdateDescriptorSets(_vkDevice, static_cast<uint32_t>(vVkDescriptorWrites.size())
			, vVkDescriptorWrites.data(), 0, nullptr);
	}
}

void VulkanRenderer::createCommandBuffers()
{
	_vkCommandBuffers.resize(_MAX_FRAMES_IN_FLIGHT);

	VkCommandBufferAllocateInfo vkAllocInfo{};
	vkAllocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	vkAllocInfo.commandPool = _vkCommandPool;
	vkAllocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	vkAllocInfo.commandBufferCount = (uint32_t)_vkCommandBuffers.size();

	if (vkAllocateCommandBuffers(_vkDevice, &vkAllocInfo, _vkCommandBuffers.data()) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::COMMANDBUFFER::FAILED_TO_CREATE_COMMAND_BUFFER\n");
	}
}

void VulkanRenderer::createSyncObjects()
{
	_vkImageAvailableSemaphores.resize(_MAX_FRAMES_IN_FLIGHT);
	_vkInFlightFences.resize(_MAX_FRAMES_IN_FLIGHT);
	_vkRenderFinishedSemaphores.resize(_vkSwapChainImages.size());

	// one fence slot per swapchain image
	_vkImagesInFlight.resize(_vkSwapChainImages.size(), VK_NULL_HANDLE);

	VkSemaphoreCreateInfo vkSemaphoreInfo{};
	vkSemaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

	VkFenceCreateInfo vkFenceInfo{};
	vkFenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	vkFenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;


	for (size_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; i++) {
		if (vkCreateSemaphore(_vkDevice, &vkSemaphoreInfo, nullptr, &_vkImageAvailableSemaphores[i]) != VK_SUCCESS ||
			vkCreateFence(_vkDevice, &vkFenceInfo, nullptr, &_vkInFlightFences[i]) != VK_SUCCESS) {
			throw std::runtime_error("ERROR::VULKAN::SYNC_OBJECTS::FAILED_TO_CREATE_SYNC_OBJECTS\n");
		}
	}

	for (size_t i = 0; i < _vkSwapChainImages.size(); i++)
	{
		if (vkCreateSemaphore(_vkDevice, &vkSemaphoreInfo, nullptr, &_vkRenderFinishedSemaphores[i]) != VK_SUCCESS)
			throw std::runtime_error("ERROR::VULKAN::SYNC_OBJECTS::FAILED_TO_CREATE_SYNC_OBJECTS\n");
	}
}

void VulkanRenderer::createIndirectBuffers() {
	_indirectPerFrame.resize(_MAX_FRAMES_IN_FLIGHT);
	for (uint32_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; ++i) {
		createBuffer(_INDIRECT_BYTES,
			VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			_indirectPerFrame[i].buf, _indirectPerFrame[i].mem);

		vkMapMemory(_vkDevice, _indirectPerFrame[i].mem, 0, _INDIRECT_BYTES, 0,
			&_indirectPerFrame[i].mapped);
	}
}

void  VulkanRenderer::createSSBO()
{
	const VkDeviceSize perDrawBytes = _MAX_INDIRECT_DRAWS * sizeof(engine::vk::PerDrawGPU);

	for (uint32_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; ++i) {
		createBuffer(perDrawBytes,
			VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			_perDraw[i].ssbo, _perDraw[i].mem);

		vkMapMemory(_vkDevice, _perDraw[i].mem, 0, perDrawBytes, 0, &_perDraw[i].mapped);
		_perDraw[i].capacity = _MAX_INDIRECT_DRAWS;
	}

}

void VulkanRenderer::queryRayTracingProperties()
{
	VkPhysicalDeviceProperties2 props2{};
	props2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
	props2.pNext = nullptr;

	// Hook RT pipeline properties into the pNext chain
	_vkRtPipelineProps.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR;
	_vkRtPipelineProps.pNext = nullptr;

	props2.pNext = &_vkRtPipelineProps;

	vkGetPhysicalDeviceProperties2(_vkPhysicalDevice, &props2);
	/*
	std::cout
		<< "RT props: handleSize=" << _vkRtPipelineProps.shaderGroupHandleSize
		<< ", handleAlign=" << _vkRtPipelineProps.shaderGroupHandleAlignment
		<< ", baseAlign=" << _vkRtPipelineProps.shaderGroupBaseAlignment
		<< std::endl;
	*/
}

void VulkanRenderer::createRayTracingSBT()
{
	// Destroy old SBT if any
	if (_vkRtSbtBuffer != VK_NULL_HANDLE) {
		vkDestroyBuffer(_vkDevice, _vkRtSbtBuffer, nullptr);
		vkFreeMemory(_vkDevice, _vkRtSbtMemory, nullptr);
		_vkRtSbtBuffer = VK_NULL_HANDLE;
		_vkRtSbtMemory = VK_NULL_HANDLE;
	}

	// 0 = raygen
	// 1 = radiance miss
	// 2 = shadow miss
	// 3 = radiance hit
	// 4 = shadow hit

	const uint32_t groupCount = 5;

	const uint32_t handleSize = _vkRtPipelineProps.shaderGroupHandleSize;
	const uint32_t baseAlign = _vkRtPipelineProps.shaderGroupBaseAlignment;

	// Align record size to shaderGroupBaseAlignment
	const uint32_t handleSizeAligned = (handleSize + baseAlign - 1) & ~(baseAlign - 1);

	const uint32_t sbtSize = groupCount * handleSizeAligned;

	// Get raw group handles
	std::vector<uint8_t> handleStorage(groupCount * handleSize);

	VkResult res = _vlRtGetRayTracingShaderGroupHandlesKHR_PFN(
		_vkDevice,
		_vkRtPipeline,
		0,                         // firstGroup
		groupCount,
		static_cast<uint32_t>(handleStorage.size()),
		handleStorage.data()
	);

	if (res != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::RT::FAILED_TO_GET_SHADER_GROUP_HANDLES");
	}

	// Allocate SBT buffer
	VkBufferUsageFlags usage =
		VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;

	VkMemoryPropertyFlags memProps =
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
		VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;

	createBuffer(sbtSize, usage, memProps, _vkRtSbtBuffer, _vkRtSbtMemory);

	// Fill SBT: one record per group, in order
	uint8_t* pData = nullptr;
	vkMapMemory(_vkDevice, _vkRtSbtMemory, 0, sbtSize, 0, reinterpret_cast<void**>(&pData));

	for (uint32_t g = 0; g < groupCount; ++g) {
		memcpy(
			pData + g * handleSizeAligned,          // destination slot
			handleStorage.data() + g * handleSize,  // source handle
			handleSize
		);
	}

	vkUnmapMemory(_vkDevice, _vkRtSbtMemory);

	// Get SBT buffer device address
	VkBufferDeviceAddressInfo addrInfo{};
	addrInfo.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
	addrInfo.buffer = _vkRtSbtBuffer;

	VkDeviceAddress sbtAddress = vkGetBufferDeviceAddress(_vkDevice, &addrInfo);

	// Raygen table: group 0
	_vkSbtRaygenRegion.deviceAddress = sbtAddress + 0 * handleSizeAligned;
	_vkSbtRaygenRegion.stride = handleSizeAligned;
	_vkSbtRaygenRegion.size = handleSizeAligned;

	// Miss table: groups 1 (radiance miss) and 2 (shadow miss)
	_vkSbtMissRegion.deviceAddress = sbtAddress + 1 * handleSizeAligned;
	_vkSbtMissRegion.stride = handleSizeAligned;
	_vkSbtMissRegion.size = 2 * handleSizeAligned;

	// Hit table: groups 3 (radiance hit) and 4 (shadow hit)
	_vkSbtHitRegion.deviceAddress = sbtAddress + 3 * handleSizeAligned;
	_vkSbtHitRegion.stride = handleSizeAligned;
	_vkSbtHitRegion.size = 2 * handleSizeAligned;

	// No callables
	_vkSbtCallableRegion.deviceAddress = 0;
	_vkSbtCallableRegion.stride = 0;
	_vkSbtCallableRegion.size = 0;
}

void VulkanRenderer::createRtCameraBuffers()
{
	VkDeviceSize bufferSize = sizeof(engine::vk::RtCameraUBO);

	_vkRtCameraBuffers.resize(_MAX_FRAMES_IN_FLIGHT);
	_vkRtCameraBuffersMemory.resize(_MAX_FRAMES_IN_FLIGHT);
	_vkRtCameraBuffersMapped.resize(_MAX_FRAMES_IN_FLIGHT);

	for (size_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; ++i)
	{
		createBuffer(
			bufferSize,
			VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
			VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
			VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			_vkRtCameraBuffers[i],
			_vkRtCameraBuffersMemory[i]
		);

		vkMapMemory(
			_vkDevice,
			_vkRtCameraBuffersMemory[i],
			0,
			bufferSize,
			0,
			&_vkRtCameraBuffersMapped[i]
		);
	}
}

void VulkanRenderer::submitRtCameraData(const engine::vk::RtCameraUBO& data)
{
	_rtCameraHost = data;
}

void VulkanRenderer::buildClusterBlases(const std::vector<engine::vk::RtClusterBuildInfo>& clusters)
{
	// Clear any previous cluster BLASes
	for (auto& c : _rtClusterBlases)
	{
		if (c.blas != VK_NULL_HANDLE)
		{
			_vkDestroyAccelerationStructureKHR_PFN(_vkDevice, c.blas, nullptr);
			c.blas = VK_NULL_HANDLE;
		}
		if (c.blasBuffer != VK_NULL_HANDLE)
		{
			vkDestroyBuffer(_vkDevice, c.blasBuffer, nullptr);
			c.blasBuffer = VK_NULL_HANDLE;
		}
		if (c.blasMemory != VK_NULL_HANDLE)
		{
			vkFreeMemory(_vkDevice, c.blasMemory, nullptr);
			c.blasMemory = VK_NULL_HANDLE;
		}
	}
	_rtClusterBlases.clear();
	_rtClusterBlases.reserve(clusters.size());

	// Common: device addresses for global vertex/index buffers
	VkBufferDeviceAddressInfo addrInfo{};
	addrInfo.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;

	addrInfo.buffer = _vkRtVertexBuffer;
	VkDeviceAddress vertexAddress = vkGetBufferDeviceAddress(_vkDevice, &addrInfo);

	addrInfo.buffer = _vkRtIndexBuffer;
	VkDeviceAddress indexAddress = vkGetBufferDeviceAddress(_vkDevice, &addrInfo);

	const uint32_t vertexCount =
		static_cast<uint32_t>(_vkRtVertexCount); 

	for (const auto& info : clusters)
	{
		const uint32_t firstIndex = info.firstIndex;
		const uint32_t indexCount = info.indexCount;
		const uint32_t primitiveCnt = indexCount / 3;

		if (indexCount == 0 || primitiveCnt == 0)
			continue;

		// Triangle geometry for this cluster
		VkAccelerationStructureGeometryTrianglesDataKHR triangles{};
		triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
		triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
		triangles.vertexData.deviceAddress = vertexAddress;
		triangles.vertexStride = sizeof(engine::mesh::Vertex);
		triangles.maxVertex = vertexCount;
		triangles.indexType = VK_INDEX_TYPE_UINT32;
		// shift indexData to the start of this clusters index range
		triangles.indexData.deviceAddress = indexAddress + firstIndex * sizeof(uint32_t);
		triangles.transformData.deviceAddress = 0;

		VkAccelerationStructureGeometryKHR asGeom{};
		asGeom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
		asGeom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
		asGeom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
		asGeom.geometry.triangles = triangles;

		VkAccelerationStructureBuildGeometryInfoKHR buildInfo{};
		buildInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
		buildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
		buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
		buildInfo.geometryCount = 1;
		buildInfo.pGeometries = &asGeom;

		VkAccelerationStructureBuildSizesInfoKHR sizeInfo{};
		sizeInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;

		uint32_t primitiveCountArray[1] = { primitiveCnt };

		_vkGetAccelerationStructureBuildSizesKHR_PFN(
			_vkDevice,
			VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
			&buildInfo,
			primitiveCountArray,
			&sizeInfo);

		// Allocate buffer for this BLAS
		engine::vk::RtClusterBlas clusterBlas{};
		clusterBlas.firstIndex = firstIndex;
		clusterBlas.indexCount = indexCount;
		clusterBlas.clusterId = info.clusterId;

		VkBufferCreateInfo bufInfo{};
		bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
		bufInfo.size = sizeInfo.accelerationStructureSize;
		bufInfo.usage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
			VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
		bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

		if (vkCreateBuffer(_vkDevice, &bufInfo, nullptr, &clusterBlas.blasBuffer) != VK_SUCCESS)
		{
			std::cerr << "buildClusterBlases: Failed to create BLAS buffer\n";
			continue;
		}

		VkMemoryRequirements memReq{};
		vkGetBufferMemoryRequirements(_vkDevice, clusterBlas.blasBuffer, &memReq);

		VkMemoryAllocateFlagsInfo allocFlags{};
		allocFlags.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
		allocFlags.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT_KHR;

		VkMemoryAllocateInfo allocInfo{};
		allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
		allocInfo.pNext = &allocFlags;
		allocInfo.allocationSize = memReq.size;
		allocInfo.memoryTypeIndex = getMemoryType(
			memReq.memoryTypeBits,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

		if (vkAllocateMemory(_vkDevice, &allocInfo, nullptr, &clusterBlas.blasMemory) != VK_SUCCESS)
		{
			std::cerr << "buildClusterBlases: Failed to allocate BLAS memory\n";
			vkDestroyBuffer(_vkDevice, clusterBlas.blasBuffer, nullptr);
			continue;
		}

		vkBindBufferMemory(_vkDevice, clusterBlas.blasBuffer, clusterBlas.blasMemory, 0);

		//Create the acceleration structure object
		VkAccelerationStructureCreateInfoKHR asCreateInfo{};
		asCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
		asCreateInfo.buffer = clusterBlas.blasBuffer;
		asCreateInfo.size = sizeInfo.accelerationStructureSize;
		asCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;

		if (_vkCreateAccelerationStructureKHR_PFN(
			_vkDevice,
			&asCreateInfo,
			nullptr,
			&clusterBlas.blas) != VK_SUCCESS)
		{
			std::cerr << "buildClusterBlases: Failed to create BLAS handle\n";
			vkDestroyBuffer(_vkDevice, clusterBlas.blasBuffer, nullptr);
			vkFreeMemory(_vkDevice, clusterBlas.blasMemory, nullptr);
			continue;
		}

		//Scratch buffer for this BLAS build
		VkBuffer scratchBuffer;
		VkDeviceMemory scratchMemory;
		createBuffer(
			sizeInfo.buildScratchSize,
			VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
			VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
			VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
			scratchBuffer,
			scratchMemory);

		addrInfo.buffer = scratchBuffer;
		VkDeviceAddress scratchAddress = vkGetBufferDeviceAddress(_vkDevice, &addrInfo);

		buildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
		buildInfo.dstAccelerationStructure = clusterBlas.blas;
		buildInfo.scratchData.deviceAddress = scratchAddress;

		VkAccelerationStructureBuildRangeInfoKHR rangeInfo{};
		rangeInfo.primitiveCount = primitiveCnt;
		rangeInfo.primitiveOffset = 0;
		rangeInfo.firstVertex = 0;
		rangeInfo.transformOffset = 0;

		VkAccelerationStructureBuildRangeInfoKHR* pRangeInfo = &rangeInfo;

		VkCommandBuffer cmd = beginSingleTimeCommands();
		_vkCmdBuildAccelerationStructuresKHR_PFN(
			cmd,
			1,
			&buildInfo,
			&pRangeInfo);
		endSingleTimeCommands(cmd);

		// Destroy scratch
		vkDestroyBuffer(_vkDevice, scratchBuffer, nullptr);
		vkFreeMemory(_vkDevice, scratchMemory, nullptr);

		//Get device address for this BLAS
		VkAccelerationStructureDeviceAddressInfoKHR addrAs{};
		addrAs.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
		addrAs.accelerationStructure = clusterBlas.blas;

		clusterBlas.deviceAddress =
			_vkGetAccelerationStructureDeviceAddressKHR_PFN(_vkDevice, &addrAs);

		_rtClusterBlases.push_back(clusterBlas);
	}

	//std::cout << "buildClusterBlases: built " << _rtClusterBlases.size() << " cluster BLASes\n";
}

void VulkanRenderer::buildClusterTlasAll()
{
	if (_rtClusterBlases.empty())
	{
		std::cerr << "buildClusterTlasAll: no cluster BLASes, skipping TLAS build\n";
		return;
	}

	// Make sure previous cluster TLAS is gone
	if (_vkClusterTlas != VK_NULL_HANDLE)
	{
		_vkDestroyAccelerationStructureKHR_PFN(_vkDevice, _vkClusterTlas, nullptr);
		_vkClusterTlas = VK_NULL_HANDLE;
	}
	if (_vkClusterTlasBuffer != VK_NULL_HANDLE)
	{
		vkDestroyBuffer(_vkDevice, _vkClusterTlasBuffer, nullptr);
		_vkClusterTlasBuffer = VK_NULL_HANDLE;
	}
	if (_vkClusterTlasMemory != VK_NULL_HANDLE)
	{
		vkFreeMemory(_vkDevice, _vkClusterTlasMemory, nullptr);
		_vkClusterTlasMemory = VK_NULL_HANDLE;
	}

	//Build instance array on CPU
	std::vector<VkAccelerationStructureInstanceKHR> instances;
	instances.reserve(_rtClusterBlases.size());

	for (uint32_t i = 0; i < _rtClusterBlases.size(); ++i)
	{
		const auto& cb = _rtClusterBlases[i];
		if (cb.blas == VK_NULL_HANDLE || cb.deviceAddress == 0)
			continue;

		VkAccelerationStructureInstanceKHR inst{};
		// identity transform
		inst.transform.matrix[0][0] = 1.0f;
		inst.transform.matrix[0][1] = 0;
		inst.transform.matrix[0][2] = 0;
		inst.transform.matrix[0][3] = 0;

		inst.transform.matrix[1][0] = 0;
		inst.transform.matrix[1][1] = 1.0f;
		inst.transform.matrix[1][2] = 0;
		inst.transform.matrix[1][3] = 0;

		inst.transform.matrix[2][0] = 0;
		inst.transform.matrix[2][1] = 0;
		inst.transform.matrix[2][2] = 1.0f;
		inst.transform.matrix[2][3] = 0;

		inst.instanceCustomIndex = i;  // index into _rtClusterBlases / future per-instance data
		inst.mask = 0xFF;
		inst.instanceShaderBindingTableRecordOffset = 0;
		inst.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
		inst.accelerationStructureReference = cb.deviceAddress;

		instances.push_back(inst);
	}

	const uint32_t instanceCount = static_cast<uint32_t>(instances.size());
	if (instanceCount == 0)
	{
		std::cerr << "buildClusterTlasAll: no valid instances, skipping\n";
		return;
	}

	//Upload instances to a GPU buffer with device address
	VkDeviceSize instancesSize = sizeof(VkAccelerationStructureInstanceKHR) * instanceCount;

	VkBuffer instancesBuffer;
	VkDeviceMemory instancesMemory;
	createBuffer(
		instancesSize,
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		instancesBuffer,
		instancesMemory);

	// Map & copy
	void* data = nullptr;
	vkMapMemory(_vkDevice, instancesMemory, 0, instancesSize, 0, &data);
	std::memcpy(data, instances.data(), static_cast<size_t>(instancesSize));
	vkUnmapMemory(_vkDevice, instancesMemory);

	// Get device address for instancesBuffer
	VkBufferDeviceAddressInfo addrInfo{};
	addrInfo.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
	addrInfo.buffer = instancesBuffer;

	VkDeviceAddress instancesAddress = vkGetBufferDeviceAddress(_vkDevice, &addrInfo);

	//Describe TLAS geometry (instances)
	VkAccelerationStructureGeometryInstancesDataKHR instancesData{};
	instancesData.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
	instancesData.arrayOfPointers = VK_FALSE;
	instancesData.data.deviceAddress = instancesAddress;

	VkAccelerationStructureGeometryKHR asGeom{};
	asGeom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
	asGeom.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
	asGeom.geometry.instances = instancesData;

	VkAccelerationStructureBuildGeometryInfoKHR buildInfo{};
	buildInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
	buildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
	buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
	buildInfo.geometryCount = 1;
	buildInfo.pGeometries = &asGeom;

	VkAccelerationStructureBuildSizesInfoKHR sizeInfo{};
	sizeInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;

	uint32_t primCountArray[1] = { instanceCount };

	_vkGetAccelerationStructureBuildSizesKHR_PFN(
		_vkDevice,
		VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
		&buildInfo,
		primCountArray,
		&sizeInfo);

	//Allocate buffer + memory for TLAS
	VkBufferCreateInfo bufInfo{};
	bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufInfo.size = sizeInfo.accelerationStructureSize;
	bufInfo.usage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
	bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

	if (vkCreateBuffer(_vkDevice, &bufInfo, nullptr, &_vkClusterTlasBuffer) != VK_SUCCESS)
	{
		std::cerr << "buildClusterTlasAll: Failed to create TLAS buffer\n";
		vkDestroyBuffer(_vkDevice, instancesBuffer, nullptr);
		vkFreeMemory(_vkDevice, instancesMemory, nullptr);
		return;
	}

	VkMemoryRequirements memReq{};
	vkGetBufferMemoryRequirements(_vkDevice, _vkClusterTlasBuffer, &memReq);

	VkMemoryAllocateFlagsInfo allocFlags{};
	allocFlags.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
	allocFlags.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT_KHR;

	VkMemoryAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.pNext = &allocFlags;
	allocInfo.allocationSize = memReq.size;
	allocInfo.memoryTypeIndex = getMemoryType(
		memReq.memoryTypeBits,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	if (vkAllocateMemory(_vkDevice, &allocInfo, nullptr, &_vkClusterTlasMemory) != VK_SUCCESS)
	{
		std::cerr << "buildClusterTlasAll: Failed to allocate TLAS memory\n";
		vkDestroyBuffer(_vkDevice, _vkClusterTlasBuffer, nullptr);
		_vkClusterTlasBuffer = VK_NULL_HANDLE;

		vkDestroyBuffer(_vkDevice, instancesBuffer, nullptr);
		vkFreeMemory(_vkDevice, instancesMemory, nullptr);
		return;
	}

	vkBindBufferMemory(_vkDevice, _vkClusterTlasBuffer, _vkClusterTlasMemory, 0);

	//Create TLAS handle
	VkAccelerationStructureCreateInfoKHR asCreateInfo{};
	asCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
	asCreateInfo.buffer = _vkClusterTlasBuffer;
	asCreateInfo.size = sizeInfo.accelerationStructureSize;
	asCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;

	if (_vkCreateAccelerationStructureKHR_PFN(
		_vkDevice,
		&asCreateInfo,
		nullptr,
		&_vkClusterTlas) != VK_SUCCESS)
	{
		std::cerr << "buildClusterTlasAll: Failed to create TLAS handle\n";
		vkDestroyBuffer(_vkDevice, _vkClusterTlasBuffer, nullptr);
		vkFreeMemory(_vkDevice, _vkClusterTlasMemory, nullptr);
		_vkClusterTlasBuffer = VK_NULL_HANDLE;
		_vkClusterTlasMemory = VK_NULL_HANDLE;

		vkDestroyBuffer(_vkDevice, instancesBuffer, nullptr);
		vkFreeMemory(_vkDevice, instancesMemory, nullptr);
		return;
	}

	//Scratch buffer + build
	VkBuffer scratchBuffer;
	VkDeviceMemory scratchMemory;
	createBuffer(
		sizeInfo.buildScratchSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
		scratchBuffer,
		scratchMemory);

	addrInfo.buffer = scratchBuffer;
	VkDeviceAddress scratchAddress = vkGetBufferDeviceAddress(_vkDevice, &addrInfo);

	buildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
	buildInfo.dstAccelerationStructure = _vkClusterTlas;
	buildInfo.scratchData.deviceAddress = scratchAddress;

	VkAccelerationStructureBuildRangeInfoKHR rangeInfo{};
	rangeInfo.primitiveCount = instanceCount;
	rangeInfo.primitiveOffset = 0;
	rangeInfo.firstVertex = 0;
	rangeInfo.transformOffset = 0;

	VkAccelerationStructureBuildRangeInfoKHR* pRangeInfo = &rangeInfo;

	VkCommandBuffer cmd = beginSingleTimeCommands();
	_vkCmdBuildAccelerationStructuresKHR_PFN(
		cmd,
		1,
		&buildInfo,
		&pRangeInfo);
	endSingleTimeCommands(cmd);

	// Clean up scratch + instances buffer
	vkDestroyBuffer(_vkDevice, scratchBuffer, nullptr);
	vkFreeMemory(_vkDevice, scratchMemory, nullptr);

	vkDestroyBuffer(_vkDevice, instancesBuffer, nullptr);
	vkFreeMemory(_vkDevice, instancesMemory, nullptr);

	//Create per-instance RtInstanceData buffer for all clusters
	if (_vkRtInstanceBuffer != VK_NULL_HANDLE)
	{
		vkDestroyBuffer(_vkDevice, _vkRtInstanceBuffer, nullptr);
		_vkRtInstanceBuffer = VK_NULL_HANDLE;
	}
	if (_vkRtInstanceBufferMemory != VK_NULL_HANDLE)
	{
		vkFreeMemory(_vkDevice, _vkRtInstanceBufferMemory, nullptr);
		_vkRtInstanceBufferMemory = VK_NULL_HANDLE;
	}

	const size_t rtInstanceCount = _rtClusterBlases.size();
	if (rtInstanceCount == 0)
	{
		std::cerr << "buildClusterTlasAll: no cluster BLASes for instance data\n";
		return;
	}

	VkDeviceSize rtInstSize =
		static_cast<VkDeviceSize>(rtInstanceCount * sizeof(engine::vk::RtInstanceData));

	createBuffer(
		rtInstSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		_vkRtInstanceBuffer,
		_vkRtInstanceBufferMemory);

	// Fill per-instance data
	engine::vk::RtInstanceData* instCpuPtr = nullptr;
	vkMapMemory(
		_vkDevice,
		_vkRtInstanceBufferMemory,
		0,
		rtInstSize,
		0,
		reinterpret_cast<void**>(&instCpuPtr));

	for (size_t i = 0; i < rtInstanceCount; ++i)
	{
		const auto& cb = _rtClusterBlases[i];
		instCpuPtr[i].baseIndex = cb.firstIndex;   // slice into global index buffer
		instCpuPtr[i].clusterId = cb.clusterId;    // for debugging / future use
		instCpuPtr[i].lodLevel = 0;               // optional: make LOD-aware later
		instCpuPtr[i].renderMode = 0;
	}

	vkUnmapMemory(_vkDevice, _vkRtInstanceBufferMemory);

	// Update descriptors 4/5/6/7 to use _vkClusterTlas and per-instance buffer
	VkWriteDescriptorSetAccelerationStructureKHR accelInfo{};
	accelInfo.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR;
	accelInfo.accelerationStructureCount = 1;
	accelInfo.pAccelerationStructures = &_vkClusterTlas;

	VkDescriptorBufferInfo vertexSsboInfo{};
	vertexSsboInfo.buffer = _vkRtVertexBuffer;
	vertexSsboInfo.offset = 0;
	vertexSsboInfo.range = VK_WHOLE_SIZE;

	VkDescriptorBufferInfo indexSsboInfo{};
	indexSsboInfo.buffer = _vkRtIndexBuffer;
	indexSsboInfo.offset = 0;
	indexSsboInfo.range = VK_WHOLE_SIZE;

	VkDescriptorBufferInfo instanceSsboInfo{};
	instanceSsboInfo.buffer = _vkRtInstanceBuffer;
	instanceSsboInfo.offset = 0;
	instanceSsboInfo.range = VK_WHOLE_SIZE;

	for (size_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; ++i)
	{
		std::array<VkWriteDescriptorSet, 4> writes{};

		// binding 4: TLAS
		writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writes[0].pNext = &accelInfo;
		writes[0].dstSet = _vkDescriptorSets[i];
		writes[0].dstBinding = 4;
		writes[0].dstArrayElement = 0;
		writes[0].descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
		writes[0].descriptorCount = 1;

		// binding 5: vertex SSBO
		writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writes[1].dstSet = _vkDescriptorSets[i];
		writes[1].dstBinding = 5;
		writes[1].dstArrayElement = 0;
		writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		writes[1].descriptorCount = 1;
		writes[1].pBufferInfo = &vertexSsboInfo;

		// binding 6: index SSBO
		writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writes[2].dstSet = _vkDescriptorSets[i];
		writes[2].dstBinding = 6;
		writes[2].dstArrayElement = 0;
		writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		writes[2].descriptorCount = 1;
		writes[2].pBufferInfo = &indexSsboInfo;

		// binding 7: per-instance data SSBO
		writes[3].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writes[3].dstSet = _vkDescriptorSets[i];
		writes[3].dstBinding = 7;
		writes[3].dstArrayElement = 0;
		writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		writes[3].descriptorCount = 1;
		writes[3].pBufferInfo = &instanceSsboInfo;

		vkUpdateDescriptorSets(
			_vkDevice,
			static_cast<uint32_t>(writes.size()),
			writes.data(),
			0,
			nullptr);
	}

	//std::cout << "buildClusterTlasAll: built TLAS with " << instanceCount << " instances\n";

}

void VulkanRenderer::buildClusterTlasVisible(const std::vector<engineID_t>& visibleClusters, uint32_t renderMode)
{
	// For correctness while experimenting: make sure GPU isnt using the old TLAS
	vkDeviceWaitIdle(_vkDevice);

	// Destroy previous cluster TLAS
	if (_vkClusterTlas != VK_NULL_HANDLE)
	{
		_vkDestroyAccelerationStructureKHR_PFN(_vkDevice, _vkClusterTlas, nullptr);
		_vkClusterTlas = VK_NULL_HANDLE;
	}
	if (_vkClusterTlasBuffer != VK_NULL_HANDLE)
	{
		vkDestroyBuffer(_vkDevice, _vkClusterTlasBuffer, nullptr);
		_vkClusterTlasBuffer = VK_NULL_HANDLE;
	}
	if (_vkClusterTlasMemory != VK_NULL_HANDLE)
	{
		vkFreeMemory(_vkDevice, _vkClusterTlasMemory, nullptr);
		_vkClusterTlasMemory = VK_NULL_HANDLE;
	}

	// Build TLAS instances only for the visible clusters
	std::vector<VkAccelerationStructureInstanceKHR> instances;
	instances.reserve(visibleClusters.size());

	// Parallel array of which BLAS each instance refers to
	std::vector<const engine::vk::RtClusterBlas*> usedBlases;
	usedBlases.reserve(visibleClusters.size());

	for (engineID_t cid : visibleClusters)
	{
		// Find matching BLAS for this clusterId (simple linear search for now)
		const engine::vk::RtClusterBlas* found = nullptr;
		for (const auto& cb : _rtClusterBlases)
		{
			if (cb.clusterId == cid)
			{
				found = &cb;
				break;
			}
		}

		if (!found || found->blas == VK_NULL_HANDLE || found->deviceAddress == 0)
			continue;

		usedBlases.push_back(found);

		VkAccelerationStructureInstanceKHR inst{};
		// identity transform
		inst.transform.matrix[0][0] = 1.0f;
		inst.transform.matrix[0][1] = 0.0f;
		inst.transform.matrix[0][2] = 0.0f;
		inst.transform.matrix[0][3] = 0.0f;

		inst.transform.matrix[1][0] = 0.0f;
		inst.transform.matrix[1][1] = 1.0f;
		inst.transform.matrix[1][2] = 0.0f;
		inst.transform.matrix[1][3] = 0.0f;

		inst.transform.matrix[2][0] = 0.0f;
		inst.transform.matrix[2][1] = 0.0f;
		inst.transform.matrix[2][2] = 1.0f;
		inst.transform.matrix[2][3] = 0.0f;

		uint32_t instIndex = static_cast<uint32_t>(usedBlases.size() - 1);
		inst.instanceCustomIndex = instIndex;  // matches rtInstances[instIndex]
		inst.mask = 0xFF;
		inst.instanceShaderBindingTableRecordOffset = 0;
		inst.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
		inst.accelerationStructureReference = found->deviceAddress;

		instances.push_back(inst);
	}

	const uint32_t instanceCount = static_cast<uint32_t>(instances.size());
	if (instanceCount == 0)
	{
		std::cerr << "buildClusterTlasVisible: no valid instances after filtering\n";
		return;
	}

	//Upload instances to a GPU buffer
	VkDeviceSize instancesSize =
		sizeof(VkAccelerationStructureInstanceKHR) * instanceCount;

	VkBuffer instancesBuffer;
	VkDeviceMemory instancesMemory;
	createBuffer(
		instancesSize,
		VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		instancesBuffer,
		instancesMemory);

	void* data = nullptr;
	vkMapMemory(_vkDevice, instancesMemory, 0, instancesSize, 0, &data);
	std::memcpy(data, instances.data(), static_cast<size_t>(instancesSize));
	vkUnmapMemory(_vkDevice, instancesMemory);

	// Device address for instances
	VkBufferDeviceAddressInfo addrInfo{};
	addrInfo.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
	addrInfo.buffer = instancesBuffer;
	VkDeviceAddress instancesAddress = vkGetBufferDeviceAddress(_vkDevice, &addrInfo);

	//TLAS geometry description
	VkAccelerationStructureGeometryInstancesDataKHR instancesData{};
	instancesData.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
	instancesData.arrayOfPointers = VK_FALSE;
	instancesData.data.deviceAddress = instancesAddress;

	VkAccelerationStructureGeometryKHR asGeom{};
	asGeom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
	asGeom.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
	asGeom.geometry.instances = instancesData;

	VkAccelerationStructureBuildGeometryInfoKHR buildInfo{};
	buildInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
	buildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
	buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
	buildInfo.geometryCount = 1;
	buildInfo.pGeometries = &asGeom;

	VkAccelerationStructureBuildSizesInfoKHR sizeInfo{};
	sizeInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;

	uint32_t primCountArray[1] = { instanceCount };

	_vkGetAccelerationStructureBuildSizesKHR_PFN(
		_vkDevice,
		VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
		&buildInfo,
		primCountArray,
		&sizeInfo);

	//Allocate TLAS buffer + memory
	VkBufferCreateInfo bufInfo{};
	bufInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufInfo.size = sizeInfo.accelerationStructureSize;
	bufInfo.usage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
	bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

	if (vkCreateBuffer(_vkDevice, &bufInfo, nullptr, &_vkClusterTlasBuffer) != VK_SUCCESS)
	{
		std::cerr << "buildClusterTlasVisible: Failed to create TLAS buffer\n";
		vkDestroyBuffer(_vkDevice, instancesBuffer, nullptr);
		vkFreeMemory(_vkDevice, instancesMemory, nullptr);
		return;
	}

	VkMemoryRequirements memReq{};
	vkGetBufferMemoryRequirements(_vkDevice, _vkClusterTlasBuffer, &memReq);

	VkMemoryAllocateFlagsInfo allocFlags{};
	allocFlags.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
	allocFlags.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT_KHR;

	VkMemoryAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.pNext = &allocFlags;
	allocInfo.allocationSize = memReq.size;
	allocInfo.memoryTypeIndex = getMemoryType(
		memReq.memoryTypeBits,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	if (vkAllocateMemory(_vkDevice, &allocInfo, nullptr, &_vkClusterTlasMemory) != VK_SUCCESS)
	{
		std::cerr << "buildClusterTlasVisible: Failed to allocate TLAS memory\n";
		vkDestroyBuffer(_vkDevice, _vkClusterTlasBuffer, nullptr);
		_vkClusterTlasBuffer = VK_NULL_HANDLE;

		vkDestroyBuffer(_vkDevice, instancesBuffer, nullptr);
		vkFreeMemory(_vkDevice, instancesMemory, nullptr);
		return;
	}

	vkBindBufferMemory(_vkDevice, _vkClusterTlasBuffer, _vkClusterTlasMemory, 0);

	//Create TLAS handle
	VkAccelerationStructureCreateInfoKHR asCreateInfo{};
	asCreateInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
	asCreateInfo.buffer = _vkClusterTlasBuffer;
	asCreateInfo.size = sizeInfo.accelerationStructureSize;
	asCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;

	if (_vkCreateAccelerationStructureKHR_PFN(
		_vkDevice,
		&asCreateInfo,
		nullptr,
		&_vkClusterTlas) != VK_SUCCESS)
	{
		std::cerr << "buildClusterTlasVisible: Failed to create TLAS handle\n";
		vkDestroyBuffer(_vkDevice, _vkClusterTlasBuffer, nullptr);
		vkFreeMemory(_vkDevice, _vkClusterTlasMemory, nullptr);
		_vkClusterTlasBuffer = VK_NULL_HANDLE;
		_vkClusterTlasMemory = VK_NULL_HANDLE;

		vkDestroyBuffer(_vkDevice, instancesBuffer, nullptr);
		vkFreeMemory(_vkDevice, instancesMemory, nullptr);
		return;
	}

	//Scratch buffer + TLAS build
	VkBuffer scratchBuffer;
	VkDeviceMemory scratchMemory;
	createBuffer(
		sizeInfo.buildScratchSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
		VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
		VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
		scratchBuffer,
		scratchMemory);

	addrInfo.buffer = scratchBuffer;
	VkDeviceAddress scratchAddress = vkGetBufferDeviceAddress(_vkDevice, &addrInfo);

	buildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
	buildInfo.dstAccelerationStructure = _vkClusterTlas;
	buildInfo.scratchData.deviceAddress = scratchAddress;

	VkAccelerationStructureBuildRangeInfoKHR rangeInfo{};
	rangeInfo.primitiveCount = instanceCount;
	rangeInfo.primitiveOffset = 0;
	rangeInfo.firstVertex = 0;
	rangeInfo.transformOffset = 0;

	VkAccelerationStructureBuildRangeInfoKHR* pRangeInfo = &rangeInfo;

	VkCommandBuffer cmd = beginSingleTimeCommands();
	_vkCmdBuildAccelerationStructuresKHR_PFN(
		cmd,
		1,
		&buildInfo,
		&pRangeInfo);
	endSingleTimeCommands(cmd);

	// Destroy scratch + instance buffer
	vkDestroyBuffer(_vkDevice, scratchBuffer, nullptr);
	vkFreeMemory(_vkDevice, scratchMemory, nullptr);

	vkDestroyBuffer(_vkDevice, instancesBuffer, nullptr);
	vkFreeMemory(_vkDevice, instancesMemory, nullptr);

	//Build per-instance RtInstanceData in the same order as usedBlases
	if (_vkRtInstanceBuffer != VK_NULL_HANDLE)
	{
		vkDestroyBuffer(_vkDevice, _vkRtInstanceBuffer, nullptr);
		_vkRtInstanceBuffer = VK_NULL_HANDLE;
	}
	if (_vkRtInstanceBufferMemory != VK_NULL_HANDLE)
	{
		vkFreeMemory(_vkDevice, _vkRtInstanceBufferMemory, nullptr);
		_vkRtInstanceBufferMemory = VK_NULL_HANDLE;
	}

	const size_t rtInstanceCount = usedBlases.size();
	VkDeviceSize rtInstSize =
		static_cast<VkDeviceSize>(rtInstanceCount * sizeof(engine::vk::RtInstanceData));

	createBuffer(
		rtInstSize,
		VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
		_vkRtInstanceBuffer,
		_vkRtInstanceBufferMemory);

	engine::vk::RtInstanceData* instCpuPtr = nullptr;

	vkMapMemory(
		_vkDevice,
		_vkRtInstanceBufferMemory,
		0,
		rtInstSize,
		0,
		reinterpret_cast<void**>(&instCpuPtr));

	for (size_t i = 0; i < rtInstanceCount; ++i)
	{
		const auto* cb = usedBlases[i];
		instCpuPtr[i].baseIndex = cb->firstIndex;
		instCpuPtr[i].clusterId = cb->clusterId;
		instCpuPtr[i].lodLevel = 0; // optional 
		instCpuPtr[i].renderMode = renderMode;
	}

	vkUnmapMemory(_vkDevice, _vkRtInstanceBufferMemory);

	VkWriteDescriptorSetAccelerationStructureKHR accelInfo{};
	accelInfo.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR;
	accelInfo.accelerationStructureCount = 1;
	accelInfo.pAccelerationStructures = &_vkClusterTlas;

	VkDescriptorBufferInfo vertexSsboInfo{};
	vertexSsboInfo.buffer = _vkRtVertexBuffer;
	vertexSsboInfo.offset = 0;
	vertexSsboInfo.range = VK_WHOLE_SIZE;

	VkDescriptorBufferInfo indexSsboInfo{};
	indexSsboInfo.buffer = _vkRtIndexBuffer;
	indexSsboInfo.offset = 0;
	indexSsboInfo.range = VK_WHOLE_SIZE;

	VkDescriptorBufferInfo instanceSsboInfo{};
	instanceSsboInfo.buffer = _vkRtInstanceBuffer;
	instanceSsboInfo.offset = 0;
	instanceSsboInfo.range = VK_WHOLE_SIZE;


	for (size_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; ++i)
	{
		std::array<VkWriteDescriptorSet, 4> writes{};

		writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writes[0].pNext = &accelInfo;
		writes[0].dstSet = _vkDescriptorSets[i];
		writes[0].dstBinding = 4;
		writes[0].descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
		writes[0].descriptorCount = 1;

		writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writes[1].dstSet = _vkDescriptorSets[i];
		writes[1].dstBinding = 5;
		writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		writes[1].descriptorCount = 1;
		writes[1].pBufferInfo = &vertexSsboInfo;

		writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writes[2].dstSet = _vkDescriptorSets[i];
		writes[2].dstBinding = 6;
		writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		writes[2].descriptorCount = 1;
		writes[2].pBufferInfo = &indexSsboInfo;

		writes[3].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writes[3].dstSet = _vkDescriptorSets[i];
		writes[3].dstBinding = 7;
		writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		writes[3].descriptorCount = 1;
		writes[3].pBufferInfo = &instanceSsboInfo;

		vkUpdateDescriptorSets(
			_vkDevice,
			static_cast<uint32_t>(writes.size()),
			writes.data(),
			0,
			nullptr);
	}
}
