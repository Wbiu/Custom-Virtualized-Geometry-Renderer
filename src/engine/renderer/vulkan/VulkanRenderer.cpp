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

	vkDestroyDescriptorPool(_vkDevice, _vkDescriptorPool, nullptr);
	vkDestroyDescriptorPool(_vkDevice, _vkDescriptorPoolUI, nullptr);
	vkDestroyDescriptorSetLayout(_vkDevice, _vkDescriptorSetLayout, nullptr);

	vkDestroyBuffer(_vkDevice, _vkIndexBuffer, nullptr);
	vkFreeMemory(_vkDevice, _vkIndexBufferMemory, nullptr);

	vkDestroyBuffer(_vkDevice, _vkVertexBuffer, nullptr);
	vkFreeMemory(_vkDevice, _vkVertexBufferMemory, nullptr);

	for (size_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; i++) {
		vkDestroySemaphore(_vkDevice, _vkImageAvailableSemaphores[i], nullptr);
		vkDestroyFence(_vkDevice, _vkInFlightFences[i], nullptr);
	}

	for (size_t i = 0; i < _vkSwapChainImages.size(); i++) {
		vkDestroySemaphore(_vkDevice, _vkRenderFinishedSemaphores[i], nullptr);
	}

	vkDestroyCommandPool(_vkDevice, _vkCommandPool, nullptr);

	vkDestroyPipeline(_vkDevice, _vkGraphicsPipeline, nullptr);
	vkDestroyPipelineLayout(_vkDevice, _vkPipelineLayout, nullptr);
	vkDestroyRenderPass(_vkDevice, _vkRenderPass, nullptr);

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
	vkResetFences(_vkDevice, 1, &_vkInFlightFences[_currentFrame]);

	uint32_t imageIndex;
	VkResult vkAcquireNextImageResult = vkAcquireNextImageKHR(_vkDevice, _vkSwapChain, UINT64_MAX,
		_vkImageAvailableSemaphores[_currentFrame], VK_NULL_HANDLE, &imageIndex);

	if (vkAcquireNextImageResult == VK_ERROR_OUT_OF_DATE_KHR) {
		recreateSwapChain();
		return;
	}
	else if (vkAcquireNextImageResult != VK_SUCCESS && vkAcquireNextImageResult != VK_SUBOPTIMAL_KHR) {
		throw std::runtime_error("ERROR::VULKAN::DRAW_CALL::FAILED_TO_ACQUIRE_SWAPCHAIN_IMAGE\n");
	}

	vkResetCommandBuffer(_vkCommandBuffers[_currentFrame], 0);
	recordCommandBuffer(_vkCommandBuffers[_currentFrame], imageIndex, drawScene);

	if(drawScene)
		updateUniformBuffer(_currentFrame);

	VkSemaphore vkWaitSemaphores[] = { _vkImageAvailableSemaphores[_currentFrame] };
	VkSemaphore vkSignalSemaphores[] = { _vkRenderFinishedSemaphores[imageIndex] };
	VkPipelineStageFlags vkWaitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };

	VkSubmitInfo vkSubmitInfo{};
	vkSubmitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	vkSubmitInfo.waitSemaphoreCount = 1;
	vkSubmitInfo.pWaitDstStageMask = vkWaitStages;
	vkSubmitInfo.commandBufferCount = 1;
	vkSubmitInfo.pCommandBuffers = &_vkCommandBuffers[_currentFrame];
	vkSubmitInfo.signalSemaphoreCount = 1;
	vkSubmitInfo.pSignalSemaphores = vkSignalSemaphores;
	vkSubmitInfo.pWaitSemaphores = vkWaitSemaphores;

	if (vkQueueSubmit(_vkGraphicsQueue, 1, &vkSubmitInfo, _vkInFlightFences[_currentFrame]) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::DRAW_CALL::COMMAND_BUFFER::FAILED_TO_SUBMIT_COMMAND_BUFFER\n");
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
	createFramebuffers();
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
		// coming from present engine
		srcStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		toColor.srcAccessMask = 0;
	}

	VkPipelineStageFlags dstStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
	toColor.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

	vkCmdPipelineBarrier(commandBuffer,
		srcStage, dstStage,
		0, 0, nullptr, 0, nullptr, 1, &toColor);

	// >>> Layout tracking: now in COLOR_ATTACHMENT_OPTIMAL
	_swapchainLayouts[imageIndex] = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;


	if(drawScene)
	{
		// =============== Main scene ===============

		VkClearValue clearColor{ };
		clearColor.color = { {0.f, 0.f, 0.f, 1.f} };

		VkClearValue clearDepth{ };
		clearDepth.depthStencil = { 1.f, 0 };

		VkRenderingAttachmentInfo colorAtt{};
		colorAtt.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO_KHR;
		colorAtt.imageView = _vkColorImageView;                     // MSAA target
		colorAtt.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		colorAtt.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		colorAtt.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;      // resolve writes to swapchain
		colorAtt.clearValue = clearColor;
		colorAtt.resolveMode = VK_RESOLVE_MODE_AVERAGE_BIT;
		colorAtt.resolveImageView = _vkSwapChainImageViews[imageIndex];
		colorAtt.resolveImageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

		VkRenderingAttachmentInfo depthAtt{};
		depthAtt.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO_KHR;
		depthAtt.imageView = _vkDepthImageView;
		depthAtt.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
		depthAtt.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		depthAtt.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		depthAtt.clearValue = clearDepth;

		VkRenderingInfo renderingInfo{};
		renderingInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO_KHR;
		renderingInfo.renderArea = { {0,0}, _vkSwapChainExtent };
		renderingInfo.layerCount = 1;
		renderingInfo.colorAttachmentCount = 1;
		renderingInfo.pColorAttachments = &colorAtt;
		renderingInfo.pDepthAttachment = &depthAtt;

		vkCmdBeginRendering(commandBuffer, &renderingInfo);

		// --- bind & draw INSIDE the scene rendering block ---
		VkViewport vkViewport{};
		vkViewport.x = 0.0f;
		vkViewport.y = 0.0f;
		vkViewport.width = static_cast<float>(_vkSwapChainExtent.width);
		vkViewport.height = static_cast<float>(_vkSwapChainExtent.height);
		vkViewport.minDepth = 0.0f;
		vkViewport.maxDepth = 1.0f;

		vkCmdSetViewport(commandBuffer, 0, 1, &vkViewport);

		VkRect2D vkScissor{};
		vkScissor.offset = { 0, 0 };
		vkScissor.extent = _vkSwapChainExtent;
		vkCmdSetScissor(commandBuffer, 0, 1, &vkScissor);

		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, _vkGraphicsPipeline);

		VkBuffer vkVertexBuffers[] = { _vkVertexBuffer };
		VkDeviceSize vkOffsets[] = { 0 };

		/*selecting vertex buffer */
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vkVertexBuffers, vkOffsets);

		/*selecting vertex index buffer */
		vkCmdBindIndexBuffer(commandBuffer, _vkIndexBuffer, 0, VK_INDEX_TYPE_UINT32);

		/*selecting descriptor */
		vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, _vkPipelineLayout
			, 0, 1, &_vkDescriptorSets[_currentFrame], 0, nullptr);

		// replaced with the indirect draw call
		const auto& ind = _indirectPerFrame[_currentFrame];
		if (ind.count > 0) {
			vkCmdDrawIndexedIndirect(commandBuffer,
				ind.buf,           // buffer with commands
				0,                 // offset
				ind.count,         // drawCount
				sizeof(VkDrawIndexedIndirectCommand));
		}
		vkCmdEndRendering(commandBuffer);
		// ============= END Frist Pass Model === 

		//  small barrier between passes (read/modify same image)
		VkImageMemoryBarrier between{ VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER };
		between.image = swapImg;
		between.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
		between.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		between.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		between.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		between.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

		vkCmdPipelineBarrier(commandBuffer,
			VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
			VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
			0, 0, nullptr, 0, nullptr, 1, &between);
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

	vkCmdEndRendering(commandBuffer); // <-- END UI PASS

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

	if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
		throw std::runtime_error("ERROR::VULKAN::RECORD_COMMAND::FAILED_TO_RECORD_COMMAND_BUFFER");
	}
}

void VulkanRenderer::updateUniformBuffer(uint32_t currentImage)
{
	memcpy(_vkUniformBuffersMapped[currentImage], _mat4Uniform, sizeof(*_mat4Uniform));
}

void VulkanRenderer::updateSSBO(uint32_t currentImage)
{
	memcpy(_perDraw[currentImage].mapped, _frameDrawItems.data(),
		_frameDrawItems.size() * sizeof(engine::vk::DrawItem));
}

void VulkanRenderer::submitUniform(engine::math::Mat4f* mat4)
{
	_mat4Uniform = mat4;
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

	createCommandPool();
	createColorResources(); 
	createDepthResources();

	createIndirectBuffers(); 

	createUniformBuffers();

	createSSBO(); // new 

	createDescriptorPool();
	createUIDescriptorPool();

	createDescriptorSets();

	createCommandBuffers();
	createSyncObjects();

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

	VkPhysicalDeviceShaderDrawParametersFeatures drawFeat{
	VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_DRAW_PARAMETERS_FEATURES
	};
	drawFeat.shaderDrawParameters = VK_TRUE;


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

	VkDescriptorSetLayoutBinding vkUBOLayoutBindingSSBO{};
	vkUBOLayoutBindingSSBO.binding = 2;
	vkUBOLayoutBindingSSBO.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	vkUBOLayoutBindingSSBO.descriptorCount = 1;
	vkUBOLayoutBindingSSBO.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
	vkUBOLayoutBindingSSBO.pImmutableSamplers = nullptr; // Optional
	
	std::array<VkDescriptorSetLayoutBinding, 2> bindings = { vkUBOLayoutBinding, vkUBOLayoutBindingSSBO };

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
			(vkMemProperties.memoryTypes[i].propertyFlags & properties)
			== properties) {

			return i;
		}
	}

	throw std::runtime_error("ERROR::VULKAN::MEMORY::GET_MEMORY_TYPE::FIALED_TO_FIND_SUITABLE_MEMORY_TYPE\n");

	return 0;
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
	vkEndCommandBuffer(commandBuffer);

	VkSubmitInfo vkSubmitInfo{};
	vkSubmitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	vkSubmitInfo.commandBufferCount = 1;
	vkSubmitInfo.pCommandBuffers = &commandBuffer;

	vkQueueSubmit(_vkGraphicsQueue, 1, &vkSubmitInfo, VK_NULL_HANDLE);
	vkQueueWaitIdle(_vkGraphicsQueue);

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

	_vkUniformBuffers.resize(_MAX_FRAMES_IN_FLIGHT);
	_vkUniformBuffersMemory.resize(_MAX_FRAMES_IN_FLIGHT);
	_vkUniformBuffersMapped.resize(_MAX_FRAMES_IN_FLIGHT);

	for (size_t i = 0; i < _MAX_FRAMES_IN_FLIGHT; i++) {
		createBuffer(vkBufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
			, _vkUniformBuffers[i], _vkUniformBuffersMemory[i]);
		vkMapMemory(_vkDevice, _vkUniformBuffersMemory[i], 0, vkBufferSize, 0, &_vkUniformBuffersMapped[i]);
	}
}

void VulkanRenderer::createDescriptorPool()
{
	std::array<VkDescriptorPoolSize, 2> vkDescriptorPoolSize{};

	vkDescriptorPoolSize[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	vkDescriptorPoolSize[0].descriptorCount = static_cast<uint32_t>(_MAX_FRAMES_IN_FLIGHT);

	vkDescriptorPoolSize[1].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	vkDescriptorPoolSize[1].descriptorCount = static_cast<uint32_t>(_MAX_FRAMES_IN_FLIGHT);


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

		VkDescriptorBufferInfo vkDescriptorBufferInfoSSBO{};
		vkDescriptorBufferInfoSSBO.buffer = _perDraw[i].ssbo;
		vkDescriptorBufferInfoSSBO.offset = 0;
		vkDescriptorBufferInfoSSBO.range = VK_WHOLE_SIZE;

		std::array<VkWriteDescriptorSet, 2> vVkDescriptorWrites{};

		vVkDescriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		vVkDescriptorWrites[0].dstSet = _vkDescriptorSets[i];
		vVkDescriptorWrites[0].dstBinding = 0;
		vVkDescriptorWrites[0].dstArrayElement = 0;
		vVkDescriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
		vVkDescriptorWrites[0].descriptorCount = 1;
		vVkDescriptorWrites[0].pBufferInfo = &vkDescriptorBufferInfo;

		vVkDescriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		vVkDescriptorWrites[1].dstSet = _vkDescriptorSets[i];
		vVkDescriptorWrites[1].dstBinding = 2; // <<< binding 2, matches shader
		vVkDescriptorWrites[1].dstArrayElement = 0;
		vVkDescriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		vVkDescriptorWrites[1].descriptorCount = 1;
		vVkDescriptorWrites[1].pBufferInfo = &vkDescriptorBufferInfoSSBO;

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