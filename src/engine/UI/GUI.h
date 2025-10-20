#pragma once
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_vulkan.h>
#include <stdexcept>
#include <functional>
#include <string.h>
#include <iostream>
#include <filesystem>

class GUI
{
private:


	// vars
	bool _swapChainRebuild = false;
	const char* _text;
	bool _buttonLoad = false;
	bool _processStatus = false;
	const char* _loadInProcessText = "Processing Mesh...\n";
	const char* _loadDoneText = "Done\n";
	const char* _loadDoneStartUp = "Select Model\n";

	const char* _toDisplayLoadText = _loadDoneStartUp;

	bool _loadButtonVisibility = true;

	bool _windowOpen = true;

	GLFWwindow* _glfwWindow;
	std::vector<std::string> _comboxItem_Model;
	std::vector<std::string> _comboxItem_RenderMode = {"Cluster","Primitive"};

	std::string _selectedItem_Model = "";
	std::string _selectedItem_RenderMode = "";
	uint32_t _selectedRenderMode_Idx = 0;
	std::filesystem::path findShaderPath(const char* path);

	// end vars

	// begin functiion
	void beginFrame();
	void frameContent();
	void endFrame();

	// end function
public:

	GUI();
	~GUI();


	ImGui_ImplVulkan_InitInfo init_info = {};

	void init();
	void initBackend(GLFWwindow* glfwWindow);
	void cleanUp();

	void buildGuiRenderData();

	void drawUI(VkCommandBuffer cmd);
	std::function<void(VkCommandBuffer)> getRecorder();
	//std::function<void(VkCommandBuffer)> setFontupLoadCmd();
	std::function<void(uint32_t)> getSwapchainRecreatedHandler();

	void setText(const char* text);
	std::string getSelectedItem() {return _selectedItem_Model;}
	bool buttonLoad() const;
	void setLoadStatus(bool isDone);
	void setLoadButtonVisibility(bool shouldShow);
	bool getSelectedRenderMode() { return _selectedRenderMode_Idx; };
};