// SPDX-License-Identifier: MIT
/*
===============================================================================
 Custom-Virtualized-Geometry-Renderer — App.cpp
-------------------------------------------------------------------------------
 Academic/experimental prototype inspired by UE5 Nanite.
 Not production-ready. Provided “AS IS”, without warranties or conditions.
 See README.md for context and LICENSE for terms.
 Source: https://github.com/Wbiu/Custom-Virtualized-Geometry-Renderer
===============================================================================
*/

#pragma once
#include <chrono>
#include <thread>
#include <engine/renderer/vulkan/VulkanRenderer.h>
#include <engine/camera/Camera.h>
#include <engine/UI/GUI.h>
#include <engine/mesh_processing/MeshProcessor.h>
#include <engine/threading/Threading.h>
#include <windows.h>
class App
{

private:

	// begin variables definition
	// windowing variables
	int _windowWidth = 0;
	int _windowHeight = 0;
	const char* _applicationName = "Virtualized-Geometry-Renderer\0";


	engine::math::Mat4f _mvpMat;

	VulkanRenderer* _vKRenderer = nullptr;
	GLFWwindow* _window;

	std::atomic<bool> _isModelPresent{ false };

	GUI* _gui = nullptr;

	Camera* _camera = nullptr;

	std::vector<engine::mesh::ClusterDrawRange> _drawList;

	engine::std_::EngineResourcePool _resourcePool;

	MeshProcessor* _meshProcessor = nullptr;
	ClusterManager* _clusterManager = nullptr;
	Threading* _thread = nullptr;
	engine::mesh::LinearizedModelData* _linearizedData;
	// end vars


	// beginn function decraration
	void initApp();
	void run();
	void initEngine();
	void initRenderer();
	void initCamera();
	std::function<void(const char*)>  meshPrecessingHandler();
	void initGUI();
	std::chrono::steady_clock::duration target_period_from_fps(double fps);
	void cleanup();
	// end function decration
public:
	
	App(int windowWidth, int windowHight);
	~App();
	void startup();

};