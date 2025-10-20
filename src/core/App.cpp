#include "App.h"

App::App(int windowWidth, int windowHeight)
	: _windowWidth(windowWidth), _windowHeight(windowHeight)
{

}

App::~App()
{
	cleanup();
}

void App::initApp()
{
}

void App::run()
{
	engine::math::Mat4f projMat = engine::math::perspective(45.0f, (float)_windowWidth / (float)_windowHeight, 0.1f, 100.0f);
	projMat[1][1] *= -1.0f;        // Vulkan NDC Y flip

	engine::math::Mat4f viewMat = _camera->getViewMatrix();
	engine::math::Mat4f modelMat = engine::math::Mat4f(1.0f);
	auto vpMat = viewMat * projMat;

	_camera->setWorldSpaceFrustom(vpMat);

	_mvpMat = modelMat * vpMat;

	// time stamps
	std::chrono::steady_clock::duration kTargetFrame = target_period_from_fps(120.0);  // set FPS cap
	float cpuFrameMs = 0.f;               // time spent doing work (no sleep)
	float frameMs = 0.f;               // total frame time incl. sleep
	float fps = 0.f;

	auto tail = std::chrono::microseconds{ 700 }; // will auto-adjust
	constexpr auto tail_min = std::chrono::microseconds{ 100 };
	constexpr auto tail_max = std::chrono::microseconds{ 2000 };

	/*  ui text elements */
	char buffer[256];
	int intersectClusters = 0;
	int primitiveDrawCnt = 0;

	while (!_vKRenderer->shouldCloseWindow())
	{
		auto frameStart = std::chrono::steady_clock::now();   // start-of-frame timestamp
		_vKRenderer->pollWindowInputEvents();
		_camera->update(frameMs,_window);
		 
		viewMat = _camera->getViewMatrix();
		vpMat = viewMat * projMat;
		_mvpMat = modelMat * vpMat;

		_camera->setWorldSpaceFrustom(vpMat);

		auto planes = _camera->getFrumstPlanes();
		auto camPos = _camera->getPos();

		if (_gui->buttonLoad())
		{
			_isModelPresent.store(false, std::memory_order_release);
			_gui->setLoadButtonVisibility(false);
			_gui->setLoadStatus(false);
			_drawList.clear();
			_vKRenderer->submitDrawList(_drawList, _gui->getSelectedRenderMode());

			_thread = new Threading();
			_thread->load(meshPrecessingHandler(),_gui->getSelectedItem().c_str());
			_thread->run();
		}

		if (_isModelPresent.load(std::memory_order_acquire))
		{
			_gui->setLoadStatus(true);
			_drawList.clear();
			_gui->setLoadButtonVisibility(true);
			
			/* frustum culling stage */
			_clusterManager->setfrustumPlanesAndCamPos(planes, camPos);
			_clusterManager->cullClusters_V2(modelMat); // this also call the pickLODLevel();

			// build per-frame draw list from chosen clusters
			size_t c0 = 0, c1 = 0, c2 = 0;
			for (engineID_t cid : _clusterManager->getClustersToDraw())
			{
				primitiveDrawCnt += _clusterManager->getCluster(cid)->primitives.size();
				const auto& drawableCluster = _linearizedData->clusterDrawRange.at(cid);
				_drawList.push_back(drawableCluster); // { firstIndex, indexCount, level }
			}

			intersectClusters = _clusterManager->getSelectedClusterCnt();

			// hand the list to the renderer
			_vKRenderer->submitDrawList(_drawList, _gui->getSelectedRenderMode());
			_vKRenderer->submitUniform(&_mvpMat);
		}

		sprintf_s(buffer,
			"Intersected clusters : %i\nDraw Primitive %i\nFramerate : %.1f\nCPU Frametime : %.2f ms\n",
			intersectClusters, primitiveDrawCnt, fps, cpuFrameMs);

		_gui->setText(buffer);
		_gui->buildGuiRenderData();

		_vKRenderer->draw(_isModelPresent.load(std::memory_order_acquire));

		auto workEnd = std::chrono::steady_clock::now();
		cpuFrameMs = std::chrono::duration<float, std::milli>(workEnd - frameStart).count();


		//cap frame rate
		auto target = frameStart + kTargetFrame;
		auto now = std::chrono::steady_clock::now();

		if (now < target) {

			const auto sleepUntil = max(now, target - tail);
			if (sleepUntil > now) {
				std::this_thread::sleep_until(sleepUntil);
				now = std::chrono::steady_clock::now();
			}

			// busy spin for the remainder (few hundreds of µs)
			while (now < target) {
#if defined(_MSC_VER) && (defined(_M_IX86) || defined(_M_X64))
				_mm_pause();
#elif defined(__i386__) || defined(__x86_64__)
				__builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
				__asm__ __volatile__("yield");
#else
				std::this_thread::yield(); // last-resort fallback
#endif
				now = std::chrono::steady_clock::now();
			}
		}

		// finalize frame timing (includes sleep)
		auto frameEnd = std::chrono::steady_clock::now();
		frameMs = std::chrono::duration<float, std::milli>(frameEnd - frameStart).count();
		fps = 1000.0f / frameMs;  // display FPS based on actual frame length

		// --- adapt tail based on *observed* overshoot/undershoot ---
		const auto delta = (frameEnd - frameStart) - kTargetFrame;  // positive => we overshot target

		// Aim for small negative/near-zero delta
		// Simple PI-like nudge (tune 0.1–0.3 to taste)
		const double k = 0.25;
		tail = std::chrono::microseconds{
			std::clamp<long long>(
				static_cast<long long>(tail.count() + k * std::chrono::duration<double, std::micro>(delta).count()),
				tail_min.count(), tail_max.count())
		};

		primitiveDrawCnt = 0;
	}

	_vKRenderer->waitIdle();
	_gui->cleanUp();
	_vKRenderer->cleanUp();
}

void App::startup()
{
	initApp();
	initEngine();
	run();

}

void App::initEngine()
{
	initRenderer();
	initGUI();
	initCamera();
}

void App::initRenderer()
{

	_vKRenderer = new VulkanRenderer(_windowWidth, _windowHeight, _applicationName);
	_vKRenderer->init();
	_window =  _vKRenderer->getWindow();
}

void App::initCamera()
{
	_camera = new Camera({ 0,0,3.0f });
}

void App::initGUI()
{
	_gui = new GUI();
	_gui->init();

	_vKRenderer->initImGUI_Info(&_gui->init_info);

	_gui->initBackend(_vKRenderer->getWindow());

	_vKRenderer->callBackUI = _gui->getRecorder();
	_vKRenderer->callBackOnSwapchainRecreateUI = _gui->getSwapchainRecreatedHandler();
}

std::chrono::steady_clock::duration App::target_period_from_fps(double fps)
{

	fps = std::clamp(fps, 1.0, 1000.0); // sanity
#if __cpp_lib_chrono >= 201907L
	// Round UP so you never run faster than the cap
	return ceil<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / fps));
#else
	// Portable ceil to nanoseconds
	const auto ns = static_cast<long long>(std::ceil(1'000'000'000.0 / fps));
	return std::chrono::nanoseconds{ ns };
#endif
}

void App::cleanup()
{
}

std::function<void(const char*)> App::meshPrecessingHandler()
{
	return[&](const char* modelPath)
		{
			_meshProcessor = new MeshProcessor();
			_meshProcessor->process(std::move(modelPath), &_resourcePool.pool);

			_clusterManager = _meshProcessor->getClusterManager();
			_linearizedData = _meshProcessor->getLinearizedMeshData();

			_vKRenderer->submitRenderData(_linearizedData->vertices, _linearizedData->indices);

			_isModelPresent.store(true,std::memory_order_release);
			_drawList.reserve(_linearizedData->clusterDrawRange.size());
		};
}