#include "GUI.h"

GUI::GUI()
{
}

GUI::~GUI()
{
}

std::filesystem::path GUI::findShaderPath(const char* path)
{
    // prefer project root when launched from VS
    if (std::filesystem::exists(path))
        return path;

    // otherwise use current path in standalone execution
    return std::filesystem::current_path() / path;
}


void GUI::init()
{
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    auto path = findShaderPath("res\\assets\\model");

    for (const auto& entry : std::filesystem::directory_iterator(path))
    {
        _comboxItem_Model.push_back(std::filesystem::path(entry).string());
    }
    ImGui::SetNextWindowSize({ 300,250 }, ImGuiCond_Once);

}

void GUI::initBackend(GLFWwindow* glfwWindow)
{
    _glfwWindow = glfwWindow;
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForVulkan(glfwWindow, true);
    ImGui_ImplVulkan_Init(&init_info);
    ImGui_ImplVulkan_CreateFontsTexture();

}

void GUI::cleanUp()
{
    ImGui_ImplVulkan_DestroyFontsTexture();
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void GUI::beginFrame()
{
    // Start the Dear ImGui frame
    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

}

void GUI::frameContent()
{

    ImGui::Begin("Data", &_windowOpen,ImGuiWindowFlags_NoResize);
    ImGui::Text(_text);

    if (_loadButtonVisibility)
        _buttonLoad = ImGui::Button("Load");
    else
        _buttonLoad = false;

    ImGui::Text(_toDisplayLoadText);

	static std::string currentItem_Model = "";

	if (ImGui::BeginCombo("##combo", currentItem_Model.c_str()))
	{
		for (int n = 0; n < _comboxItem_Model.size(); n++)
		{
            bool is_selected = std::strcmp(currentItem_Model.c_str(), _comboxItem_Model[n].c_str());

			if (ImGui::Selectable(_comboxItem_Model[n].c_str(), is_selected))
            {
                currentItem_Model = _comboxItem_Model[n].c_str();
            }

			if (is_selected)
            {
                _selectedItem_Model = currentItem_Model;
               ImGui::SetItemDefaultFocus();
            }
		}

		ImGui::EndCombo();
	}

    static std::string currentItem_RenderMode = _comboxItem_RenderMode[0];
    ImGui::Text("Select Render View Mode:\n");
    if (ImGui::BeginCombo("###combo", currentItem_RenderMode.c_str()))
    {
        for (int n = 0; n < _comboxItem_RenderMode.size(); n++)
        {
            bool is_selected = std::strcmp(currentItem_RenderMode.c_str(), _comboxItem_RenderMode[n].c_str());

            if (ImGui::Selectable(_comboxItem_RenderMode[n].c_str(), is_selected))
            {
                currentItem_RenderMode = _comboxItem_RenderMode[n].c_str();
                _selectedRenderMode_Idx = n;
            }

            if (is_selected)
            {
                _selectedItem_RenderMode = currentItem_RenderMode;
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::Text("\nControlls : W | A | S | D \n CTRL | SPACE_BAR | ESC \n");
    ImGui::End();
}

void GUI::endFrame()
{
    // Rendering
    ImGui::Render();
}

void GUI::drawUI(VkCommandBuffer cmd)
{
    ImDrawData*  drawData = ImGui::GetDrawData();
    ImGui_ImplVulkan_RenderDrawData(drawData,cmd);
}

std::function<void(VkCommandBuffer)> GUI::getRecorder()
{
    return [](VkCommandBuffer cmd)
        {
            ImDrawData* drawData = ImGui::GetDrawData();
            if (!drawData || drawData->CmdListsCount == 0) return;
            ImGui_ImplVulkan_RenderDrawData(drawData, cmd);
        };
}



std::function<void(uint32_t)> GUI::getSwapchainRecreatedHandler()
{
    return [](uint32_t newImageCount)
        {
            ImGui_ImplVulkan_SetMinImageCount(newImageCount);
        };
}



void GUI::setText(const char* text)
{
    _text = text;
}

bool GUI::buttonLoad() const
{
    return _buttonLoad;
}

void GUI::setLoadStatus(bool status)
{
    _processStatus = status;
    _toDisplayLoadText = status ? _loadDoneText : _loadInProcessText;
}

void GUI::setLoadButtonVisibility(bool shouldShow)
{
    _loadButtonVisibility = shouldShow;
}

void GUI::buildGuiRenderData()
{
    beginFrame();
    frameContent();
    endFrame();
}

