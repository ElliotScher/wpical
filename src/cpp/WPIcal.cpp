// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <fieldcalibration.h>
#include <cameracalibration.h>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <wpigui.h>
#include <portable-file-dialogs.h>

namespace gui = wpi::gui;

const char *GetWPILibVersion();

namespace ov
{
  std::string_view GetResource_ov_16_png();
  std::string_view GetResource_ov_32_png();
  std::string_view GetResource_ov_48_png();
  std::string_view GetResource_ov_64_png();
  std::string_view GetResource_ov_128_png();
  std::string_view GetResource_ov_256_png();
  std::string_view GetResource_ov_512_png();
} // namespace ov

static void DisplayGui()
{
  ImGui::GetStyle().WindowRounding = 0;

  // fill entire OS window with this window
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  int width, height;
  glfwGetWindowSize(gui::GetSystemWindow(), &width, &height);
  ImGui::SetNextWindowSize(
      ImVec2(static_cast<float>(width), static_cast<float>(height)));

  ImGui::Begin("Entries", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_MenuBar |
                   ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                   ImGuiWindowFlags_NoCollapse);

  // main menu
  ImGui::BeginMenuBar();
  gui::EmitViewMenu();
  if (ImGui::BeginMenu("View"))
  {
    ImGui::EndMenu();
  }
  ImGui::EndMenuBar();

  static std::unique_ptr<pfd::open_file> configFileSelector;
  static std::string selectedConfigFile;

  static std::unique_ptr<pfd::open_file> fieldMapSelector;
  static std::string selectedFieldMap;

  static std::unique_ptr<pfd::select_folder> calibrationDirectorySelector;
  static std::string selectedCalibrationDirectory;

  static double squareWidth = 0.709;
  static double markerWidth = 0.551;
  static int boardWidth = 12;
  static int boardHeight = 8;

  static int pinnedTag = 1;
  static int fps = 15;

  if (ImGui::Button("Upload Camera Matrix"))
  {
    configFileSelector = std::make_unique<pfd::open_file>(
        "Select Camera Matrix JSON", "",
        std::vector<std::string>{"JSON", "*.json"}, pfd::opt::none);
  }

  ImGui::SameLine();
  ImGui::Text("Or");
  ImGui::SameLine();

  if (ImGui::Button("Calibrate Camera"))
  {
    selectedConfigFile.clear();
    ImGui::OpenPopup("Camera Calibration");
  }

  if (configFileSelector)
  {
    auto selectedFiles = configFileSelector->result();
    if (!selectedFiles.empty())
    {
      selectedConfigFile = selectedFiles[0];
    }
    configFileSelector.reset();
  }

  if (!selectedConfigFile.empty())
  {
    ImGui::TextWrapped("Selected Camera Matrix: %s", selectedConfigFile.c_str());
  }

  if (ImGui::Button("Select Field Map JSON"))
  {
    fieldMapSelector = std::make_unique<pfd::open_file>(
        "Select Json File", "",
        std::vector<std::string>{"JSON Files", "*.json"}, pfd::opt::none);
  }

  if (fieldMapSelector)
  {
    auto selectedFiles = fieldMapSelector->result();
    if (!selectedFiles.empty())
    {
      selectedFieldMap = selectedFiles[0];
    }
    fieldMapSelector.reset();
  }

  if (!selectedFieldMap.empty())
  {
    ImGui::TextWrapped("Selected Field Map: %s", selectedFieldMap.c_str());
  }

  if (ImGui::Button("Select Field Calibration Folder"))
  {
    calibrationDirectorySelector = std::make_unique<pfd::select_folder>(
        "Select Field Calibration Folder", "");
  }

  if (calibrationDirectorySelector)
  {
    auto selectedFiles = calibrationDirectorySelector->result();
    if (!selectedFiles.empty())
    {
      selectedCalibrationDirectory = selectedFiles;
    }
    calibrationDirectorySelector.reset();
  }

  if (!selectedCalibrationDirectory.empty())
  {
    ImGui::TextWrapped("Selected Folder: %s",
                       selectedCalibrationDirectory.c_str());
  }
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
  ImGui::InputInt("Pinned Tag", &pinnedTag);
  ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
  ImGui::InputInt("Calibration FPS", &fps);

  if (ImGui::Button("Calibrate!!!"))
  {
    if (!selectedCalibrationDirectory.empty() && !selectedConfigFile.empty() && !selectedFieldMap.empty() && pinnedTag > 0 && pinnedTag <= 16)
    {
      int calibrationOutput = fieldcalibration::calibrate(
          selectedCalibrationDirectory.c_str(),
          selectedCalibrationDirectory + "output/output.json",
          selectedConfigFile,
          selectedFieldMap.c_str(),
          pinnedTag,
          fps);

      if (calibrationOutput == -1)
      {
        ImGui::OpenPopup("Error");
      }
      else if (calibrationOutput == 0)
      {
        ImGui::OpenPopup("Success");
      }
    }
  }
  if (selectedCalibrationDirectory.empty() || selectedConfigFile.empty() || selectedFieldMap.empty())
  {
    ImGui::TextWrapped("Some inputs are empty! please enter your camera calibration video, field map, and field calibration directory");
  }
  else if (!(pinnedTag > 0 && pinnedTag <= 16))
  {
    ImGui::TextWrapped("Make sure the pinned tag is a valid april tag (1-16)");
  }
  else if (fps < 0)
  {
    ImGui::TextWrapped("Make sure FPS is at least 1");
  }
  else
  {
    ImGui::TextWrapped("Calibration Ready");
  }

  if (ImGui::BeginPopupModal("Error", NULL, ImGuiWindowFlags_AlwaysAutoResize))
  {
    ImGui::TextWrapped("Please ensure that the detection FPS is smaller than the field video FPS.");
    ImGui::Separator();
    if (ImGui::Button("OK", ImVec2(120, 0)))
    {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  if (ImGui::BeginPopupModal("Success", NULL, ImGuiWindowFlags_AlwaysAutoResize))
  {
    ImGui::TextWrapped("Success, output JSON generated in field calibration video directory");
    ImGui::Separator();
    if (ImGui::Button("OK", ImVec2(120, 0)))
    {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }

  if (ImGui::BeginPopupModal("Camera Calibration", NULL, ImGuiWindowFlags_AlwaysAutoResize))
  {

    if (ImGui::Button("Select Camera Calibration Video"))
    {
      configFileSelector = std::make_unique<pfd::open_file>(
          "Select Camera Calibration Video", "",
          std::vector<std::string>{"Video Files", "*.mp4 *.mov *.m4v *.mkv"}, pfd::opt::none);
    }

    if (configFileSelector)
    {
      auto selectedFiles = configFileSelector->result();
      if (!selectedFiles.empty())
      {
        selectedConfigFile = selectedFiles[0];
      }
      configFileSelector.reset();
    }

    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
    ImGui::InputDouble("Square Width (in)", &squareWidth);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
    ImGui::InputDouble("Marker Width (in)", &markerWidth);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
    ImGui::InputInt("Board Width (squares)", &boardWidth);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 12);
    ImGui::InputInt("Board Height (squares)", &boardHeight);

    ImGui::Separator();
    if (ImGui::Button("Calibrate") && !selectedConfigFile.empty())
    {
      nlohmann::json cameraJson = cameracalibration::calibrate(
          selectedConfigFile.c_str(),
          squareWidth,
          markerWidth,
          boardWidth,
          boardHeight);

      // Write JSON to file
      std::string output_filename = selectedConfigFile;

      size_t pos = output_filename.rfind('/');

      if (pos != std::string::npos)
      {
        output_filename = output_filename.erase(pos);
      }
      else
      {
        pos = output_filename.rfind('\\');
        output_filename = output_filename.erase(pos);
      }
      std::ofstream ofs(output_filename.append("/camera calibration.json"));
      ofs << std::setw(4) << cameraJson << std::endl;
      ofs.close();

      selectedConfigFile = output_filename;
      ImGui::CloseCurrentPopup();
    }

    ImGui::SameLine();
    if (ImGui::Button("Close"))
    {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

#ifdef _WIN32
int __stdcall WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR pCmdLine, int nCmdShow)
{
  int argc = __argc;
  char **argv = __argv;
#else
int main(int argc, char **argv)
{
#endif
  std::string_view saveDir;
  if (argc == 2)
  {
    saveDir = argv[1];
  }

  gui::CreateContext();

  gui::AddIcon(ov::GetResource_ov_16_png());
  gui::AddIcon(ov::GetResource_ov_32_png());
  gui::AddIcon(ov::GetResource_ov_48_png());
  gui::AddIcon(ov::GetResource_ov_64_png());
  gui::AddIcon(ov::GetResource_ov_128_png());
  gui::AddIcon(ov::GetResource_ov_256_png());
  gui::AddIcon(ov::GetResource_ov_512_png());

  gui::AddLateExecute(DisplayGui);

  gui::Initialize("wpical", 600, 400);
  gui::Main();

  gui::DestroyContext();

  return 0;
}
