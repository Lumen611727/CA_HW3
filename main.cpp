#define GLAD_GL_IMPLEMENTATION
#include <glad/gl.h>
#undef GLAD_GL_IMPLEMENTATION
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include "fluid.h"
#include "simulation_defaults.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <fstream>
#include <vector>

namespace {
constexpr int W = 1440, H = 900;
// TODO: replace YOUR_STUDENT_ID with your student ID, then rebuild.
// This value appears in the operating system's window title bar.
constexpr char STUDENT_ID[] = "315553006";
float brush = 28, flip = SimulationDefaults::FlipRatio, step = SimulationDefaults::TimeStep;
float gravity = SimulationDefaults::Gravity, relaxation = SimulationDefaults::OverRelaxation;
float stiffness = SimulationDefaults::Stiffness;
int iterations = SimulationDefaults::Iterations, render_mode = 2;
bool paused = false, density = SimulationDefaults::DensityCorrection;
Fluid* water = nullptr;
ImFont* titleFont = nullptr;
void key(GLFWwindow* w, int k, int, int action, int) {
    if (action != GLFW_PRESS || ImGui::GetIO().WantCaptureKeyboard) return;
    if (k == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(w, true);
    if (k == GLFW_KEY_R) water->resetScene();
    if (k == GLFW_KEY_SPACE) paused = !paused;
    if (k == GLFW_KEY_1) render_mode = 2;
    if (k == GLFW_KEY_2) render_mode = 0;
    if (k == GLFW_KEY_3) render_mode = 1;
}
void scroll(GLFWwindow*, double, double amount) {
    if (!ImGui::GetIO().WantCaptureMouse) brush = std::clamp(brush + float(amount) * 3, 10.0f, 65.0f);
}
void panel() {
    ImGui::SetNextWindowPos(ImVec2(1070, 44), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(330, 810), ImGuiCond_Always);
    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
    ImGui::TextColored(ImVec4(0.57f, 0.78f, 0.70f, 1), "A LITTLE DIG. A HAPPY CROC.");
    ImGui::Spacing();
    ImGui::PushFont(titleFont);
    ImGui::TextUnformatted("Croc's\nWaterworks");
    ImGui::PopFont();
    ImGui::TextDisabled("PIC / FLIP  -  interactive fluid puzzle");
    ImGui::Dummy(ImVec2(0, 12));
    ImGui::TextWrapped("Carve a path through the soil. Guide the water into the pipe on the right to give our little croc a bath.");
    ImGui::Dummy(ImVec2(0, 10));
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.87f, 0.76f, 0.53f, 1), "01   MAKE A PATH");
    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.94f, 0.86f, 0.66f, 1), "DIG MODE");
    ImGui::SetNextItemWidth(182);
    ImGui::SliderFloat("Brush", &brush, 10, 65, "%.0f px");
    ImGui::TextDisabled("Drag to carve. Scroll to resize.");
    ImGui::TextWrapped("Teal stone walls are permanent. Dig around them to reach the pipe inlet.");
    ImGui::Dummy(ImVec2(0, 10));
    ImGui::Separator(); ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.87f, 0.76f, 0.53f, 1), "02   FILL THE BATH");
    ImGui::Spacing();
    char progress[48];
    const float collectedRatio = water->getCollectedRatio();
    std::snprintf(progress, sizeof(progress), "Collected water  %.0f%%", collectedRatio * 100);
    ImGui::ProgressBar(collectedRatio, ImVec2(-1, 25), progress);
    ImGui::Text("Water collected  %d / %d",
                water->getCollectedParticleCount(), water->getInitialParticleCount());
    ImGui::TextDisabled("Clear threshold: %.0f%%    Great: 85%%    Perfect: 95%%",
                        water->getClearThreshold() * 100);
    ImGui::TextDisabled("In pipe  %d     In scene  %d", water->getPipeParticleCount(), water->getActiveParticleCount());
    if (water->isLevelCleared()) {
        ImGui::TextColored(ImVec4(0.63f, 0.92f, 0.56f, 1), "BATH TIME!  %d / 3 stars", water->getStarRating());
    } else ImGui::TextDisabled("70%% clear   /   85%% great   /   95%% perfect");
    ImGui::Dummy(ImVec2(0, 10));
    if (ImGui::Button(paused ? "Resume  [SPACE]" : "Pause  [SPACE]", ImVec2(140, 37))) paused = !paused;
    ImGui::SameLine();
    if (ImGui::Button("Restart  [R]", ImVec2(140, 37))) water->resetScene();
    ImGui::Dummy(ImVec2(0, 8));
    if (ImGui::CollapsingHeader("Simulation settings")) {
        if (ImGui::Button("Restore defaults + restart")) {
            flip = SimulationDefaults::FlipRatio;
            gravity = SimulationDefaults::Gravity;
            step = SimulationDefaults::TimeStep;
            stiffness = SimulationDefaults::Stiffness;
            relaxation = SimulationDefaults::OverRelaxation;
            iterations = SimulationDefaults::Iterations;
            density = SimulationDefaults::DensityCorrection;
            water->resetScene();
        }
        ImGui::SetNextItemWidth(140); ImGui::SliderFloat("FLIP", &flip, 0, 1);
        ImGui::SetNextItemWidth(140); ImGui::SliderFloat("Gravity", &gravity, 0, 100);
        ImGui::SetNextItemWidth(140); ImGui::SliderFloat("Time step", &step, 0.005f, 0.05f, "%.3f");
        ImGui::SetNextItemWidth(140); ImGui::InputFloat("Stiffness", &stiffness, 0.1f);
        ImGui::SetNextItemWidth(140); ImGui::SliderFloat("Relaxation", &relaxation, 1.0f, 2.0f);
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 22);
            ImGui::TextUnformatted("Pressure-solver correction multiplier (SOR omega), not wall permeability. "
                "Default: 1.85. Near 1, more iterations may be needed. "
                "High gravity, FLIP and time step can expose collision limitations. "
                "Restore defaults and restart after a leak.");
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
        ImGui::SetNextItemWidth(140); ImGui::SliderInt("Iterations", &iterations, 1, 10);
        ImGui::Checkbox("Density correction", &density);
        ImGui::RadioButton("Water", &render_mode, 2); ImGui::SameLine();
        ImGui::RadioButton("Cells", &render_mode, 0); ImGui::SameLine();
        ImGui::RadioButton("Both", &render_mode, 1);
        ImGui::TextDisabled("%.0f FPS", ImGui::GetIO().Framerate);
    }
    ImGui::End();
}
}

int main(int argc, char** argv)
{
    if (argc > 1 && std::strcmp(argv[1], "--self-test") == 0) return Fluid::selfTest() ? 0 : 1;
    bool preview = argc > 2 && std::strcmp(argv[1], "--preview") == 0;
    glfwSetErrorCallback([](int, const char* message) { std::fprintf(stderr, "%s\n", message); });
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    if (preview) glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    GLFWwindow* w = glfwCreateWindow(W, H, STUDENT_ID, nullptr, nullptr);
    if (!w) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(w);
    glfwSwapInterval(1);
    if (!gladLoadGL(glfwGetProcAddress)) { glfwDestroyWindow(w); glfwTerminate(); return 1; }
    glfwSetKeyCallback(w, key);
    glfwSetScrollCallback(w, scroll);
    ImGui::CreateContext();
    auto& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    ImFont* font = nullptr;
    if (std::ifstream("C:/Windows/Fonts/segoeui.ttf").good())
        font = io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeui.ttf", 17);
    if (!font) font = io.Fonts->AddFontDefault();
    if (std::ifstream("C:/Windows/Fonts/segoeuib.ttf").good())
        titleFont = io.Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeuib.ttf", 36);
    if (!titleFont) titleFont = font;
    io.FontDefault = font;
    ImGui::StyleColorsDark();
    auto& style = ImGui::GetStyle();
    style.WindowPadding = ImVec2(16, 18);
    style.ItemSpacing = ImVec2(10, 10);
    style.FramePadding = ImVec2(10, 7);
    style.WindowRounding = 14; style.FrameRounding = 6; style.WindowBorderSize = 0;
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.075f, 0.13f, 0.16f, 1);
    style.Colors[ImGuiCol_Button] = ImVec4(0.18f, 0.32f, 0.34f, 1);
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.25f, 0.45f, 0.43f, 1);
    style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.32f, 0.55f, 0.47f, 1);
    style.Colors[ImGuiCol_FrameBg] = ImVec4(0.11f, 0.22f, 0.26f, 1);
    style.Colors[ImGuiCol_PlotHistogram] = ImVec4(0.36f, 0.73f, 0.71f, 1);
    style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.65f, 0.84f, 0.69f, 1);
    ImGui_ImplGlfw_InitForOpenGL(w, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    Fluid simulation(SimulationDefaults::ParticleCount, SimulationDefaults::ParticleRadius,
                     flip, SimulationDefaults::CellSize, SimulationDefaults::RelaxationCellSize,
                     iterations, SimulationDefaults::DomainWidth, SimulationDefaults::DomainHeight,
                     step, gravity, stiffness, density, relaxation);
    water = &simulation;
    if (preview && argc > 3 && std::strcmp(argv[3], "--flow") == 0) {
        const Eigen::Vector2f route[] = {{680, 535}, {710, 495}, {735, 465}, {850, 455}, {900, 400}, {900, 335}, {904, 300}};
        for (int segment = 1; segment < 7; ++segment)
            for (int k = 0; k <= 30; ++k) {
                Eigen::Vector2f q = route[segment - 1] + (route[segment] - route[segment - 1]) * (k / 30.0f);
                water->editTerrain(q.x(), H - q.y(), 38);
            }
        bool showCleared = argc > 4 && std::strcmp(argv[4], "--cleared") == 0;
        for (int i = 0; i < 4000 && (showCleared ? water->getCollectedRatio() < 0.90f : water->getCollectedParticleCount() < 260); ++i)
            water->update();
    }
    bool wasDown = false;
    double lastX = 0, lastY = 0;
    int frames = 0;
    while (!glfwWindowShouldClose(w)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
        panel();
        double mx, my; glfwGetCursorPos(w, &mx, &my);
        bool down = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && !io.WantCaptureMouse;
        if (down) {
            if (!wasDown) { lastX = mx; lastY = my; }
            int samples = std::max(1, int(std::hypot(mx - lastX, my - lastY) / (brush * 0.35)));
            for (int i = 1; i <= samples; ++i) {
                float t = float(i) / samples;
                water->editTerrain(float(lastX + (mx - lastX) * t), float(lastY + (my - lastY) * t), brush);
            }
        }
        wasDown = down; lastX = mx; lastY = my;
        water->setFlipRatio(flip); water->setGravity(gravity); water->setDt(step);
        water->setIterations(iterations); water->setDensityCorrection(density);
        water->setStiffnessCoeff(stiffness); water->setOverRelaxation(relaxation);
        // Original HW3 cadence: exactly one PIC/FLIP update per rendered frame.
        if (!paused) water->update();
        water->drawScene(render_mode);
        if (!io.WantCaptureMouse && mx > 100 && mx < 1000 && my > 100 && my < 830) {
            ImGui::GetForegroundDrawList()->AddCircle(ImVec2(float(mx), float(my)), brush,
                IM_COL32(243, 226, 180, 200), 40, 1.5f);
        }
        int fw, fh; glfwGetFramebufferSize(w, &fw, &fh);
        glViewport(0, 0, fw, fh); glClearColor(0.035f, 0.07f, 0.095f, 1); glClear(GL_COLOR_BUFFER_BIT);
        ImGui::Render(); ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        if (preview && ++frames == 3) {
            std::vector<unsigned char> pixels(fw * fh * 3);
            glPixelStorei(GL_PACK_ALIGNMENT, 1); glReadPixels(0, 0, fw, fh, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
            std::ofstream out(argv[2], std::ios::binary);
            out << "P6\n" << fw << ' ' << fh << "\n255\n";
            for (int y = fh - 1; y >= 0; --y) out.write(reinterpret_cast<char*>(pixels.data() + y * fw * 3), fw * 3);
            glfwSetWindowShouldClose(w, true);
        }
        glfwSwapBuffers(w);
    }
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown(); ImGui::DestroyContext();
    glfwDestroyWindow(w); glfwTerminate(); return 0;
}
