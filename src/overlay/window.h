#pragma once

#include "calibration.h"
#include "ipc_client.h"
#include "platform.h"

#if OS_WINDOWS
#define GLFW_EXPOSE_NATIVE_WIN32
#endif // OS_WINDOWS
#include <GLFW/glfw3.h>
#if OS_WINDOWS
#include <GLFW/glfw3native.h>
#endif // OS_WINDOWS

#include "renderer/renderer.h"
#include <map>

namespace spacecal {

// handles window creation, setting the icon, and initialising a renderer for UI, also handles connecting it to SteamVR as an overlay
class Window {
public:
    // startMinimised: the desktop window starts minimised, not taking the focus (when SteamVR starts Space Calibrator)
    bool CreateNativeWindow(renderer::GraphicsBackend gfxApi, bool startMinimised);
    void SetupImGuiStyle();
    void Shutdown();
    void RunLoop();

private:
    renderer::GraphicsBackend m_desiredGraphicsBackend = renderer::GraphicsBackend::OpenGL;
    renderer::IRenderContext* m_graphicsContext = nullptr;

    GLFWwindow* m_glfwWindow = nullptr;
    int m_windowWidth = 1580;
    int m_windowHeight = 950;

    bool m_isYFlipped = false;

    // the fonts' files, mapped: the UI reads only the few glyphs it draws out of them, where loading them read some 50 MB (the
    // Chinese, Japanese and Korean fonts) into memory for as long as Space Calibrator ran
    std::map<std::string, platform::MappedFile_t> m_fontFiles;
    ImFont* addFont(const std::string& fileName, const ImFontConfig& cfg);

    const float k_MINIMIZED_MAX_FPS = 60.0f;
    // a desktop window nobody is using (not under the mouse, not focused) is drawn this often: as often as what it shows changes,
    // the calibration's tick. one in use, and the dashboard, as often as before
    static constexpr double k_IDLE_WINDOW_FRAME_INTERVAL_SEC = 0.045;
    // when nothing in use is drawn, the loop wakes this long after the calibration's tick is due. a tick has come at the first
    // turn of the loop 50 ms after the last, which with the loop turning every 11 ms or so made them some 57 ms apart (in the
    // logs of 18 Nov: a batch every 2.85 s at the least, 50 samples): waking 7 ms late keeps them there, where waking at once
    // would make the batches of samples shorter in time
    static constexpr double k_TICK_WAKE_MARGIN_SEC = 0.007;
};
}
