#pragma once

#include "imgui.h"
#include "backends/imgui_impl_win32.h"
#include "backends/imgui_impl_dx12.h"
#include <d3d12.h>
#include <dxgi1_4.h>
#include <tchar.h>

class App
{
public:
    bool init();
    void end();

    void beginFrame();
    void render();

    virtual void update();

private:

    WNDCLASSEX wc;
    HWND hwnd;
    ImVec4 m_clearColor;
};