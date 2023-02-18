// Molecular dynamics playground

#include "imgui.h"
#include "implot.h"
#include <cmath>
#include "app.h"
#include <math/vector.h>
#include <numbers>
#include <random>

static constexpr auto Pi = std::numbers::pi_v<double>;
static constexpr auto TwoPi = 2 * std::numbers::pi_v<double>;

using namespace math;

class PendulumApp : public App
{
public:
    PendulumApp()
    {
    }

    void update() override
    {
        // Plot params
        if (ImGui::CollapsingHeader("Params"))
        {
        }
        // Plot state
        if(ImGui::CollapsingHeader("State"))
        {
            if (ImGui::Button("Randomize"))
            {
            }
            if (ImGui::Button("Perturbate"))
            {
            }
        }
        // randomize state button
        // Bool is paused or running?
        ImGui::Checkbox("Run", &m_isRunningSimulation);
        // if running
        //   get dt
        //   run substeps
        // 
        // Display results
        // Plot state
    }

private:
    struct PendulumParams
    {
        double l1, l2; // Bar lengths
        double m1, m2; // Bar masses
        double b1, b2; // Friction at the joints
        double I1, I2; // Inertia tensors

        void refreshInertia();
    } m_pendulumParams;

    struct PendulumState
    {
        double theta;
        double dTheta;
    } m_pendulumState;

    bool m_isRunningSimulation = false;

    static int squirrelNoise(int position, int seed = 0)
    {
        constexpr unsigned int BIT_NOISE1 = 0xB5297A4D;
        constexpr unsigned int BIT_NOISE2 = 0x68E31DA4;
        constexpr unsigned int BIT_NOISE3 = 0x1B56C4E9;

        int mangled = position;
        mangled *= BIT_NOISE1;
        mangled += seed;
        mangled ^= (mangled >> 8);
        mangled *= BIT_NOISE2;
        mangled ^= (mangled << 8);
        mangled *= BIT_NOISE3;
        mangled ^= (mangled >> 8);
        return mangled;
    }

    struct squirrelRng
    {
        int rand()
        {
            return squirrelNoise(state++);
        }

        int state = 0;
    } m_rng;
};

// Main code
int main(int, char**)
{
    PendulumApp app;
    if (!app.init())
        return -1;

    // Main loop
    bool done = false;
    while (!done)
    {
        // Poll and handle messages (inputs, window resize, etc.)
        // See the WndProc() function below for our to dispatch events to the Win32 backend.
        MSG msg;
        while (::PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE))
        {
            ::TranslateMessage(&msg);
            ::DispatchMessage(&msg);
            if (msg.message == WM_QUIT)
                done = true;
        }
        if (done)
            break;

        app.beginFrame();
        app.update();
        app.render();
    }

    app.end();

    return 0;
}
