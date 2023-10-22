// Molecular dynamics playground

#include "imgui.h"
#include "implot.h"
#include <cmath>
#include "app.h"
#include <math/vector.h>
#include <math/matrix.h>
#include <math/noise.h>
#include <numbers>
#include <random>

static constexpr auto Pi = std::numbers::pi_v<double>;
static constexpr auto TwoPi = 2 * std::numbers::pi_v<double>;

using namespace math;

struct RenderShape
{
    RenderShape(const std::string& name) : m_name(name) {}
    virtual void Render() const = 0;


    const std::string m_name;
};

struct RenderCircle : RenderShape
{
    RenderCircle(const std::string& name, float radius)
        : RenderShape(name)
        , m_pos{}
        , m_radius(radius)
    {}

    void Render() const override
    {
        float x[kNumSegments + 1];
        float y[kNumSegments + 1];
        for (int i = 0; i < kNumSegments + 1; ++i)
        {
            auto theta = i * TwoPi / kNumSegments;
            x[i] = m_radius * cos(theta) + m_pos.x();
            y[i] = m_radius * sin(theta) + m_pos.y();
        }
        ImPlot::PlotLine(m_name.c_str(), x, y, kNumSegments + 1);
    }

    math::Vec2f m_pos;
    float m_radius;

    static constexpr inline size_t kNumSegments = 32;
};

struct RenderLine : RenderShape
{
    RenderLine(const std::string& name)
        : RenderShape(name)
        , a{}
        , b{}
    {}

    void Render() const override
    {
        float x[2] = { float(a.x()), float(b.x()) };
        float y[2] = { float(a.y()), float(b.y()) };

        ImPlot::PlotLine(m_name.c_str(), x, y, 2);
    }

    math::Vec2f a, b;
};

struct RigidBody
{
    // Parameters
    float m_InvMass;
    float m_InvInertia;
    Vec2f m_CenterOfMass{};

    // State
    Vec2f m_Position{};
    Vec2f m_LinearVelocity{};
    float m_Angle = 0;
    float m_AngularVelocity = 0;

    // Forces
    Vec2f m_AccumForces{};
    float m_AccumTorque{};

    void ResetForces()
    {
        m_AccumForces = {};
        m_AccumTorque = 0;
    }

    void ApplyForce(const Vec2f& force, const Vec2f& relativePos)
    {
        Vec2f arm = relativePos - m_CenterOfMass;
        float torque = arm.x() * force.y() - force.x() * arm.y();
        m_AccumForces += force;
        m_AccumTorque += torque;
    }

    void ApplyForce(const Vec2f & force)
    {
        m_AccumForces += force;
    }

    void Integrate(float dt)
    {
        Vec2f linearAcceleration = m_AccumForces * m_InvMass;
        float angularAcceleration = m_AccumTorque * m_InvInertia;

        // Basic euler integration
        m_Position += m_LinearVelocity * dt + 0.5 * linearAcceleration * (dt*dt);
        m_Angle += m_AngularVelocity * dt + 0.5 * angularAcceleration * (dt*dt);
        m_LinearVelocity += linearAcceleration * dt;
        m_AngularVelocity += angularAcceleration * dt;
    }
};

struct RigidBodyWorld
{
    static inline RigidBodyWorld* sInstance = nullptr;
    static void Init() {
        sInstance = new RigidBodyWorld();
    }
    static RigidBodyWorld* Get() { return sInstance; }

    void AddRigidBody(RigidBody& body)
    {
        m_bodies.push_back(&body);
    }

    void RemoveRigidBody(RigidBody& body)
    {
        for (size_t i = 0; i < m_bodies.size(); ++i)
        {
            if (m_bodies[i] == &body)
            {
                m_bodies[i] = m_bodies.back();
                m_bodies.pop_back();
                return;
            }
        }
    }

    void Update(float dt)
    {
        // Add gravity to every body
        for (auto body : m_bodies)
        {
            body->ApplyForce(Vec2f(0.f, -9.81 / body->m_InvInertia));
        }

        // Update simulation
        m_stepResidual += dt;
        while (m_stepResidual > m_fixedStepSize)
        {
            m_stepResidual -= m_fixedStepSize;
            Step();
        }

        // Clear forces
        for (auto body : m_bodies)
        {
            body->ResetForces();
        }
    }

    float m_fixedStepSize = 0.01f;

private:
    void Step()
    {
        for (auto body : m_bodies)
        {
            body->Integrate(m_fixedStepSize);
        }
    }

    float m_stepResidual = 0;
    std::vector<RigidBody*> m_bodies;
};

struct Particle
{
    Particle(const std::string& name, float mass, float radius, const Vec2f& pos)
    {
        auto invMass = mass ? 1 / mass : 0;
        
        m_rigidBody = std::make_unique<RigidBody>();
        m_rigidBody->m_InvMass = invMass;
        m_rigidBody->m_InvInertia = invMass; // TODO: Use the correct inertia distribution based on shape and size
        m_rigidBody->m_Position = pos;
        RigidBodyWorld::Get()->AddRigidBody(*m_rigidBody);

        m_renderer = std::make_unique<RenderCircle>(name, radius);
    }

    ~Particle()
    {
        RigidBodyWorld::Get()->RemoveRigidBody(*m_rigidBody);
    }

    void Update(float dt)
    {
        m_renderer->m_pos = m_rigidBody->m_Position;
    }

    void Render()
    {
        m_renderer->Render();
    }

    std::unique_ptr<RigidBody> m_rigidBody;
    std::unique_ptr<RenderCircle> m_renderer;
};

class SegwayApp : public App
{
public:

    SegwayApp()
    {
        RigidBodyWorld::Init();

        float a = -5;
        float b = 5;
        m_Particles.push_back(std::make_unique<Particle>("p0", 1.f, 1.f, Vec2f(m_rng.uniform(a, b), m_rng.uniform(a, b))));
        m_Particles.push_back(std::make_unique<Particle>("p1", 1.f, 1.f, Vec2f(m_rng.uniform(a, b), m_rng.uniform(a, b))));
        m_Particles.push_back(std::make_unique<Particle>("p2", 1.f, 1.f, Vec2f(m_rng.uniform(a, b), m_rng.uniform(a, b))));
        m_Particles.push_back(std::make_unique<Particle>("p3", 1.f, 1.f, Vec2f(m_rng.uniform(a, b), m_rng.uniform(a, b))));
    }

    void resetSimulation()
    {
        float a = -5;
        float b = 5;
        for (auto& p : m_Particles)
        {
            p->m_rigidBody->m_LinearVelocity = {};
            p->m_rigidBody->m_Position = Vec2f(m_rng.uniform(a, b), m_rng.uniform(a, b));
        }
    }

    void update() override
    {
        float dt = 0.01666f; // TODO: Get this from std::chrono

        // Plot params
        if (ImGui::CollapsingHeader("Control"))
        {
            ImGui::Checkbox("Running", &m_RunningSim);
            if (ImGui::Button("Reset"))
            {
                resetSimulation();
            }
        }

        // Advance physics simulation
        if (m_RunningSim)
        {
            RigidBodyWorld::Get()->Update(dt);
        }

        // Update renderers
        for (auto& p : m_Particles)
        {
            p->Update(dt);
        }

        // Display results
        if(ImGui::Begin("Simulation"))
        {
            if(ImPlot::BeginPlot("SimViewer", ImVec2(-1, -1), ImPlotFlags_Equal))
            {
                // Set up rigid axes
                float size = 20.f;
                ImPlot::SetupAxis(ImAxis_X1, NULL, ImPlotAxisFlags_AuxDefault);
                ImPlot::SetupAxisLimits(ImAxis_X1, -size, size, ImGuiCond_Always);
                ImPlot::SetupAxis(ImAxis_Y1, NULL, ImPlotAxisFlags_AuxDefault);

                for (auto& p : m_Particles)
                {
                    p->Render();
                }
            }
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

private:
    bool m_RunningSim = false;
    SquirrelRng m_rng;
    std::vector<std::unique_ptr<Particle>> m_Particles;
};

// Main code
int main(int, char**)
{
    SegwayApp app;
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
