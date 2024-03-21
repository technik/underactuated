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

template<class Derived>
struct Singleton
{
    static inline Derived* sInstance = nullptr;
    static void Init() {
        sInstance = new Derived();
    }
    static Derived* Get() { return sInstance; }
};

struct RenderShape
{
    RenderShape(const std::string& name) : m_name(name) {}
    virtual void Render() const = 0;

    const std::string m_name;
};

struct Presentation : Singleton<Presentation>
{
    void AddShape(RenderShape& shape)
    {
        m_Shapes.push_back(&shape);
    }

    void RemoveShape(RenderShape& shape)
    {
        for (size_t i = 0; i < m_Shapes.size(); ++i)
        {
            if (m_Shapes[i] == &shape)
            {
                m_Shapes[i] = m_Shapes.back();
                m_Shapes.pop_back();
                return;
            }
        }
    }

    void Render()
    {
        // Set up viewport
        float size = 20.f;
        ImPlot::SetupAxis(ImAxis_X1, NULL, ImPlotAxisFlags_AuxDefault);
        ImPlot::SetupAxisLimits(ImAxis_X1, -size, size, ImGuiCond_Always);
        ImPlot::SetupAxis(ImAxis_Y1, NULL, ImPlotAxisFlags_AuxDefault);

        // Render all shapes
        for (auto& shape : m_Shapes)
        {
            shape->Render();
        }
    }

private:
    std::vector<RenderShape*> m_Shapes;
};

struct Circle : RenderShape
{
    Circle(const std::string& name, float radius)
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
            auto theta = i * math::TwoPi / kNumSegments;
            x[i] = m_radius * cos(theta) + m_pos.x();
            y[i] = m_radius * sin(theta) + m_pos.y();
        }
        ImPlot::SetNextLineStyle(m_Colliding ? kRed : m_Color);
        ImPlot::PlotLine(m_name.c_str(), x, y, kNumSegments + 1);
    }

    // Params
    ImVec4 m_Color = ImVec4(0.5f, 0.5f, 0.5f, 0.5f);
    float m_radius;

    // State
    math::Vec2f m_pos;
    bool m_Colliding = false;

    static inline const ImVec4 kRed = ImVec4(1.f, 0.f, 0.f, 1.f);
    static constexpr inline size_t kNumSegments = 32;
};

struct KinematicAABB : RenderShape
{
    KinematicAABB(const std::string& name, const Vec2f& _min, const Vec2f& _max)
        : RenderShape(name)
        , m_Min(_min)
        , m_Max(_max)
    {
    }

    void Render() const override
    {
        float x[5] = { m_Min.x(), m_Min.x(), m_Max.x(), m_Max.x(), m_Min.x() };
        float y[5] = { m_Max.y(), m_Min.y(), m_Min.y(), m_Max.y(), m_Max.y() };

        ImPlot::SetNextLineStyle(m_Color);
        ImPlot::PlotLine(m_name.c_str(), x, y, 5);
    }

    // Params
    ImVec4 m_Color = ImVec4(0.5f, 0.5f, 0.5f, 0.5f);
    Vec2f m_Min, m_Max;
};

bool intersect(const Circle& a, const Circle& b)
{
    float R = a.m_radius + b.m_radius;
    Vec2f relPos = b.m_pos - a.m_pos;
    return relPos.sqNorm() <= R * R;
}

bool intersect(const Circle& a, const Vec2f& pos)
{
    Vec2f relPos = pos - a.m_pos;
    return relPos.sqNorm() <= a.m_radius * a.m_radius;
}

bool intersect(const KinematicAABB& aabb, const Circle& c)
{
    // Find the point in the AABB closest to the circle
    auto x = c.m_pos.x();
    auto y = c.m_pos.y();
    x = std::min(x, aabb.m_Max.x());
    y = std::min(y, aabb.m_Max.y());
    x = std::max(x, aabb.m_Min.x());
    y = std::max(y, aabb.m_Min.y());

    return intersect(c, Vec2f(x, y));
}

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

struct ForceGenerator
{
    virtual void ApplyForces() = 0;
};

struct Spring : ForceGenerator
{
    Spring(RigidBody& a, RigidBody& b, float restLength, float k) // TODO: Support body offsets
        : m_a(&a)
        , m_b(&b)
        , m_restLength(restLength)
        , m_k(k)
    {
    }

    void ApplyForces() override
    {
        Vec2f deltaPos = m_b->m_Position - m_a->m_Position;
        float len = deltaPos.norm();
        Vec2f F = (len ? ((len - m_restLength)/len * m_k) : 0) * deltaPos;
        m_b->ApplyForce(-F);
        m_a->ApplyForce(F);
    }

    RigidBody* m_a;
    RigidBody* m_b;
    float m_restLength;
    float m_k;
};

struct Constraint
{
    virtual void ApplyConstraintForce() = 0;
};

struct PivotConstraint : Constraint
{
    PivotConstraint(RigidBody& body, const Vec2f& pivotPos, float distance)
        : m_body(&body)
        , m_pivotPos(pivotPos)
        , m_distance(distance)
    {
        assert(body.m_InvMass != 0);
    }

    void ApplyConstraintForce() override
    {
        Vec2f relPos = m_body->m_Position - m_pivotPos;
        Vec2f v = m_body->m_LinearVelocity;
        float r2 = relPos.sqNorm();
        float v2 = v.sqNorm();
        float work = dot(m_body->m_AccumForces, relPos);
        float invMass = m_body->m_InvMass;
        float lambda = -(work * invMass + v2) / (r2 * invMass);
        m_body->ApplyForce(lambda * relPos);
    }

    RigidBody* m_body;
    Vec2f m_pivotPos;
    float m_distance;
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
    
    void AddKinematicBody(KinematicAABB& body)
    {
        m_KinematicBodies.push_back(&body);
    }

    void RemoveKinematicBody(KinematicAABB& body)
    {
        for (size_t i = 0; i < m_KinematicBodies.size(); ++i)
        {
            if (m_KinematicBodies[i] == &body)
            {
                m_KinematicBodies[i] = m_KinematicBodies.back();
                m_KinematicBodies.pop_back();
                return;
            }
        }
    }

    void AddCollider(Circle& collider)
    {
        m_circleColliders.push_back(&collider);
    }

    void RemoveCollider(Circle& collider)
    {
        for (size_t i = 0; i < m_circleColliders.size(); ++i)
        {
            if (m_circleColliders[i] == &collider)
            {
                m_circleColliders[i] = m_circleColliders.back();
                m_circleColliders.pop_back();
                return;
            }
        }
    }

    void AddForceGenerator(ForceGenerator& generator)
    {
        m_ForceGenerators.push_back(&generator);
    }

    void RemoveForceGenerator(ForceGenerator& generator)
    {
        for (size_t i = 0; i < m_ForceGenerators.size(); ++i)
        {
            if (m_ForceGenerators[i] == &generator)
            {
                m_ForceGenerators[i] = m_ForceGenerators.back();
                m_ForceGenerators.pop_back();
                return;
            }
        }
    }

    void AddConstraint(Constraint& constraint)
    {
        m_Constraints.push_back(&constraint);
    }

    void RemoveConstraint(Constraint& constraint)
    {
        for (size_t i = 0; i < m_Constraints.size(); ++i)
        {
            if (m_Constraints[i] == &constraint)
            {
                m_Constraints[i] = m_Constraints.back();
                m_Constraints.pop_back();
                return;
            }
        }
    }

    void Update(float dt)
    {
        // Update simulation
        m_stepResidual += dt;
        while (m_stepResidual > m_fixedStepSize)
        {
            m_stepResidual -= m_fixedStepSize;
            Step();
        }
    }

    float m_fixedStepSize = 0.01f;

private:
    void Step()
    {
        // Clear collisions
        for (auto collider : m_circleColliders)
        {
            collider->m_Colliding = false;
        }

        // Detect collisions
        for (size_t i = 0; i < m_circleColliders.size(); ++i)
        {
            for (size_t j = i + 1; j < m_circleColliders.size(); ++j)
            {
                if (intersect(*m_circleColliders[i], *m_circleColliders[j]))
                {
                    m_circleColliders[i]->m_Colliding = true;
                    m_circleColliders[j]->m_Colliding = true;
                }
            }
        }
        for (size_t i = 0; i < m_circleColliders.size(); ++i)
        {
            for (size_t j = 0; j < m_KinematicBodies.size(); ++j)
            {
                if (intersect(*m_KinematicBodies[j], *m_circleColliders[i]))
                {
                    m_circleColliders[i]->m_Colliding = true;
                }
            }
        }

        // Add gravity to every body
        for (auto body : m_bodies)
        {
            if (body->m_InvMass > 0)
                body->ApplyForce(Vec2f(0.f, -9.81 / body->m_InvMass));
        }

        // Apply custom force generators
        for (auto generator : m_ForceGenerators)
        {
            generator->ApplyForces();
        }

        // Apply constraints force generators
        for (auto c : m_Constraints)
        {
            c->ApplyConstraintForce();
        }

        // Integrate trajectories
        for (auto body : m_bodies)
        {
            body->Integrate(m_fixedStepSize);
        }

        // Clear forces
        for (auto body : m_bodies)
        {
            body->ResetForces();
        }
    }

    float m_stepResidual = 0;
    std::vector<RigidBody*> m_bodies;
    std::vector<Circle*> m_circleColliders;
    std::vector<KinematicAABB*> m_KinematicBodies;
    std::vector<ForceGenerator*> m_ForceGenerators;
    std::vector<Constraint*> m_Constraints;
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

        m_renderer = std::make_unique<Circle>(name, radius);
        Presentation::Get()->AddShape(*m_renderer);
        RigidBodyWorld::Get()->AddCollider(*m_renderer);
    }

    ~Particle()
    {
        RigidBodyWorld::Get()->RemoveRigidBody(*m_rigidBody);
        RigidBodyWorld::Get()->RemoveCollider(*m_renderer);
        Presentation::Get()->RemoveShape(*m_renderer);
    }

    void Update()
    {
        m_renderer->m_pos = m_rigidBody->m_Position;
    }

    void Render()
    {
        m_renderer->Render();
    }

    std::unique_ptr<RigidBody> m_rigidBody;
    std::unique_ptr<Circle> m_renderer;
};

struct Obstacle
{
    Obstacle(const std::string& name, const Vec2f& _min, const Vec2f& _max)
    {
        m_box = std::make_unique<KinematicAABB>(name, _min, _max);
        RigidBodyWorld::Get()->AddKinematicBody(*m_box);
        Presentation::Get()->AddShape(*m_box);
    }

    ~Obstacle()
    {
        RigidBodyWorld::Get()->RemoveKinematicBody(*m_box);
        Presentation::Get()->RemoveShape(*m_box);
    }

    std::unique_ptr<KinematicAABB> m_box;
};

class SegwayApp : public App
{
public:

    SegwayApp()
    {
        RigidBodyWorld::Init();
        Presentation::Init();

        float a = -5;
        float b = 5;
        m_Particles.push_back(std::make_unique<Particle>("p0", 0.f, 1.f, Vec2f(m_rng.uniform(a, b), m_rng.uniform(a, b)))); // kinematic
        m_Particles.push_back(std::make_unique<Particle>("p1", 1.f, 1.f, Vec2f(m_rng.uniform(a, b), m_rng.uniform(a, b))));
        m_Particles.push_back(std::make_unique<Particle>("p2", 1.f, 1.f, Vec2f(m_rng.uniform(a, b), m_rng.uniform(a, b))));
        m_Particles.push_back(std::make_unique<Particle>("p3", 1.f, 1.f, Vec2f(m_rng.uniform(a, b), m_rng.uniform(a, b))));

        const float armLen = 3;
        m_Constraints.push_back(std::make_unique<PivotConstraint>(*m_Particles[3]->m_rigidBody, m_Particles[3]->m_rigidBody->m_Position + Vec2f(armLen, 0), armLen));
        RigidBodyWorld::Get()->AddConstraint(*m_Constraints.back());

        m_Spring = std::make_unique<Spring>(*m_Particles[0]->m_rigidBody, *m_Particles[1]->m_rigidBody, 4.f, 10.f);
        RigidBodyWorld::Get()->AddForceGenerator(*m_Spring);

        m_Obstacles.push_back(std::make_unique<Obstacle>("ground", Vec2f(-6.f, -8.f), Vec2f(6.f, -7.f)));
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
        // Plot params
        if (ImGui::CollapsingHeader("Control"))
        {
            ImGui::Checkbox("Running", &m_RunningSim);
            if (ImGui::Button("Reset"))
            {
                resetSimulation();
            }
        }

        const float dt = m_RunningSim ? 0.01666f : 0; // TODO: Get this from std::chrono

        // Advance physics simulation
        RigidBodyWorld::Get()->Update(dt);

        // Update renderers
        for (auto& p : m_Particles)
        {
            p->Update();
        }

        // Display results
        if(ImGui::Begin("Simulation"))
        {
            if(ImPlot::BeginPlot("SimViewer", ImVec2(-1, -1), ImPlotFlags_Equal))
            {
                Presentation::Get()->Render();
            }
            ImPlot::EndPlot();
        }
        ImGui::End();
    }

private:
    bool m_RunningSim = false;
    SquirrelRng m_rng;
    std::unique_ptr<Spring> m_Spring;
    std::vector<std::unique_ptr<Particle>> m_Particles;
    std::vector<std::unique_ptr<Obstacle>> m_Obstacles;
    std::vector<std::unique_ptr<Constraint>> m_Constraints;
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
