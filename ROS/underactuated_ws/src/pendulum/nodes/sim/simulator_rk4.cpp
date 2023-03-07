// Stand in pendulum simulator

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "gazebo_msgs/ModelState.h"
#include <cmath>
#include <mutex>

constexpr double PI = 3.14159265358979323;

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

    float uniform()
    {
        auto i = rand();
        return float(i & ((1 << 24) - 1)) / (1 << 24);
    }

    int state = 0;
};

struct Pendulum
{
    struct Params
    {
        double l1 = 1; // Bar lengths
        double m1 = 1; // Bar masses
        // double b1 = 0; // Friction at the joints
        // double MaxQ = 0; // Torque limit. 0 means unlimited torque
        // double MaxPower = 0; // Power limit. 0 means unlimited power

        // void refreshInertia()
        // {
        //     // Inertia concentrated at the end
        //     I1 = m1 * l1 * l1 / 3;
        // }
        
        void randomize(squirrelRng& rng)
        {
            l1 = rng.uniform() * 10;
            m1 = rng.uniform() * 10;
            // b1 = rng.uniform() * 10;
            // refreshInertia();
        }
    } m_params;

    struct State
    {
        State() : theta(0.0), dTheta(0.0) {}
        State(const double theta, const double dTheta) : theta(theta), dTheta(dTheta) {}
        double theta = 0;
        double dTheta = 0;

        void perturbate(squirrelRng& rng)
        {
            dTheta += rng.uniform() - 0.5;
        }
        
        void randomize(squirrelRng& rng)
        {
            theta = rng.uniform() * 2 * 3.1415927;
        }
        State operator* (double val) const
        {
            return State(val*theta, val*dTheta);
        }
        State operator+ (const State& rhs) const
        {
            return State(theta + rhs.theta, dTheta + rhs.dTheta);
        }
    } m_state;
    
    static constexpr auto g = 9.81;

    State f(const State state, const double u)
    {
        State nextState;
        nextState.theta  = state.dTheta;
        nextState.dTheta = -g/m_params.l1 * std::sin(state.theta) + u/(m_params.m1*m_params.l1*m_params.l1);
        return nextState;
    }
    void stepSimulation(double stepDt, double u)
    {
        const auto k1 = f(m_state, u);
        const auto k2 = f(m_state + (k1*(stepDt/2.0)), u);
        const auto k3 = f(m_state + (k2*(stepDt/2.0)), u);
        const auto k4 = f(m_state + k3*stepDt, u);

        m_state = m_state + (k1 + k2*2.0 + k3*2.0 + k4) * (stepDt/6.0);
        m_state.theta = std::fmod(m_state.theta, 2.0*PI);
    }
};

using StateMsg = gazebo_msgs::ModelState;
using PoseMsg = geometry_msgs::Pose;

int main(int argc, char **argv)
{
    // Init ROS
    ros::init(argc, argv, "pendulum_sim");
    ros::NodeHandle nodeHandle;

    // Set up ROS topics for this node
    constexpr auto cQueueSize = 1;
    auto statePublisher = nodeHandle.advertise<StateMsg>("pendulum_x", cQueueSize);

    // Initialize a pendulum
    squirrelRng rng;
    rng.rand(); // Seed random number generator
    Pendulum pendulum;
    // pendulum.m_params.randomize(rng);
    // pendulum.m_state.randomize(rng);
    pendulum.m_state.theta = PI;
    
    // Set up simulation loop
    constexpr int updateRate = 100; // Hertz
    constexpr double stepDt = 1.0 / updateRate;
    ros::Rate loopRate(100);
    int i = 0;
    while(ros::ok())
    {
        // Random perturbations
        if(++i >= 300)
        {
            i = 0;
            pendulum.m_state.perturbate(rng);
        }
        // Simulate
        const auto u = 0.0;
        pendulum.stepSimulation(stepDt, u);

        // Publish state
        StateMsg stateUpdate;
        stateUpdate.pose.orientation.w = pendulum.m_state.theta;
        stateUpdate.twist.angular.z = pendulum.m_state.dTheta;
        statePublisher.publish(stateUpdate);

        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}