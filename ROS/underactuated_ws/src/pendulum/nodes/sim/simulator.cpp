// Stand in pendulum simulator

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "gazebo_msgs/ModelState.h"
#include <cmath>
#include <mutex>

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
        double b1 = 0; // Friction at the joints
        double I1 = 1; // Inertia tensors
        double MaxQ = 0; // Torque limit. 0 means unlimited torque
        double MaxPower = 0; // Power limit. 0 means unlimited power

        void refreshInertia()
        {
            // Inertia concentrated at the end
            I1 = m1 * l1 * l1 / 3;
        }
        
        void randomize(squirrelRng& rng)
        {
            l1 = rng.uniform() * 10;
            m1 = rng.uniform() * 10;
            b1 = rng.uniform() * 10;
            refreshInertia();
        }
    } m_params;

    struct State
    {
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
    } m_state;
    
    static constexpr auto g = 9.81;

    void stepSimulation(double stepDt)
    {
        auto& p = m_params;
        auto& x = m_state;

        auto u = 0;//computeControllerInput();

        auto b = p.b1;
        auto torque = u -p.b1 * x.dTheta - sin(x.theta) * g * p.l1;

        const auto invInertia = p.I1 > 0 ? (1 / p.I1) : 0;
        auto ddq = torque * invInertia;

        x.theta += stepDt * x.dTheta + 0.5 *ddq * stepDt * stepDt;
        x.dTheta += ddq * stepDt;
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
    pendulum.m_params.randomize(rng);
    pendulum.m_state.randomize(rng);
    
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
            pendulum.m_state.randomize(rng);
        }
        // Simulate
        pendulum.stepSimulation(stepDt);

        // Publish state
        StateMsg stateUpdate;
        stateUpdate.pose.orientation.w = pendulum.m_state.theta;
        statePublisher.publish(stateUpdate);

        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}