// Stand in pendulum simulator

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "gazebo_msgs/ModelState.h"
#include <atomic>
#include <math/noise.h>
#include <cmath>
#include <mutex>

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
        
        void randomize(math::LinearCongruentalGenerator& rng)
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

        void perturbate(math::LinearCongruentalGenerator& rng)
        {
            dTheta += rng.uniform() - 0.5;
        }
        
        void randomize(math::LinearCongruentalGenerator& rng)
        {
            theta = rng.uniform() * 2 * 3.1415927;
        }
    } m_state;
    
    static constexpr auto g = 9.81;

    void stepSimulation(double stepDt, double controllerInput)
    {
        auto& p = m_params;
        auto& x = m_state;

        auto u = controllerInput;

        auto b = p.b1;
        auto torque = u -p.b1 * x.dTheta - sin(x.theta) * g * p.l1;

        const auto invInertia = p.I1 > 0 ? (1 / p.I1) : 0;
        auto ddq = torque * invInertia;

        x.theta += stepDt * x.dTheta + 0.5 *ddq * stepDt * stepDt;
        x.dTheta += ddq * stepDt;
    }
};

using StateMsg = gazebo_msgs::ModelState;
using ControlMsg = geometry_msgs::Wrench;
using PoseMsg = geometry_msgs::Pose;

int main(int argc, char **argv)
{
    // Init ROS
    ros::init(argc, argv, "pendulum_sim");
    ros::NodeHandle nodeHandle;

    // Control action
    std::atomic<double> torque{};
    
    // Set up ROS topics for this node
    constexpr auto cQueueSize = 1;
    auto statePublisher = nodeHandle.advertise<StateMsg>("pendulum_x", cQueueSize);
    auto subscriber = nodeHandle.subscribe<ControlMsg>("control_u", 1,
        [&](const ControlMsg::ConstPtr& controlAction){
            torque = controlAction->torque.z;
        }
    );

    // Initialize a pendulum
    math::LinearCongruentalGenerator rng;
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
        // Simulate
        pendulum.stepSimulation(stepDt, torque);

        // Publish state
        StateMsg stateUpdate;
        stateUpdate.pose.orientation.w = pendulum.m_state.theta;
        statePublisher.publish(stateUpdate);

        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}