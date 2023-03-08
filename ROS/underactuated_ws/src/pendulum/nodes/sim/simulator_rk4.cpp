// 4th Order Runge-Kutta based pendulum simulator

#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "gazebo_msgs/ModelState.h"
#include <cmath>
#include <mutex>
#include <cstdlib>
#include <ctime>

constexpr double PI = 3.14159265358979323;

struct Pendulum
{
    struct Params
    {
        double l1 = 1; // Bar lengths
        double m1 = 1; // Bar masses
        double b1 = 0; // Friction at the joints
    } m_params;

    struct State
    {
        State() : theta(0.0), dTheta(0.0) {}
        State(const double theta, const double dTheta) : theta(theta), dTheta(dTheta) {}
        double theta = 0;
        double dTheta = 0;

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
        nextState.dTheta = -g/m_params.l1 * std::sin(state.theta) + 
                           (u+m_params.b1*state.dTheta)/(m_params.m1*m_params.l1*m_params.l1);
        return nextState;
    }
    void stepSimulation(double stepDt, double u)
    {
        ROS_INFO("U: %f", u);
        const auto k1 = f(m_state, u);
        const auto k2 = f(m_state + (k1*(stepDt/2.0)), u);
        const auto k3 = f(m_state + (k2*(stepDt/2.0)), u);
        const auto k4 = f(m_state + k3*stepDt, u);

        m_state = m_state + (k1 + k2*2.0 + k3*2.0 + k4) * (stepDt/6.0);
        m_state.theta = std::fmod(m_state.theta+2.0*PI, 2.0*PI);
    }
};

int main(int argc, char **argv)
{
    // Init ROS
    ros::init(argc, argv, "pendulum_sim");
    ros::NodeHandle nodeHandle;

    // Set up ROS topics for this node
    constexpr auto cQueueSize = 1;
    auto statePublisher = nodeHandle.advertise<gazebo_msgs::ModelState>("pendulum_x", cQueueSize);

    std::atomic<double> torque{};
    auto subscriber = nodeHandle.subscribe<geometry_msgs::Wrench>("control_u", 1,
        [&](const geometry_msgs::Wrench::ConstPtr& controlAction){
            torque = controlAction->torque.z;
        }
    );

    // Initialize a pendulum
    srand((unsigned) time(0)); // seed the random number generator
    Pendulum pendulum;
    pendulum.m_state.theta = (rand()%100)*2*PI;
    pendulum.m_state.dTheta = (rand()%100)*0.05;
    
    // Set up simulation loop
    constexpr int updateRate = 100; // Hertz
    constexpr double stepDt = 1.0 / updateRate;
    ros::Rate loopRate(updateRate);
    int i = 0;
    while(ros::ok())
    {
        // Simulate
        pendulum.stepSimulation(stepDt, torque.load());

        // Publish state
        gazebo_msgs::ModelState stateUpdate;

        // Uncomment the math below to treat theta as a quaternion. Currently just using w == theta
        // stateUpdate.pose.orientation.x = 0.0;
        // stateUpdate.pose.orientation.y = 0.0;
        // stateUpdate.pose.orientation.z = 1.0;
        stateUpdate.pose.orientation.w = pendulum.m_state.theta;//std::cos(pendulum.m_state.theta/2.0);
        stateUpdate.twist.angular.z = pendulum.m_state.dTheta;
        statePublisher.publish(stateUpdate);

        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}