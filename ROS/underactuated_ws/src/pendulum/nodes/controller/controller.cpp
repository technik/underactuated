#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "gazebo_msgs/ModelState.h"
#include <cmath>
#include <mutex>

constexpr double PI = 3.14159265358979323;

struct Pendulum
{
    struct Params
    {
        double l1 = 1.0; // Bar lengths
        double m1 = 1.0; // Bar masses
        double b1 = 0.0; // Friction at the joints
        double I1 = 1.0; // Inertia tensors
        double MaxQ = 1.0; // Torque limit. 0 means unlimited torque
        double MaxPower = 0.0; // Power limit. 0 means unlimited power

        void refreshInertia()
        {
            // Inertia concentrated at the end
            I1 = m1 * l1 * l1 / 3;
        }
    };

    struct State
    {
        double theta = 0;
        double dTheta = 0;
    };

    struct Controller
    {
        virtual double control(const State& x, const Params& p, const double controlHz) = 0;
    };
};

struct EnergyPumpController : public Pendulum::Controller
{
    static constexpr auto g = 9.81;
    double energyGain = 1.0;
    double ePrevPD = 0.0;

    double control(const Pendulum::State& x, const Pendulum::Params& p, const double controlHz=100.0) override
    {
        // Target energy to stay still at the top:
        auto mgl = p.m1 * g * p.l1;
        auto Egoal = mgl;

        // Current energy
        auto T = 0.5 * p.m1 * p.l1 * p.l1 * x.dTheta * x.dTheta;
        auto V = p.m1 * g * p.l1 * -std::cos(x.theta);
        auto E = T + V;

        // PD Controller params
        constexpr auto linearRegion = 5.0*PI/180.0; // +- 5 deg at the top is linear enough.
        double Kp      = 15.0;
        double Kd      = 1.5;

        // Choose control method
        bool pumpEnergy = false;
        // Find the lowest angle we can stall at with MaxQ
        bool torqueLimited = p.MaxQ != 0 && p.MaxQ < p.m1* p.l1* g;
        if (torqueLimited)
        {
            auto minCos = p.MaxQ / mgl;
            if (-std::cos(x.theta) <= minCos)
                pumpEnergy = true;
        }


        if (pumpEnergy && std::abs(x.theta-PI) > linearRegion)
        {
            auto dE = Egoal - E;

            // Expected damping
            auto tDamp = -p.b1 * x.dTheta;
            auto Ugoal = -tDamp + energyGain * dE * x.dTheta;
            auto U = Ugoal;
            if(p.MaxQ > 0)
                U = std::min(p.MaxQ, std::max(-p.MaxQ, Ugoal));
            return U;
        }
        else if (std::abs(x.theta-PI) < linearRegion)
        {
            // PD Controller
            const auto e     = PI-x.theta;
            const auto dT = 1/controlHz;
            const auto e_dot = (e-ePrevPD)/dT;
            auto U = Kp*e + Kd*e_dot;

            ePrevPD = e;

            return std::min(p.MaxQ, std::max(-p.MaxQ, U));
        }
        else
        {
            // Switch to a position error controller
            // or maybe a bang-bang?
            return 0;
        }
    }
};

struct PendulumInstance
{
    Pendulum::Params     params;
    Pendulum::State      state;
    EnergyPumpController controller;
    std::mutex           state_mutex; 
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;

    PendulumInstance p;

    ros::Publisher  controller_pub = n.advertise<geometry_msgs::Wrench>("control_u", 1);
    ros::Subscriber controller_sub = n.subscribe<gazebo_msgs::ModelState>("pendulum_x", 1, 
        [&](const gazebo_msgs::ModelState::ConstPtr& pendulum_x) {
            const std::lock_guard<std::mutex> lock(p.state_mutex);
            // Uncomment the math below to treat theta as a quaternion. Currently just using w == theta
            // p.state.theta = 2.0*std::acos(pendulum_x->pose.orientation.w);
            p.state.theta =  pendulum_x->pose.orientation.w;
            p.state.dTheta = pendulum_x->twist.angular.z;
            // lock_guard mutex released when it goes out of scope
        }
    );

    const auto controlHz = 100.0;
    ros::Rate loop_rate(controlHz);
    
    geometry_msgs::Wrench control_u;
    Pendulum::State localState;
    while (ros::ok())
    {
        
        {
            const std::lock_guard<std::mutex> lock(p.state_mutex);
            localState = p.state;
        }

        auto u = p.controller.control(localState, p.params, controlHz);

        control_u.torque.z = u;
        controller_pub.publish(control_u);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}