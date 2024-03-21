#pragma once

#include <math/vector.h>

struct DifferentialCart
{
    struct Params
    {
        double maxWheelVel = 2;
        double axisLen = 0.2;
    } m_params;

    struct State
    {
        double vRight = 0;
        double vLeft = 0;
        math::Vec2d pos = {};
        double orient = 0;
    } m_state;

    struct Input
    {
        double dvRight = 0;
        double dvLeft = 0;
    };

    void step(double dt, const Input& action);

    void draw();
};