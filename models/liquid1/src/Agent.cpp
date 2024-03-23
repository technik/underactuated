#pragma once

#include <Agent.h>
#include <math/vector.h>

#include <math/linear.h>
#include <algorithm>

#include "plot.h"

using namespace std;

void DifferentialCart::State::randomize(float minX, float maxX, float trackWidth, math::SquirrelRng& rng)
{
    pos.x() = rng.uniform(minX, maxX);
    pos.y() = rng.uniform(-trackWidth * 0.5f, trackWidth * 0.5f);
    orient = (-0.5 + rng.uniform()) * math::Pi;
    vLeft = 0;
    vRight = 0;
}

void DifferentialCart::step(double dt, const Input& action)
{
    // Basic Euler integration
    double meanVel = 0.5 * (m_state.vRight + m_state.vLeft);
    double diffVel = (m_state.vRight - m_state.vLeft) / m_params.axisLen;
    double ct = cos(m_state.orient);
    double st = sin(m_state.orient);
    m_state.pos += (dt * meanVel) * math::Vec2d(ct, st);
    m_state.orient += dt * diffVel;
    if (m_state.orient < -math::TwoPi)
        m_state.orient += math::TwoPi;
    if (m_state.orient > math::TwoPi)
        m_state.orient -= math::TwoPi;

    // Apply the action
    //m_state.vRight = max(-m_params.maxWheelVel, min(m_params.maxWheelVel, m_state.vRight + action.dvRight));
    //m_state.vLeft = max(-m_params.maxWheelVel, min(m_params.maxWheelVel, m_state.vLeft + action.dvLeft));
    m_state.vRight = max(-m_params.maxWheelVel, min(m_params.maxWheelVel, action.dvRight));
    m_state.vLeft = max(-m_params.maxWheelVel, min(m_params.maxWheelVel, action.dvLeft));
}

void DifferentialCart::draw()
{
    float radius = float(m_params.axisLen * 0.5);
    plotCircle<20>("bot", m_state.pos.x(), m_state.pos.y(), radius);
    math::Vec2d lookAt = m_state.pos + radius * math::Vec2d(cos(m_state.orient), sin(m_state.orient));
    plotLine("botDir", m_state.pos, lookAt);
}