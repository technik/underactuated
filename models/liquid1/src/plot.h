#pragma once
#include <math/linear.h>
#include <math/vector.h>

#include "implot.h"

// Auxiliary drawing
template<int numSegments>
static void plotCircle(const char* name, float x0, float y0, float radius)
{
    static_assert(numSegments > 1);
    float x[numSegments + 1];
    float y[numSegments + 1];
    for (int i = 0; i < numSegments + 1; ++i)
    {
        auto theta = i * math::TwoPi / numSegments;
        x[i] = radius * cos(theta) + x0;
        y[i] = radius * sin(theta) + y0;
    }
    ImPlot::PlotLine(name, x, y, numSegments + 1);
}

inline void plotLine(const char* name, const math::Vec2d& a, const math::Vec2d& b)
{
    float x[2] = { float(a.x()), float(b.x()) };
    float y[2] = { float(a.y()), float(b.y()) };

    ImPlot::PlotLine(name, x, y, 2);
}