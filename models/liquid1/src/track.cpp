#include "track.h"

#include <math/linear.h>
#include "imgui.h"
#include "implot.h"
#include "plot.h"

using namespace math;

void LinearTrack::Init()
{
    // vertices:
    constexpr auto numVtx = 5;

    // Init track corners
    Vec2d v[numVtx];
    for (size_t i = 0; i < numVtx; ++i)
    {
        v[i] = Vec2d(
            radius * cos(-math::TwoPi * i / numVtx),
            radius * sin(-math::TwoPi * i / numVtx));
    }

    // Init track sectors
    m_sectors.resize(numVtx - 1);
    for (size_t i = 0; i < numVtx - 1; ++i)
    {
        m_sectors[i] = Segment(v[i], v[i + 1], width);
    }

    // Cache drawing lines
    m_insideLineX.resize(numVtx);
    m_insideLineY.resize(numVtx);
    m_outsideLineX.resize(numVtx);
    m_outsideLineY.resize(numVtx);
    for (size_t i = 0; i < numVtx; ++i)
    {
        Vec2d n;
        if (i == 0)
        {
            n = 0.5 * width * m_sectors[0].m_normal;
        }
        else if (i == numVtx - 1)
        {
            n = 0.5 * width * m_sectors.back().m_normal;
        }
        else
        {
            n = 0.5 * width * normalize(m_sectors[i - 1].m_normal + m_sectors[i].m_normal);
        }
        auto inside = v[i] - n;
        auto outside = v[i] + n;
        m_insideLineX[i] = inside.x();
        m_insideLineY[i] = inside.y();
        m_outsideLineX[i] = outside.x();
        m_outsideLineY[i] = outside.y();
    }

    // cache sector cummulative length
    float accumPath = 0;
    m_sectorCumLen.resize(m_sectors.size());
    for (size_t i = 0; i < numVtx - 1; ++i)
    {
        m_sectorCumLen[i] = accumPath;
        accumPath += m_sectors[i].m_len;
    }
}

void LinearTrack::BeginPlot()
{
    // Draw simulation state
    if (ImGui::Begin("SimTrack"))
    {
        float size = radius + width;
        m_plotOpen = ImPlot::BeginPlot("Track", ImVec2(-1, -1), ImPlotFlags_Equal);
        if (m_plotOpen)
        {
            // Set up rigid axes
            ImPlot::SetupAxis(ImAxis_X1, NULL, ImPlotAxisFlags_AuxDefault);
            ImPlot::SetupAxisLimits(ImAxis_X1, -size, size, ImGuiCond_Always);
            ImPlot::SetupAxis(ImAxis_Y1, NULL, ImPlotAxisFlags_AuxDefault);

            // Draw track limits
            ImPlot::PlotLine("inside", m_insideLineX.data(), m_insideLineY.data(), m_insideLineX.size());
            ImPlot::PlotLine("outside", m_outsideLineX.data(), m_outsideLineY.data(), m_outsideLineX.size());
        }
    }
}

void LinearTrack::EndPlot()
{
    if (m_plotOpen)
    {
        ImPlot::EndPlot();
    }
    ImGui::End();
}

float LinearTrack::Segment::closestPointDistance(const Vec2d& pos, Vec2d& outP) const
{
    auto t = dot(pos - m_start, m_dir);
    outP = (t <= 0) ? m_start : ((t >= m_len) ? m_end : (m_start + t * m_dir));
    return (outP - pos).norm();
}

LinearTrack::Segment::ProjectedState LinearTrack::Segment::project(const Vec2d& pos, const Vec2d& dir) const
{
    ProjectedState result;
    auto relPos = pos - m_start;
    result.x = dot(relPos, m_dir);
    result.y = dot(relPos, m_normal);
    result.cosT = dot(dir, m_dir);
    result.sinT = dot(dir, m_normal);

    return result;
}

Vec2d LinearTrack::Segment::project(const Vec2d& pos) const
{
    Vec2d result;
    auto relPos = pos - m_start;
    result.x() = dot(relPos, m_dir);
    result.y() = dot(relPos, m_normal);

    return result;
}

LinearTrack::Segment::ProjectedState LinearTrack::project(const math::Vec2d& pos, const math::Vec2d& dir) const
{
    auto s = closestSegment(pos);
    return m_sectors[s].project(pos, dir);
}

math::Vec2d LinearTrack::project(const math::Vec2d& pos) const
{
    auto s = closestSegment(pos);
    return m_sectors[s].project(pos);
}

bool LinearTrack::Segment::isInside(const Vec2d& pos) const
{
    auto relPos = pos - m_start;
    auto x = dot(relPos, m_dir);
    auto y = dot(relPos, m_normal);
    return (abs(y) <= 0.5 * m_width) && (x >= 0) && (x <= m_len);
}

size_t LinearTrack::closestSegment(const Vec2d& pos) const
{
    Vec2d cp;
    float minDistance = std::numeric_limits<float>::max();
    size_t segmentId = -1;
    for (size_t i = 0; i < m_sectors.size(); ++i)
    {
        float t = m_sectors[i].closestPointDistance(pos, cp);
        if (t < minDistance)
        {
            minDistance = t;
            segmentId = i;
        }
    }
    return segmentId;
};

Vec2d LinearTrack::sampleStartPos(SquirrelRng& rng) const
{
    auto& sector = m_sectors.front();
    auto y = (rng.uniform() - 0.5f) * width;
    return sector.m_start + y * sector.m_normal;
}

Vec2d LinearTrack::samplePosition(SquirrelRng& rng) const
{
    // Last sector is considered the goal
    int sectorNdx = rng.rand() % (m_sectors.size() - 1);
    auto& sector = m_sectors[sectorNdx];
    auto x = rng.uniform() * sector.m_len;
    auto y = (rng.uniform() - 0.5f) * width;
    return sector.m_start + x * sector.m_dir + y * sector.m_normal;
}