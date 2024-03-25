#pragma once

#include <vector>
#include <math/noise.h>
#include <math/vector.h>

struct LinearTrack
{
    double width = 1;
    float radius = 10;

    std::vector<float> m_insideLineX;
    std::vector<float> m_insideLineY;
    std::vector<float> m_outsideLineX;
    std::vector<float> m_outsideLineY;

    std::vector<float> m_sectorCumLen;

    void Init();

    bool m_plotOpen = false;

    void BeginPlot();

    void EndPlot();

    struct Segment
    {
        float m_width;
        Segment() = default;
        Segment(math::Vec2d s, math::Vec2d e, float width)
            : m_start(s), m_end(e)
        {
            m_dir = normalize(m_end - m_start);
            m_normal = math::Vec2d(-m_dir.y(), m_dir.x());
            m_len = dot(m_end - m_start, m_dir);
            m_width = width;
        }
        math::Vec2d m_start, m_end;
        math::Vec2d m_dir, m_normal;
        double m_len;

        float closestPointDistance(const math::Vec2d& pos, math::Vec2d& outP) const;

        struct ProjectedState
        {
            float x, y;
            float cosT, sinT;
        };

        ProjectedState project(const math::Vec2d& pos, const math::Vec2d& dir) const;

        math::Vec2d project(const math::Vec2d& pos) const;

        bool isInside(const math::Vec2d& pos) const;
    };

    Segment::ProjectedState project(const math::Vec2d& pos, const math::Vec2d& dir) const;

    math::Vec2d project(const math::Vec2d& pos) const;

    size_t closestSegment(const math::Vec2d& pos) const;

    math::Vec2d sampleStartPos(math::SquirrelRng& rng) const;

    math::Vec2d samplePosition(math::SquirrelRng& rng) const;

    float score(const math::Vec2d& pos) const
    {
        auto segId = closestSegment(pos);
        return m_sectorCumLen[segId] + m_sectors[segId].project(pos).x();
    }

    bool isValid(const math::Vec2d& x) const
    {
        for (auto& s : m_sectors)
        {
            if (s.isInside(x))
                return true;
        }
        return false;
    }

    bool isGoal(const math::Vec2d& x) const
    {
        return m_sectors.back().isInside(x);
    }

    std::vector<Segment> m_sectors;
};