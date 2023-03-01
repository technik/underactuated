#pragma once

namespace math
{
    inline int squirrelNoise(int position, int seed = 0)
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

    struct LinearCongruentalGenerator
    {
        int rand()
        {
            return squirrelNoise(++m_state);
        }

        float uniform()
        {
            auto i = rand();
            return float(i & ((1 << 24) - 1)) / (1 << 24);
        }

        int m_state = 0;
    };
}