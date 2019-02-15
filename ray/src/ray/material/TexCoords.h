#pragma once

namespace ray
{
    struct TexCoords
    {
        float u, v;
    };

    inline TexCoords operator*(const TexCoords& lhs, float rhs)
    {
        return { lhs.u * rhs, lhs.v * rhs };
    }

    inline TexCoords operator+(const TexCoords& lhs, const TexCoords& rhs)
    {
        return { lhs.u + rhs.u, lhs.v + rhs.v };
    }
}
