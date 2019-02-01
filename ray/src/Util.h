#pragma once

namespace ray
{
    float mix(float a, float b, float mix)    
    {
        return b * mix + a * (1.0f - mix);
    }
}
