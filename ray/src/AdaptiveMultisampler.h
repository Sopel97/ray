#pragma once

#include "Array2.h"
#include "Camera.h"
#include "Color.h"
#include "IterableNumber.h"
#include "Ray.h"
#include "Vec3.h"

#include <algorithm>
#include <cmath>
#include <execution>

namespace ray
{
    template <typename MultisamplerT>
    struct AdaptiveMultisampler
    {
        AdaptiveMultisampler(float threshold, MultisamplerT&& multisampler) :
            m_threshold(threshold),
            m_multisampler(std::move(multisampler))
        {
        }

        template <typename TraceFuncT, typename StoreFuncT, typename ExecT = std::execution::sequenced_policy>
        void forEachSample(const Camera& camera, TraceFuncT traceFunc, StoreFuncT storeFunc, ExecT exec = ExecT{}) const
        {
            const Viewport vp = camera.viewport();

            auto sample = [&](float x, float y) {
                return traceFunc(vp.rayAt(x, y));
            };

            Array2<ColorRGBf> samples(vp.widthPixels, vp.heightPixels);
            std::for_each_n(exec, IterableNumber(0), vp.heightPixels, [&](int yi) {
                for (int xi = 0; xi < vp.widthPixels; ++xi)
                {
                    samples(xi, yi) = sample(static_cast<float>(xi), static_cast<float>(yi));
                }
            });

            auto distance = [&](const ColorRGBf& lhs, const ColorRGBf& rhs) {
                return
                      std::abs(lhs.r - rhs.r)
                    + std::abs(lhs.g - rhs.g)
                    + std::abs(lhs.b - rhs.b);
            };

            auto isAliased = [&](int x, int y) {
                if (distance(samples(x, y), samples(x, y + 1)) > m_threshold) return true;
                if (distance(samples(x, y), samples(x, y - 1)) > m_threshold) return true;
                if (distance(samples(x, y), samples(x + 1, y)) > m_threshold) return true;
                if (distance(samples(x, y), samples(x - 1, y)) > m_threshold) return true;

                return false;
            };

            std::for_each_n(exec, IterableNumber(0), vp.heightPixels, [&](int yi) {
                for (int xi = 0; xi < vp.widthPixels; ++xi)
                {
                    if (xi == 0 || yi == 0 || xi == vp.widthPixels - 1 || yi == vp.heightPixels - 1)
                    {
                        storeFunc(xi, yi, samples(xi, yi));
                    }
                    else if (isAliased(xi, yi))
                    {
                        ColorRGBf color{};
                        int numSamples = 0;
                        m_multisampler.forEachSampleOffset(xi, yi, [&](float dx, float dy, float contribution) {
                            color += sample(static_cast<float>(xi) + dx, static_cast<float>(yi) + dy) * contribution;
                            ++numSamples;
                        });
                        color = 
                            (color * static_cast<float>(numSamples) + samples(xi, yi)) 
                            / static_cast<float>(numSamples + 1);
                        storeFunc(xi, yi, color);
                    }
                    else
                    {
                        storeFunc(xi, yi, samples(xi, yi));
                    }
                }
            });
        }

    private:
        float m_threshold;
        MultisamplerT m_multisampler;
    };
}
