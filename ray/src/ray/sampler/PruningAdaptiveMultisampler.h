#pragma once

#include <ray/material/Color.h>

#include <ray/math/Ray.h>
#include <ray/math/Vec2.h>
#include <ray/math/Vec3.h>

#include <ray/utility/Array2.h>
#include <ray/utility/IntRange2.h>

#include <ray/Camera.h>

#include <algorithm>
#include <cmath>
#include <execution>

namespace ray
{
    template <typename MultisamplerT>
    struct PruningAdaptiveMultisampler
    {
        PruningAdaptiveMultisampler(float threshold, MultisamplerT&& multisampler) :
            m_threshold(threshold),
            m_multisampler(std::move(multisampler))
        {
        }

        template <typename TraceFuncT, typename StoreFuncT, typename ExecT = std::execution::sequenced_policy>
        void forEachSample(const Camera& camera, TraceFuncT traceFunc, StoreFuncT storeFunc, ExecT exec = ExecT{}) const
        {
            const Viewport vp = camera.viewport();

            auto sample = [&](const Point2f& coords) {
                return traceFunc(vp.rayAt(coords));
            };

            Array2<ColorRGBf> samples(vp.widthPixels, vp.heightPixels);
            auto range = IntRange2(Point2i(vp.widthPixels, vp.heightPixels));
            std::for_each(exec, range.begin(), range.end(), [&](const Point2i& xyi) {
                auto[xi, yi] = xyi;
                samples(xi, yi) = sample(Point2f(static_cast<float>(xi), static_cast<float>(yi)));
            });

            auto distance = [&](const ColorRGBf& lhs, const ColorRGBf& rhs) {
                return
                    std::abs(lhs.r - rhs.r)
                    + std::abs(lhs.g - rhs.g)
                    + std::abs(lhs.b - rhs.b);
            };

            auto sampleInterpolate = [&](const Point2i& xyi, const Vec2f& offset) {
                int xmin = xyi.x - 1;
                int ymin = xyi.y - 1;
                float tx = offset.x + 1.0f;
                float ty = offset.y + 1.0f;
                if (offset.x > 1.0f)
                {
                    xmin += 1;
                    tx -= 1.0f;
                }
                if (offset.y > 1.0f)
                {
                    ymin += 1;
                    ty -= 1.0f;
                }

                const ColorRGBf interpolated =
                    samples(xmin, ymin) * ((1.0f - tx) * (1.0f - ty))
                    + samples(xmin + 1, ymin) * ((tx) * (1.0f - ty))
                    + samples(xmin, ymin + 1) * ((1.0f - tx) * (ty))
                    + samples(xmin + 1, ymin + 1) * ((tx) * (ty));

                return interpolated;
            };

            auto sampleOrInterpolate = [&](const Point2i& xyi, const Vec2f& offset) {
                const ColorRGBf interpolated = sampleInterpolate(xyi, offset);

                if (distance(interpolated, samples(xyi.x, xyi.y)) > m_threshold)
                {
                    const Point2f xyf(static_cast<float>(xyi.x), static_cast<float>(xyi.y));
                    return sample(xyf + offset);
                }
                else
                {
                    return interpolated;
                }
            };

            std::for_each(exec, range.begin(), range.end(), [&](const Point2i& xyi) {
                auto[xi, yi] = xyi;
                if (xi == 0 || yi == 0 || xi == vp.widthPixels - 1 || yi == vp.heightPixels - 1)
                {
                    storeFunc(xyi, samples(xi, yi));
                }
                else
                {
                    const Point2f xyf(static_cast<float>(xi), static_cast<float>(yi));
                    ColorRGBf color{};
                    int numSamples = 0;
                    m_multisampler.forEachSampleOffset(xyi, [&](const Vec2f& offset, float contribution) {
                        color += sampleOrInterpolate(xyi, offset) * contribution;
                        ++numSamples;
                    });
                    color =
                        (color * static_cast<float>(numSamples) + samples(xi, yi))
                        / static_cast<float>(numSamples + 1);
                    storeFunc(xyi, color);
                }
            });
        }

    private:
        float m_threshold;
        MultisamplerT m_multisampler;
    };
}
