#pragma once

#include <ray/material/Color.h>

#include <ray/math/Ray.h>
#include <ray/math/Vec2.h>
#include <ray/math/Vec3.h>

#include <ray/utility/Array2.h>
#include <ray/utility/IterableNumber.h>

#include <ray/Camera.h>

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

            auto sample = [&](const Point2f& coords) {
                return traceFunc(vp.rayAt(coords));
            };

            Array2<ColorRGBf> samples(vp.widthPixels, vp.heightPixels);
            std::for_each_n(exec, IterableNumber(0), vp.heightPixels, [&](int yi) {
                for (int xi = 0; xi < vp.widthPixels; ++xi)
                {
                    samples(xi, yi) = sample(Point2f(static_cast<float>(xi), static_cast<float>(yi)));
                }
            });

            auto distance = [&](const ColorRGBf& lhs, const ColorRGBf& rhs) {
                return
                      std::abs(lhs.r - rhs.r)
                    + std::abs(lhs.g - rhs.g)
                    + std::abs(lhs.b - rhs.b);
            };

            auto isAliased = [&](const Point2i& pos) {
                if (distance(samples(pos.x, pos.y), samples(pos.x, pos.y + 1)) > m_threshold) return true;
                if (distance(samples(pos.x, pos.y), samples(pos.x, pos.y - 1)) > m_threshold) return true;
                if (distance(samples(pos.x, pos.y), samples(pos.x + 1, pos.y)) > m_threshold) return true;
                if (distance(samples(pos.x, pos.y), samples(pos.x - 1, pos.y)) > m_threshold) return true;

                return false;
            };

            std::for_each_n(exec, IterableNumber(0), vp.heightPixels, [&](int yi) {
                for (int xi = 0; xi < vp.widthPixels; ++xi)
                {
                    const Point2i xyi(xi, yi);
                    if (xi == 0 || yi == 0 || xi == vp.widthPixels - 1 || yi == vp.heightPixels - 1)
                    {
                        storeFunc(xyi, samples(xi, yi));
                    }
                    else if (isAliased(xyi))
                    {
                        const Point2f xyf(static_cast<float>(xi), static_cast<float>(yi));
                        ColorRGBf color{};
                        int numSamples = 0;
                        m_multisampler.forEachSampleOffset(xyi, [&](const Vec2f& offset, float contribution) {
                            color += sample(xyf + offset) * contribution;
                            ++numSamples;
                        });
                        color = 
                            (color * static_cast<float>(numSamples) + samples(xi, yi)) 
                            / static_cast<float>(numSamples + 1);
                        storeFunc(xyi, color);
                    }
                    else
                    {
                        storeFunc(xyi, samples(xi, yi));
                    }
                }
            });
        }

    private:
        float m_threshold;
        MultisamplerT m_multisampler;
    };
}
