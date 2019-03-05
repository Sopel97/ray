#pragma once

#include <ray/material/Color.h>

#include <ray/math/Ray.h>
#include <ray/math/Vec2.h>
#include <ray/math/Vec3.h>

#include <ray/utility/IntRange2.h>

#include <ray/Camera.h>

#include <algorithm>
#include <cmath>
#include <execution>
#include <vector>

namespace ray
{
    struct UniformGridMultisampler
    {
        UniformGridMultisampler(int order)
        {
            m_offsets.reserve(order * order);
            const float s = 1.0f / static_cast<float>(order);
            const Vec2f subpixelSize = Vec2f::broadcast(s);
            const Vec2f add = Vec2f::broadcast(s * 0.5f - 0.5f); // to center it on (0.0, 0.0)
            for (int xxi = 0; xxi < order; ++xxi)
            {
                for (int yyi = 0; yyi < order; ++yyi)
                {
                    const Vec2f xyf(static_cast<float>(xxi), static_cast<float>(yyi));
                    m_offsets.emplace_back(xyf * subpixelSize + add);
                }
            }
        }

        template <typename FuncT>
        void forEachSampleOffset(const Point2i pixel, FuncT func) const
        {
            const float contribution = 1.0f / static_cast<float>(m_offsets.size());
            for (Vec2f offset : m_offsets)
            {
                func(offset, contribution);
            }
        }

        template <typename TraceFuncT, typename StoreFuncT, typename ExecT = std::execution::sequenced_policy>
        void forEachSample(const Camera& camera, TraceFuncT traceFunc, StoreFuncT storeFunc, ExecT exec = ExecT{}) const
        {
            const Viewport vp = camera.viewport();

            auto sample = [&](const Point2f& coords) {
                return traceFunc(vp.rayAt(coords));
            };

            const float singleSampleContribution = 1.0f / static_cast<float>(m_offsets.size());

            auto range = IntRange2(Point2i(vp.widthPixels, vp.heightPixels));
            std::for_each(exec, range.begin(), range.end(), [&](const Point2i& xyi) {
                auto[xi, yi] = xyi;
                const Point2f xyf(static_cast<float>(xi), static_cast<float>(yi));
                ColorRGBf totalColor{};
                forEachSampleOffset(xyi, [&](const Vec2f& offset, float c) {
                    totalColor += sample(xyf + offset);
                });
                storeFunc(Point2i(xi, yi), totalColor * singleSampleContribution);
            });
        }

    private:
        std::vector<Vec2f> m_offsets;
    };
}
