#pragma once

#include <ray/material/Color.h>

#include <ray/math/Ray.h>
#include <ray/math/Vec2.h>
#include <ray/math/Vec3.h>

#include <ray/utility/IterableNumber.h>

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
            const float subpixelSize = 1.0f / static_cast<float>(order);
            const float add = subpixelSize * 0.5f - 0.5f; // to center it on (0.0, 0.0)
            for (int xxi = 0; xxi < order; ++xxi)
            {
                for (int yyi = 0; yyi < order; ++yyi)
                {
                    const float dx = static_cast<float>(xxi) * subpixelSize + add;
                    const float dy = static_cast<float>(yyi) * subpixelSize + add;
                    m_offsets.emplace_back(dx, dy);
                }
            }
        }

        template <typename FuncT>
        void forEachSampleOffset(int x, int y, FuncT func) const
        {
            const float contribution = 1.0f / static_cast<float>(m_offsets.size());
            for (Vec2f offset : m_offsets)
            {
                func(offset.x, offset.y, contribution);
            }
        }

        template <typename TraceFuncT, typename StoreFuncT, typename ExecT = std::execution::sequenced_policy>
        void forEachSample(const Camera& camera, TraceFuncT traceFunc, StoreFuncT storeFunc, ExecT exec = ExecT{}) const
        {
            const Viewport vp = camera.viewport();

            auto sample = [&](float x, float y) {
                return traceFunc(vp.rayAt(x, y));
            };

            const float singleSampleContribution = 1.0f / static_cast<float>(m_offsets.size());

            std::for_each_n(exec, IterableNumber(0), vp.heightPixels, [&](int yi) {
                for (int xi = 0; xi < vp.widthPixels; ++xi)
                {
                    ColorRGBf totalColor{};
                    forEachSampleOffset(xi, yi, [&](float dx, float dy, float c) {
                        totalColor += sample(
                            static_cast<float>(xi) + dx,
                            static_cast<float>(yi) + dy
                        );
                    });
                    storeFunc(xi, yi, totalColor * singleSampleContribution);
                }
            });
        }

    private:
        std::vector<Vec2f> m_offsets;
    };
}
