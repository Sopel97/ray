#pragma once

#include "Camera.h"
#include "Color.h"
#include "IterableNumber.h"
#include "Ray.h"
#include "Sampler.h"
#include "Vec3.h"

#include <algorithm>
#include <cmath>
#include <execution>

namespace ray
{
    struct UniformGridMultisampler : Sampler
    {
        UniformGridMultisampler(int order) :
            m_order(order)
        {
        }

        template <typename FuncT>
        void forEachSampleOffset(FuncT func) const
        {
            const float subpixelSize = 1.0f / m_order;
            const float add = subpixelSize * 0.5f - 0.5f; // to center it on (0.0, 0.0)
            for (int xxi = 0; xxi < m_order; ++xxi)
            {
                for (int yyi = 0; yyi < m_order; ++yyi)
                {
                    const float dx = static_cast<float>(xxi) * subpixelSize + add;
                    const float dy = static_cast<float>(yyi) * subpixelSize + add;
                    func(dx, dy);
                }
            }
        }

        template <typename TraceFuncT, typename StoreFuncT, typename ExecT = std::execution::sequenced_policy>
        void forEachSample(const Camera& camera, TraceFuncT traceFunc, StoreFuncT storeFunc, ExecT exec = ExecT{}) const
        {
            const Viewport vp = camera.viewport();

            auto sample = [&](float x, float y) {
                return traceFunc(vp.rayAt(x, y));
            };

            const float singleSampleContribution = 1.0f / (m_order * m_order);

            std::for_each_n(exec, IterableNumber(0), vp.heightPixels, [&](int yi) {
                for (int xi = 0; xi < vp.widthPixels; ++xi)
                {
                    ColorRGBf totalColor{};
                    forEachSampleOffset([&](float dx, float dy) {
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
        int m_order;
    };
}
