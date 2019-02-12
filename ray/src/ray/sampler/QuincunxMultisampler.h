#pragma once

#include <ray/material/Color.h>

#include <ray/math/Ray.h>
#include <ray/math/Vec3.h>

#include <ray/utility/Array2.h>
#include <ray/utility/IterableNumber.h>

#include <ray/Camera.h>

#include <algorithm>
#include <cmath>
#include <execution>

namespace ray
{
    struct QuincunxMultisampler
    {
        int numSampleOffsets() const
        {
            return 5;
        }

        template <typename FuncT>
        void forEachSampleOffset(int x, int y, FuncT func) const
        {
            func(0.0f, 0.0f, 0.2f);
            func(0.5f, 0.5f, 0.2f);
            func(0.5f, -0.5f, 0.2f);
            func(-0.5f, 0.5f, 0.2f);
            func(-0.5f, -0.5f, 0.2f);
        }

        template <typename TraceFuncT, typename StoreFuncT, typename ExecT = std::execution::sequenced_policy>
        void forEachSample(const Camera& camera, TraceFuncT traceFunc, StoreFuncT storeFunc, ExecT exec = ExecT{}) const
        {
            const Viewport vp = camera.viewport();

            auto sample = [&](float x, float y) {
                return traceFunc(vp.rayAt(x, y));
            };

            Array2<ColorRGBf> supersamples(vp.widthPixels + 1, vp.heightPixels + 1);
            std::for_each_n(exec, IterableNumber(0), vp.heightPixels + 1, [&](int yi) {
                for (int xi = 0; xi < vp.widthPixels + 1; ++xi)
                {
                    supersamples(xi, yi) = sample(static_cast<float>(xi), static_cast<float>(yi));
                }
            });

            std::for_each_n(exec, IterableNumber(0), vp.heightPixels, [&](int yi) {
                for (int xi = 0; xi < vp.widthPixels; ++xi)
                {
                    const ColorRGBf total =
                        sample(static_cast<float>(xi), static_cast<float>(yi))
                        + supersamples(xi, yi)
                        + supersamples(xi, yi + 1)
                        + supersamples(xi + 1, yi)
                        + supersamples(xi + 1, yi + 1);

                    storeFunc(xi, yi, total * 0.2f);
                }
            });
        }
    };
}
