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
    struct QuincunxMultisampler
    {
        template <typename FuncT>
        void forEachSampleOffset(int x, int y, FuncT func) const
        {
            func(0.0f, 0.0f);
            func(0.5f, 0.5f);
            func(0.5f, -0.5f);
            func(-0.5f, 0.5f);
            func(-0.5f, -0.5f);
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
