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
    struct Sampler
    {
        template <typename FuncT>
        void forEachSampleOffset(FuncT func) const
        {
            func(0.0f, 0.0f);
        }

        template <typename TraceFuncT, typename StoreFuncT, typename ExecT = std::execution::sequenced_policy>
        void forEachSample(const Camera& camera, TraceFuncT traceFunc, StoreFuncT storeFunc, ExecT exec = ExecT{}) const
        {
            const Viewport vp = camera.viewport();

            auto sample = [&](float x, float y) {
                return traceFunc(vp.rayAt(x, y));
            };

            std::for_each_n(exec, IterableNumber(0), vp.heightPixels, [&](int yi) {
                for (int xi = 0; xi < vp.widthPixels; ++xi)
                {
                    storeFunc(xi, yi, sample(static_cast<float>(xi), static_cast<float>(yi)));
                }
            });
        }
    };
}
