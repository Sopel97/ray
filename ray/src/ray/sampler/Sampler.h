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

namespace ray
{
    struct Sampler
    {
        template <typename FuncT>
        void forEachSampleOffset(const Point2i& pixel, FuncT func) const
        {
            func(Point2f(0.0f, 0.0f));
        }

        template <typename TraceFuncT, typename StoreFuncT, typename ExecT = std::execution::sequenced_policy>
        void forEachSample(const Camera& camera, TraceFuncT traceFunc, StoreFuncT storeFunc, ExecT exec = ExecT{}) const
        {
            const Viewport vp = camera.viewport();

            auto sample = [&](const Point2f& coords) {
                return traceFunc(vp.rayAt(coords));
            };

            auto range = IntRange2(Point2i(vp.widthPixels, vp.heightPixels));
            std::for_each(exec, range.begin(), range.end(), [&](const Point2i& xyi) {
                auto[xi, yi] = xyi;
                const Point2f xyf(static_cast<float>(xi), static_cast<float>(yi));
                storeFunc(xyi, sample(xyf));
            });
        }
    };
}
