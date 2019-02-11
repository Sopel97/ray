#pragma once

#include "Camera.h"
#include "Color.h"
#include "IterableNumber.h"
#include "Ray.h"
#include "Vec3.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <execution>
#include <random>
#include <vector>

namespace ray
{
    struct JitteredMultisampler
    {
        template <typename RngT = std::minstd_rand>
        JitteredMultisampler(int order, int numOffsets, float scale = 1.0f, RngT&& rng = RngT{}) :
            m_order(order),
            m_offsets(numOffsets)
        {
            const std::uniform_real_distribution<float> dOffset(-0.5f * scale, 0.5f * scale);
            std::generate(m_offsets.begin(), m_offsets.end(), [&]() {
                return dOffset(rng);
            });
        }

        int numSampleOffsets() const
        {
            return m_order * m_order;
        }

        template <typename FuncT>
        void forEachSampleOffset(int x, int y, FuncT func) const
        {
            const float subpixelSize = 1.0f / m_order;
            const float add = subpixelSize * 0.5f - 0.5f; // to center it on (0.0, 0.0)
            const float contribution = 1.0f / numSampleOffsets();
            for (int xxi = 0; xxi < m_order; ++xxi)
            {
                for (int yyi = 0; yyi < m_order; ++yyi)
                {
                    const float dx = (static_cast<float>(xxi) + chooseOffsetX(x, y, xxi, yyi)) * subpixelSize + add;
                    const float dy = (static_cast<float>(yyi) + chooseOffsetY(x, y, xxi, yyi)) * subpixelSize + add;
                    func(dx, dy, contribution);
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

            const float singleSampleContribution = 1.0f / numSampleOffsets();

            std::for_each_n(exec, IterableNumber(0), vp.heightPixels, [&](int yi) {
                for (int xi = 0; xi < vp.widthPixels; ++xi)
                {
                    ColorRGBf totalColor{};
                    forEachSampleOffset(xi, yi, [&](float dx, float dy) {
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
        std::vector<float> m_offsets;
        
        float chooseOffset(std::uint64_t idx) const
        {
            idx = (idx ^ (idx >> 30)) * 0xbf58476d1ce4e5b9ull;
            idx = (idx ^ (idx >> 27)) * 0x94d049bb133111ebull;
            idx = idx ^ (idx >> 31);
            return m_offsets[idx % m_offsets.size()];
        }

        float chooseOffsetX(std::uint64_t x, std::uint64_t y, std::uint64_t dx, std::uint64_t dy) const
        {
            const std::uint64_t xx = x * static_cast<std::uint64_t>(m_order) + dx;
            const std::uint64_t yy = y * static_cast<std::uint64_t>(m_order) + dy;
            return chooseOffset(((xx & 0xFFFFFFFFu) << 32u) | (yy & 0xFFFFFFFFu));
        }

        float chooseOffsetY(std::uint64_t x, std::uint64_t y, std::uint64_t dx, std::uint64_t dy) const
        {
            const std::uint64_t xx = x * static_cast<std::uint64_t>(m_order) + dx;
            const std::uint64_t yy = y * static_cast<std::uint64_t>(m_order) + dy;
            return chooseOffset(~(((yy & 0xFFFFFFFFu) << 32u) | (xx & 0xFFFFFFFFu)));
        }
    };
}
