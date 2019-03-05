#pragma once

#include <ray/material/Color.h>

#include <ray/math/Ray.h>
#include <ray/math/Vec2.h>
#include <ray/math/Vec3.h>

#include <ray/utility/IntRange2.h>

#include <ray/Camera.h>

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

        template <typename FuncT>
        void forEachSampleOffset(const Point2i& pixel, FuncT func) const
        {
            const float s = 1.0f / m_order;
            const Vec2f subpixelSize = Vec2f::broadcast(s);
            const Vec2f add = Vec2f::broadcast(s * 0.5f - 0.5f); // to center it on (0.0, 0.0)
            const float contribution = 1.0f / (m_order * m_order);
            for (int xxi = 0; xxi < m_order; ++xxi)
            {
                for (int yyi = 0; yyi < m_order; ++yyi)
                {
                    const Point2i xyi(xxi, yyi);
                    const Point2f xyf(static_cast<float>(xxi), static_cast<float>(yyi));
                    const Vec2f offset = Vec2f(xyf + chooseOffset(pixel, xyi)) * subpixelSize + add;
                    func(offset, contribution);
                }
            }
        }

        template <typename TraceFuncT, typename StoreFuncT, typename ExecT = std::execution::sequenced_policy>
        void forEachSample(const Camera& camera, TraceFuncT traceFunc, StoreFuncT storeFunc, ExecT exec = ExecT{}) const
        {
            const Viewport vp = camera.viewport();

            auto sample = [&](const Point2f& coords) {
                return traceFunc(vp.rayAt(coords));
            };

            const float singleSampleContribution = 1.0f / (m_order * m_order);

            auto range = IntRange2(Point2i(vp.widthPixels, vp.heightPixels));
            std::for_each(exec, range.begin(), range.end(), [&](const Point2i& xyi) {
                auto[xi, yi] = xyi;
                const Point2f xyf(static_cast<float>(xi), static_cast<float>(yi));
                ColorRGBf totalColor{};
                forEachSampleOffset(xyi, [&](const Vec2f& offset, float contribution) {
                    totalColor += sample(xyf + offset);
                });
                storeFunc(xyi, totalColor * singleSampleContribution);
            });
        }

    private:
        int m_order;
        std::vector<float> m_offsets;
        
        [[nodiscard]] float chooseOffset(std::uint64_t idx) const
        {
            idx = (idx ^ (idx >> 30)) * 0xbf58476d1ce4e5b9ull;
            idx = (idx ^ (idx >> 27)) * 0x94d049bb133111ebull;
            idx = idx ^ (idx >> 31);
            return m_offsets[idx % m_offsets.size()];
        }

        [[nodiscard]] float chooseOffsetX(std::uint64_t x, std::uint64_t y, std::uint64_t dx, std::uint64_t dy) const
        {
            const std::uint64_t xx = x * static_cast<std::uint64_t>(m_order) + dx;
            const std::uint64_t yy = y * static_cast<std::uint64_t>(m_order) + dy;
            return chooseOffset(((xx & 0xFFFFFFFFu) << 32u) | (yy & 0xFFFFFFFFu));
        }

        [[nodiscard]] float chooseOffsetY(std::uint64_t x, std::uint64_t y, std::uint64_t dx, std::uint64_t dy) const
        {
            const std::uint64_t xx = x * static_cast<std::uint64_t>(m_order) + dx;
            const std::uint64_t yy = y * static_cast<std::uint64_t>(m_order) + dy;
            return chooseOffset(~(((yy & 0xFFFFFFFFu) << 32u) | (xx & 0xFFFFFFFFu)));
        }

        [[nodiscard]] Vec2f chooseOffset(const Point2i& pixel, const Point2i& subpoint) const
        {
            return Vec2f(
                chooseOffsetX(pixel.x, pixel.y, subpoint.x, subpoint.y),
                chooseOffsetY(pixel.x, pixel.y, subpoint.x, subpoint.y)
            );
        }
    };
}
