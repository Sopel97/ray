#pragma once

#include <atomic>
#include <cstdint>
#include <chrono>
#include <string>
#include <vector>

namespace ray
{
    struct RaytracerStats
    {
        RaytracerStats(int maxDepth) :
            m_numRaysByDepth(maxDepth + 1),
            m_numRayHitsByDepth(maxDepth + 1)
        {

        }

        std::uint64_t numRays(int depth) const
        {
            return m_numRaysByDepth[depth].load();
        }

        std::uint64_t numHits(int depth) const
        {
            return m_numRayHitsByDepth[depth].load();
        }

        std::uint64_t totalNumRays() const
        {
            std::uint64_t total = 0;
            for (const auto& v : m_numRaysByDepth)
            {
                total += v.load();
            }
            return total;
        }

        std::uint64_t totalNumHits() const
        {
            std::uint64_t total = 0;
            for (const auto& v : m_numRayHitsByDepth)
            {
                total += v.load();
            }
            return total;
        }

        void addRay(int depth)
        {
            ++m_numRaysByDepth[depth];
        }

        void addHit(int depth)
        {
            ++m_numRayHitsByDepth[depth];
        }

        void addTime(std::chrono::nanoseconds elapsed)
        {
            std::chrono::nanoseconds s = m_duration.load();
            std::chrono::nanoseconds new_s;
            do {
                new_s = s;
                new_s += elapsed; // whatever modifications you want
            } while (!m_duration.compare_exchange_strong(s, new_s));
        }

        std::chrono::nanoseconds timeElapsed() const
        {
            return m_duration.load();
        }

        std::string summary() const
        {
            std::string out;

            auto ratio = [&](std::uint64_t a, std::uint64_t b) {
                auto pct = (static_cast<double>(a) / static_cast<double>(b)) * 100.0;
                return std::to_string(a) + "/" + std::to_string(b) + " (" + std::to_string(pct) + "%)";
            };

            auto timeSeconds = static_cast<double>(timeElapsed().count()) / 1e9;
            out += "Time elapsed: " + std::to_string(timeSeconds) + "s\n";
            out += "Total hits/rays: " + ratio(totalNumHits(), totalNumRays()) + "\n";
            for (int i = 0; i <= m_numRaysByDepth.size(); ++i)
            {
                out += "  hits/rays at depth " + std::to_string(i) + ": " + ratio(numHits(i), numRays(i)) + "\n";
            }
            out += "Rays/s: " + std::to_string(totalNumRays() / timeSeconds) + "\n";

            return out;
        }

    private:
        std::vector<std::atomic<std::uint64_t>> m_numRaysByDepth;
        std::vector<std::atomic<std::uint64_t>> m_numRayHitsByDepth;
        std::atomic<std::chrono::nanoseconds> m_duration;
    };
}
