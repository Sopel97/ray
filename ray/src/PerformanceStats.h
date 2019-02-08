#pragma once

#include <atomic>
#include <chrono>
#include <string>
#include <vector>

namespace ray 
{
    namespace perf
    {
        struct PerformanceStats
        {
            constexpr static int maxDepth = 32;

            struct TraceStatsTotal
            {
                std::uint64_t all;
                std::uint64_t hits;
                std::uint64_t resolved;
            };

            struct TraceStats
            {
                void add(std::uint64_t c)
                {
                    all.fetch_add(c);
                }

                void addHit(std::uint64_t c)
                {
                    hits.fetch_add(c);
                }

                void addResolved(std::uint64_t c)
                {
                    resolved.fetch_add(c);
                }

                std::atomic<std::uint64_t> all;
                std::atomic<std::uint64_t> hits;
                std::atomic<std::uint64_t> resolved;
            };

            struct ObjectRaycastStats
            {
                void add(std::uint64_t c)
                {
                    all.fetch_add(c);
                }

                void addHit(std::uint64_t c)
                {
                    hits.fetch_add(c);
                }

                std::atomic<std::uint64_t> all;
                std::atomic<std::uint64_t> hits;
            };

            struct BvRaycastStats
            {
                void add(std::uint64_t c)
                {
                    all.fetch_add(c);
                }

                void addHit(std::uint64_t c)
                {
                    hits.fetch_add(c);
                }

                std::atomic<std::uint64_t> all;
                std::atomic<std::uint64_t> hits;
            };

            struct TimeStats
            {
                void add(std::chrono::nanoseconds c)
                {
                    std::chrono::nanoseconds s = time.load();
                    std::chrono::nanoseconds new_s;
                    do {
                        new_s = s + c;
                    } while (!time.compare_exchange_strong(s, new_s));
                }

                std::atomic<std::chrono::nanoseconds> time;
            };

            PerformanceStats() :
                m_tracesByDepth(maxDepth + 1),
                m_sphereRaycasts{},
                m_boxBvRaycasts{},
                m_traceDuration{},
                m_constructionDuration{}
            {
            }

            void addTrace(int depth, std::uint64_t count = 1)
            {
                m_tracesByDepth[depth].add(count);
            }

            void addTraceHit(int depth, std::uint64_t count = 1)
            {
                m_tracesByDepth[depth].addHit(count);
            }

            void addTraceResolved(int depth, std::uint64_t count = 1)
            {
                m_tracesByDepth[depth].addResolved(count);
            }

            void addSphereRaycast(std::uint64_t count = 1)
            {
                m_sphereRaycasts.add(count);
            }

            void addSphereRaycastHit(std::uint64_t count = 1)
            {
                m_sphereRaycasts.addHit(count);
            }

            void addBoxBvRaycast(std::uint64_t count = 1)
            {
                m_boxBvRaycasts.add(count);
            }

            void addBoxBvRaycastHit(std::uint64_t count = 1)
            {
                m_boxBvRaycasts.addHit(count);
            }

            void addTraceTime(std::chrono::nanoseconds dur)
            {
                m_traceDuration.add(dur);
            }

            void addConstructionTime(std::chrono::nanoseconds dur)
            {
                m_constructionDuration.add(dur);
            }

            TraceStatsTotal total(const std::vector<TraceStats>& stats) const
            {
                TraceStatsTotal tot{};

                for (const auto& s : stats)
                {
                    tot.all += s.all.load();
                    tot.hits += s.hits.load();
                    tot.resolved += s.resolved.load();
                }

                return tot;
            }
            
            std::string summary() const
            {
                std::string out;

                auto entry3 = [&](std::uint64_t a, std::uint64_t b, std::uint64_t r) {
                    auto pct = (static_cast<double>(a) / static_cast<double>(b)) * 100.0;
                    return std::to_string(a) + "/" + std::to_string(b) + " (" + std::to_string(pct) + "%); [" + std::to_string(r) + "]";
                };

                auto constructionTimeSeconds = static_cast<double>(m_constructionDuration.time.load().count()) / 1e9;
                auto traceTimeSeconds = static_cast<double>(m_traceDuration.time.load().count()) / 1e9;
                TraceStatsTotal totalTraces = total(m_tracesByDepth);
                out += "Scene constrution time: " + std::to_string(constructionTimeSeconds) + "s\n";
                out += "Trace time: " + std::to_string(traceTimeSeconds) + "s\n";
                out += "Total hits/rays: " + entry3(totalTraces.hits, totalTraces.all, totalTraces.resolved) + "\n";
                for (int i = 0; i < m_tracesByDepth.size(); ++i)
                {
                    out += "  hits/rays at depth " + std::to_string(i) + ": " + entry3(m_tracesByDepth[i].hits.load(), m_tracesByDepth[i].all.load(), m_tracesByDepth[i].resolved.load()) + "\n";
                }
                out += "Rays/s: " + std::to_string(totalTraces.all / traceTimeSeconds) + "\n";

                out += "Intersection test stats:\n";
                out += "Sphere tests:\n";
                out += "   All: " + std::to_string(m_sphereRaycasts.all.load()) + "\n";
                out += "  Hits: " + std::to_string(m_sphereRaycasts.hits.load()) + "\n";
                out += "Box BV tests:\n";
                out += "   All: " + std::to_string(m_boxBvRaycasts.all.load()) + "\n";
                out += "  Hits: " + std::to_string(m_boxBvRaycasts.hits.load()) + "\n";

                return out;
            }

        private:
            std::vector<TraceStats> m_tracesByDepth;
            ObjectRaycastStats m_sphereRaycasts;
            BvRaycastStats m_boxBvRaycasts;
            TimeStats m_traceDuration;
            TimeStats m_constructionDuration;
        };

        inline PerformanceStats gPerfStats;
    }
}
