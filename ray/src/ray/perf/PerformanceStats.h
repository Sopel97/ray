#pragma once

#include <ray/utility/Util.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <intrin.h>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <typeinfo>
#include <vector>

namespace ray 
{
    struct Box3;
    struct Capsule;
    struct ClosedTriangleMeshFace;
    struct Cylinder;
    struct Disc3;
    struct HalfSphere;
    struct OrientedBox3;
    struct Plane;
    struct Sphere;
    struct Triangle3;

    namespace perf
    {
        static constexpr std::size_t cacheLineSize = std::hardware_destructive_interference_size;

        // provides same operations as atomic, but only exchange is atomic
        struct Count
        {
            void operator+=(std::uint64_t d)
            {
                m_value += d;
            }

            std::uint64_t load() const
            {
                return m_value;
            }

            std::uint64_t exchange(std::uint64_t d)
            {
                std::int64_t s, new_s;
                do {
                    s = load();
                    new_s = d;
                } while (_InterlockedCompareExchange64(reinterpret_cast<volatile std::int64_t*>(&m_value), new_s, s) != s);
                return s;
            }

        private:
            std::uint64_t m_value;
        };

        struct AtomicCount : std::atomic<std::uint64_t>
        {
            void operator+=(std::uint64_t d)
            {
                std::atomic<std::uint64_t>::fetch_add(d, std::memory_order_relaxed);
            }
        };

        struct AtomicDuration : std::atomic<std::chrono::nanoseconds>
        {
            using Base = std::atomic<std::chrono::nanoseconds>;

            void operator+=(std::chrono::nanoseconds c)
            {
                std::chrono::nanoseconds s = Base::load();
                std::chrono::nanoseconds new_s;
                do {
                    new_s = s + c;
                } while (!Base::compare_exchange_strong(s, new_s));
            }
        };

        template <typename ShapeT, bool IsAtomicV>
        struct ObjectRaycastStats;
        template <typename BvShapeT, bool IsAtomicV>
        struct BvRaycastStats;
        template <typename ShapeT, bool IsAtomicV>
        struct IntervalRaycastStats;
        template <typename ShapeT, bool IsAtomicV>
        struct DistRaycastStats;

        template <typename ShapeT>
        struct ObjectRaycastStats<ShapeT, true>
        {
            using NonAtomicType = ObjectRaycastStats<ShapeT, false>;

            AtomicCount all;
            AtomicCount hits;
        };

        template <typename BvShapeT>
        struct BvRaycastStats<BvShapeT, true>
        {
            using NonAtomicType = BvRaycastStats<BvShapeT, false>;

            AtomicCount all;
            AtomicCount hits;
        };

        template <typename ShapeT>
        struct IntervalRaycastStats<ShapeT, true>
        {
            using NonAtomicType = IntervalRaycastStats<ShapeT, false>;

            AtomicCount all;
            AtomicCount hits;
        };

        template <typename ShapeT>
        struct DistRaycastStats<ShapeT, true>
        {
            using NonAtomicType = DistRaycastStats<ShapeT, false>;

            AtomicCount all;
            AtomicCount hits;
        };

        template <typename ShapeT>
        struct ObjectRaycastStats<ShapeT, false>
        {
            using AtomicType = ObjectRaycastStats<ShapeT, true>;

            Count all;
            Count hits;
        };

        template <typename BvShapeT>
        struct BvRaycastStats<BvShapeT, false>
        {
            using AtomicType = BvRaycastStats<BvShapeT, true>;

            Count all;
            Count hits;
        };

        template <typename ShapeT>
        struct IntervalRaycastStats<ShapeT, false>
        {
            using AtomicType = IntervalRaycastStats<ShapeT, true>;

            Count all;
            Count hits;
        };

        template <typename ShapeT>
        struct DistRaycastStats<ShapeT, false>
        {
            using AtomicType = DistRaycastStats<ShapeT, true>;

            Count all;
            Count hits;
        };

        struct TraceStatsTotal
        {
            std::uint64_t all;
            std::uint64_t hits;
            std::uint64_t resolved;
        };

        struct TraceStats
        {
            AtomicCount all;
            AtomicCount hits;
            AtomicCount resolved;
        };

        struct TimeStats
        {
            AtomicDuration time;
        };

        template <bool IsAtomicV>
        using AllRaycastStatsTypes = std::tuple <
            ObjectRaycastStats<Box3, IsAtomicV>,
            ObjectRaycastStats<Capsule, IsAtomicV>,
            ObjectRaycastStats<ClosedTriangleMeshFace, IsAtomicV>,
            ObjectRaycastStats<Cylinder, IsAtomicV>,
            ObjectRaycastStats<Disc3, IsAtomicV>,
            ObjectRaycastStats<HalfSphere, IsAtomicV>,
            ObjectRaycastStats<OrientedBox3, IsAtomicV>,
            ObjectRaycastStats<Plane, IsAtomicV>,
            ObjectRaycastStats<Sphere, IsAtomicV>,
            ObjectRaycastStats<Triangle3, IsAtomicV>,

            BvRaycastStats<Box3, IsAtomicV>,

            IntervalRaycastStats<Box3, IsAtomicV>,
            IntervalRaycastStats<Capsule, IsAtomicV>,
            IntervalRaycastStats<Cylinder, IsAtomicV>,
            IntervalRaycastStats<OrientedBox3, IsAtomicV>,
            IntervalRaycastStats<Sphere, IsAtomicV>,

            DistRaycastStats<Disc3, IsAtomicV>,
            DistRaycastStats<HalfSphere, IsAtomicV>
        >;

        struct alignas(cacheLineSize) ThreadLocalPerformanceStats;

        struct alignas(cacheLineSize) AtomicPerformanceStats
        {
            constexpr static int maxDepth = 16;

            friend struct ThreadLocalPerformanceStats;

            AtomicPerformanceStats() = default;

            AtomicPerformanceStats(const AtomicPerformanceStats&) = delete;
            AtomicPerformanceStats(AtomicPerformanceStats&&) = delete;
            AtomicPerformanceStats& operator=(const AtomicPerformanceStats&) = delete;
            AtomicPerformanceStats& operator=(AtomicPerformanceStats&&) = delete;

            void collect()
            {
                std::lock_guard<std::mutex> lock(m_childrenMutex);
                for (ThreadLocalPerformanceStats* child : m_children)
                {
                    collect(*child);
                }
            }

            void addTrace(int depth, std::uint64_t count = 1)
            {
                m_tracesByDepth[depth].all += count;
            }

            void addTraceHit(int depth, std::uint64_t count = 1)
            {
                m_tracesByDepth[depth].hits += count;
            }

            void addTraceResolved(int depth, std::uint64_t count = 1)
            {
                m_tracesByDepth[depth].resolved += count;
            }

            template <typename ShapeT>
            void addObjectRaycast(std::uint64_t count = 1)
            {
                objectRaycasts<ShapeT>().all += count;
            }

            template <typename ShapeT>
            void addObjectRaycastHit(std::uint64_t count = 1)
            {
                objectRaycasts<ShapeT>().hits += count;
            }

            template <typename BvShapeT>
            void addBvRaycast(std::uint64_t count = 1)
            {
                bvRaycasts<BvShapeT>().all += count;
            }

            template <typename BvShapeT>
            void addBvRaycastHit(std::uint64_t count = 1)
            {
                bvRaycasts<BvShapeT>().hits += count;
            }

            template <typename ShapeT>
            void addIntervalRaycast(std::uint64_t count = 1)
            {
                intervalRaycasts<ShapeT>().all += count;
            }

            template <typename ShapeT>
            void addIntervalRaycastHit(std::uint64_t count = 1)
            {
                intervalRaycasts<ShapeT>().hits += count;
            }

            template <typename ShapeT>
            void addDistRaycast(std::uint64_t count = 1)
            {
                distRaycasts<ShapeT>().all += count;
            }

            template <typename ShapeT>
            void addDistRaycastHit(std::uint64_t count = 1)
            {
                distRaycasts<ShapeT>().hits += count;
            }

            void addTraceTime(std::chrono::nanoseconds dur)
            {
                m_traceDuration.time += dur;
            }

            void addConstructionTime(std::chrono::nanoseconds dur)
            {
                m_constructionDuration.time += dur;
            }

            std::string summary() const
            {
                std::string out;

                auto entry3 = [&](std::uint64_t r, std::uint64_t h, std::uint64_t a) {
                    auto pct1 = (static_cast<double>(r) / static_cast<double>(h)) * 100.0;
                    auto pct2 = (static_cast<double>(h) / static_cast<double>(a)) * 100.0;
                    return std::to_string(r) + " [" + std::to_string(pct1) + "%] " + std::to_string(h) + " [" + std::to_string(pct2) + "%] " + std::to_string(a);
                };

                auto entry2 = [&](std::uint64_t a, std::uint64_t b) {
                    auto pct = (static_cast<double>(a) / static_cast<double>(b)) * 100.0;
                    return std::to_string(a) + " [" + std::to_string(pct) + "%] " + std::to_string(b);
                };

                auto constructionTimeSeconds = static_cast<double>(m_constructionDuration.time.load().count()) / 1e9;
                auto traceTimeSeconds = static_cast<double>(m_traceDuration.time.load().count()) / 1e9;
                TraceStatsTotal totalTraces = total(m_tracesByDepth);
                out += "Scene constrution time: " + std::to_string(constructionTimeSeconds) + "s\n";
                out += "Trace time: " + std::to_string(traceTimeSeconds) + "s\n";
                out += "Total resolved/hits/rays: " + entry3(totalTraces.resolved, totalTraces.hits, totalTraces.all) + "\n";
                for (int i = 0; i < m_tracesByDepth.size(); ++i)
                {
                    if (m_tracesByDepth[i].all.load() == 0) continue;
                    out += "  hits/rays at depth " + std::to_string(i) + ": " + entry3(m_tracesByDepth[i].resolved.load(), m_tracesByDepth[i].hits.load(), m_tracesByDepth[i].all.load()) + "\n";
                }
                out += "Rays/s: " + std::to_string(totalTraces.all / traceTimeSeconds) + "\n";

                for_each(m_raycasts, [&](const auto& c) {
                    using T = remove_cvref_t<decltype(c)>;
                    out += std::string(typeid(T).name()) + "\n";
                    out += "   hits/all: " + entry2(c.hits.load(), c.all.load()) + "\n";
                    });

                return out;
            }

            template <typename ShapeT>
            decltype(auto) objectRaycasts() const
            {
                return std::get<ObjectRaycastStats<ShapeT, true>>(m_raycasts);
            }

            template <typename BvShapeT>
            decltype(auto) bvRaycasts() const
            {
                return std::get<BvRaycastStats<BvShapeT, true>>(m_raycasts);
            }

            template <typename ShapeT>
            decltype(auto) intervalRaycasts() const
            {
                return std::get<IntervalRaycastStats<ShapeT, true>>(m_raycasts);
            }

            template <typename ShapeT>
            decltype(auto) distRaycasts() const
            {
                return std::get<DistRaycastStats<ShapeT, true>>(m_raycasts);
            }

        private:
            std::array<TraceStats, maxDepth + 1> m_tracesByDepth;
            AllRaycastStatsTypes<true> m_raycasts;
            TimeStats m_traceDuration;
            TimeStats m_constructionDuration;
            std::vector<ThreadLocalPerformanceStats*> m_children;
            std::mutex m_childrenMutex;

            template <typename ShapeT>
            decltype(auto) objectRaycasts()
            {
                return std::get<ObjectRaycastStats<ShapeT, true>>(m_raycasts);
            }

            template <typename BvShapeT>
            decltype(auto) bvRaycasts()
            {
                return std::get<BvRaycastStats<BvShapeT, true>>(m_raycasts);
            }

            template <typename ShapeT>
            decltype(auto) intervalRaycasts()
            {
                return std::get<IntervalRaycastStats<ShapeT, true>>(m_raycasts);
            }

            template <typename ShapeT>
            decltype(auto) distRaycasts()
            {
                return std::get<DistRaycastStats<ShapeT, true>>(m_raycasts);
            }


            TraceStatsTotal total(const std::array<TraceStats, maxDepth + 1>& stats) const
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


            void registerChild(ThreadLocalPerformanceStats& child)
            {
                std::lock_guard<std::mutex> lock(m_childrenMutex);
                m_children.emplace_back(&child);
            }

            void collectRemove(ThreadLocalPerformanceStats& perf)
            {
                std::lock_guard<std::mutex> lock(m_childrenMutex);
                collect(perf);
                auto it = std::find(m_children.begin(), m_children.end(), &perf);
                if (it != m_children.end())
                {
                    m_children.erase(it);
                }
            }

            void collect(ThreadLocalPerformanceStats& perf);
        };

        struct alignas(cacheLineSize) ThreadLocalPerformanceStats
        {
            constexpr static int maxDepth = 16;

            friend struct AtomicPerformanceStats;

            explicit ThreadLocalPerformanceStats(AtomicPerformanceStats* parent = nullptr) :
                m_tracesByDepth{},
                m_raycasts{},
                m_traceDuration{},
                m_constructionDuration{},
                m_parent(parent)
            {
                if (m_parent)
                {
                    m_parent->registerChild(*this);
                }
            }

            ThreadLocalPerformanceStats(const ThreadLocalPerformanceStats&) = delete;
            ThreadLocalPerformanceStats(ThreadLocalPerformanceStats&&) = delete;
            ThreadLocalPerformanceStats& operator=(const ThreadLocalPerformanceStats&) = delete;
            ThreadLocalPerformanceStats& operator=(ThreadLocalPerformanceStats&&) = delete;

            ~ThreadLocalPerformanceStats()
            {
                if (m_parent)
                {
                    m_parent->collectRemove(*this);
                }
            }

            void addTrace(int depth, std::uint64_t count = 1)
            {
                m_tracesByDepth[depth].all += count;
            }

            void addTraceHit(int depth, std::uint64_t count = 1)
            {
                m_tracesByDepth[depth].hits += count;
            }

            void addTraceResolved(int depth, std::uint64_t count = 1)
            {
                m_tracesByDepth[depth].resolved += count;
            }

            template <typename ShapeT>
            void addObjectRaycast(std::uint64_t count = 1)
            {
                objectRaycasts<ShapeT>().all += count;
            }

            template <typename ShapeT>
            void addObjectRaycastHit(std::uint64_t count = 1)
            {
                objectRaycasts<ShapeT>().hits += count;
            }

            template <typename BvShapeT>
            void addBvRaycast(std::uint64_t count = 1)
            {
                bvRaycasts<BvShapeT>().all += count;
            }

            template <typename BvShapeT>
            void addBvRaycastHit(std::uint64_t count = 1)
            {
                bvRaycasts<BvShapeT>().hits += count;
            }

            template <typename ShapeT>
            void addIntervalRaycast(std::uint64_t count = 1)
            {
                intervalRaycasts<ShapeT>().all += count;
            }

            template <typename ShapeT>
            void addIntervalRaycastHit(std::uint64_t count = 1)
            {
                intervalRaycasts<ShapeT>().hits += count;
            }

            template <typename ShapeT>
            void addDistRaycast(std::uint64_t count = 1)
            {
                distRaycasts<ShapeT>().all += count;
            }

            template <typename ShapeT>
            void addDistRaycastHit(std::uint64_t count = 1)
            {
                distRaycasts<ShapeT>().hits += count;
            }

            void addTraceTime(std::chrono::nanoseconds dur)
            {
                m_traceDuration.time += dur;
            }

            void addConstructionTime(std::chrono::nanoseconds dur)
            {
                m_constructionDuration.time += dur;
            }

            template <typename ShapeT>
            decltype(auto) objectRaycasts() const
            {
                return std::get<ObjectRaycastStats<ShapeT, false>>(m_raycasts);
            }

            template <typename BvShapeT>
            decltype(auto) bvRaycasts() const
            {
                return std::get<BvRaycastStats<BvShapeT, false>>(m_raycasts);
            }

            template <typename ShapeT>
            decltype(auto) intervalRaycasts() const
            {
                return std::get<IntervalRaycastStats<ShapeT, false>>(m_raycasts);
            }

            template <typename ShapeT>
            decltype(auto) distRaycasts() const
            {
                return std::get<DistRaycastStats<ShapeT, false>>(m_raycasts);
            }

        private:
            std::array<TraceStats, maxDepth + 1> m_tracesByDepth;
            AllRaycastStatsTypes<false> m_raycasts;
            TimeStats m_traceDuration;
            TimeStats m_constructionDuration;
            AtomicPerformanceStats* m_parent;
            std::mutex m_childrenMutex;

            template <typename ShapeT>
            decltype(auto) objectRaycasts()
            {
                return std::get<ObjectRaycastStats<ShapeT, false>>(m_raycasts);
            }

            template <typename BvShapeT>
            decltype(auto) bvRaycasts()
            {
                return std::get<BvRaycastStats<BvShapeT, false>>(m_raycasts);
            }

            template <typename ShapeT>
            decltype(auto) intervalRaycasts()
            {
                return std::get<IntervalRaycastStats<ShapeT, false>>(m_raycasts);
            }

            template <typename ShapeT>
            decltype(auto) distRaycasts()
            {
                return std::get<DistRaycastStats<ShapeT, false>>(m_raycasts);
            }
        };

        void AtomicPerformanceStats::collect(ThreadLocalPerformanceStats& perf)
        {
            for (int i = 0; i <= maxDepth; ++i)
            {
                m_tracesByDepth[i].all += perf.m_tracesByDepth[i].all.exchange(0);
                m_tracesByDepth[i].hits += perf.m_tracesByDepth[i].hits.exchange(0);
                m_tracesByDepth[i].resolved += perf.m_tracesByDepth[i].resolved.exchange(0);
            }

            for_each(m_raycasts, [&perf](auto& c) {
                using T = typename remove_cvref_t<decltype(c)>::NonAtomicType;
                c.all += std::get<T>(perf.m_raycasts).all.exchange(0);
                c.hits += std::get<T>(perf.m_raycasts).hits.exchange(0);
                });

            m_traceDuration.time += perf.m_traceDuration.time.exchange(std::chrono::nanoseconds(0));
            m_constructionDuration.time += perf.m_constructionDuration.time.exchange(std::chrono::nanoseconds(0));
        }

        inline AtomicPerformanceStats gGlobalPerfStats;
        inline thread_local ThreadLocalPerformanceStats gThreadLocalPerfStats(&gGlobalPerfStats);
    }
}
