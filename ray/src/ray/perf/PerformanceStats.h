#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <tuple>
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
        struct alignas(cacheLineSize) AtomicCount : std::atomic<std::uint64_t>
        {
            void operator+=(std::uint64_t d)
            {
                std::atomic<std::uint64_t>::fetch_add(d, std::memory_order_relaxed);
            }
        };

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
                AtomicCount all;
                AtomicCount hits;
                AtomicCount resolved;
            };

            template <typename ShapeT>
            struct ObjectRaycastStats
            {
                AtomicCount all;
                AtomicCount hits;
            };

            template <typename BvShapeT>
            struct BvRaycastStats
            {
                AtomicCount all;
                AtomicCount hits;
            };

            template <typename ShapeT>
            struct IntervalRaycastStats
            {
                AtomicCount all;
                AtomicCount hits;
            };

            template <typename ShapeT>
            struct DistRaycastStats
            {
                AtomicCount all;
                AtomicCount hits;
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
                m_raycasts{},
                m_traceDuration{},
                m_constructionDuration{}
            {
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

                out += "Raycasts:\n";
                out += "Box3:\n";
                out += "   hits/all: " + entry2(objectRaycasts<Box3>().hits.load(), objectRaycasts<Box3>().all.load()) + "\n";
                out += "Capsule:\n";
                out += "   hits/all: " + entry2(objectRaycasts<Capsule>().hits.load(), objectRaycasts<Capsule>().all.load()) + "\n";
                out += "ClosedTriangleMeshFace:\n";
                out += "   hits/all: " + entry2(objectRaycasts<ClosedTriangleMeshFace>().hits.load(), objectRaycasts<ClosedTriangleMeshFace>().all.load()) + "\n";
                out += "Cylinder:\n";
                out += "   hits/all: " + entry2(objectRaycasts<Cylinder>().hits.load(), objectRaycasts<Cylinder>().all.load()) + "\n";
                out += "Disc3:\n";
                out += "   hits/all: " + entry2(objectRaycasts<Disc3>().hits.load(), objectRaycasts<Disc3>().all.load()) + "\n";
                out += "HalfSphere:\n";
                out += "   hits/all: " + entry2(objectRaycasts<HalfSphere>().hits.load(), objectRaycasts<HalfSphere>().all.load()) + "\n";
                out += "OrientedBox3:\n";
                out += "   hits/all: " + entry2(objectRaycasts<OrientedBox3>().hits.load(), objectRaycasts<OrientedBox3>().all.load()) + "\n";
                out += "Plane:\n";
                out += "   hits/all: " + entry2(objectRaycasts<Plane>().hits.load(), objectRaycasts<Plane>().all.load()) + "\n";
                out += "Sphere:\n";
                out += "   hits/all: " + entry2(objectRaycasts<Sphere>().hits.load(), objectRaycasts<Sphere>().all.load()) + "\n";
                out += "Triangle3:\n";
                out += "   hits/all: " + entry2(objectRaycasts<Triangle3>().hits.load(), objectRaycasts<Triangle3>().all.load()) + "\n";
                out += "\n";
                out += "BV Raycasts:\n";
                out += "Box3:\n";
                out += "   hits/all: " + entry2(bvRaycasts<Box3>().hits.load(), bvRaycasts<Box3>().all.load()) + "\n";
                out += "\n";
                out += "Interval Raycasts:\n";
                out += "Box3:\n";
                out += "   hits/all: " + entry2(intervalRaycasts<Box3>().hits.load(), intervalRaycasts<Box3>().all.load()) + "\n";
                out += "Capsule:\n";
                out += "   hits/all: " + entry2(intervalRaycasts<Capsule>().hits.load(), intervalRaycasts<Capsule>().all.load()) + "\n";
                out += "Cylinder:\n";
                out += "   hits/all: " + entry2(intervalRaycasts<Cylinder>().hits.load(), intervalRaycasts<Cylinder>().all.load()) + "\n";
                out += "OrientedBox3:\n";
                out += "   hits/all: " + entry2(intervalRaycasts<OrientedBox3>().hits.load(), intervalRaycasts<OrientedBox3>().all.load()) + "\n";
                out += "Sphere:\n";
                out += "   hits/all: " + entry2(intervalRaycasts<Sphere>().hits.load(), intervalRaycasts<Sphere>().all.load()) + "\n";
                out += "\n";
                out += "Dist Raycasts:\n";
                out += "Disc3:\n";
                out += "   hits/all: " + entry2(distRaycasts<Disc3>().hits.load(), distRaycasts<Disc3>().all.load()) + "\n";
                out += "HalfSphere:\n";
                out += "   hits/all: " + entry2(distRaycasts<HalfSphere>().hits.load(), distRaycasts<HalfSphere>().all.load()) + "\n";

                return out;
            }

            template <typename ShapeT>
            decltype(auto) objectRaycasts() const
            {
                return std::get<ObjectRaycastStats<ShapeT>>(m_raycasts);
            }

            template <typename BvShapeT>
            decltype(auto) bvRaycasts() const
            {
                return std::get<BvRaycastStats<BvShapeT>>(m_raycasts);
            }

            template <typename ShapeT>
            decltype(auto) intervalRaycasts() const
            {
                return std::get<IntervalRaycastStats<ShapeT>>(m_raycasts);
            }

            template <typename ShapeT>
            decltype(auto) distRaycasts() const
            {
                return std::get<DistRaycastStats<ShapeT>>(m_raycasts);
            }

        private:
            std::vector<TraceStats> m_tracesByDepth;
            std::tuple<
                ObjectRaycastStats<Box3>,
                ObjectRaycastStats<Capsule>,
                ObjectRaycastStats<ClosedTriangleMeshFace>,
                ObjectRaycastStats<Cylinder>,
                ObjectRaycastStats<Disc3>,
                ObjectRaycastStats<HalfSphere>,
                ObjectRaycastStats<OrientedBox3>,
                ObjectRaycastStats<Plane>,
                ObjectRaycastStats<Sphere>,
                ObjectRaycastStats<Triangle3>,

                BvRaycastStats<Box3>,

                IntervalRaycastStats<Box3>,
                IntervalRaycastStats<Capsule>,
                IntervalRaycastStats<Cylinder>,
                IntervalRaycastStats<OrientedBox3>,
                IntervalRaycastStats<Sphere>,

                DistRaycastStats<Disc3>,
                DistRaycastStats<HalfSphere>
            > m_raycasts;
            TimeStats m_traceDuration;
            TimeStats m_constructionDuration;

            template <typename ShapeT>
            decltype(auto) objectRaycasts()
            {
                return std::get<ObjectRaycastStats<ShapeT>>(m_raycasts);
            }

            template <typename BvShapeT>
            decltype(auto) bvRaycasts()
            {
                return std::get<BvRaycastStats<BvShapeT>>(m_raycasts);
            }

            template <typename ShapeT>
            decltype(auto) intervalRaycasts()
            {
                return std::get<IntervalRaycastStats<ShapeT>>(m_raycasts);
            }

            template <typename ShapeT>
            decltype(auto) distRaycasts()
            {
                return std::get<DistRaycastStats<ShapeT>>(m_raycasts);
            }
        };

        inline PerformanceStats gPerfStats;
    }
}
