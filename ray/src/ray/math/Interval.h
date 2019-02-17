#pragma once

#include <algorithm>
#include <vector>

namespace ray
{
    template <typename DataT>
    struct Interval
    {
        float min;
        float max;
        DataT minData;
        DataT maxData;
    };

    template <typename DataT>
    bool operator==(const Interval<DataT>& lhs, const Interval<DataT>& rhs) noexcept
    {
        return lhs.min == rhs.min
            && lhs.max == rhs.max
            && lhs.minData == rhs.minData
            && lhs.maxData == rhs.maxData;
    }

    template <typename DataT>
    struct IntervalSet
    {
        IntervalSet() = default;

        IntervalSet(Interval<DataT> interval)
        {
            m_intervals.emplace_back(std::move(interval));
        }

        // assumes that it can safely be inserted at the end
        void pushBack(Interval<DataT> interval)
        {
            m_intervals.emplace_back(std::move(interval));
        }

        void reserve(int n)
        {
            m_intervals.reserve(n);
        }

        int size() const
        {
            return static_cast<int>(m_intervals.size());
        }

        bool isEmpty() const
        {
            return m_intervals.empty();
        }

        float min() const
        {
            return m_intervals.front().min;
        }

        float max() const
        {
            return m_intervals.back().max;
        }

        const Interval<DataT>& operator[](int i) const
        {
            return m_intervals[i];
        }

        const Interval<DataT>& front() const
        {
            return m_intervals.front();
        }

        const Interval<DataT>& back() const
        {
            return m_intervals.back();
        }

        auto begin() const
        {
            return m_intervals.cbegin();
        }

        auto end() const
        {
            return m_intervals.cend();
        }

        bool operator==(const IntervalSet<DataT>& rhs) const noexcept
        {
            if (size() != rhs.size()) return false;
            return std::equal(begin(), end(), rhs.begin());
        }

        IntervalSet<DataT>& operator|=(const IntervalSet<DataT>& rhs)
        {
            if (rhs.isEmpty()) return *this;
            if (isEmpty() || rhs.front().min > max())
            {
                m_intervals.insert(m_intervals.end(), rhs.begin(), rhs.end());
                return *this;
            }

            m_scratch.clear();

            auto i = begin();
            auto endi = end();
            auto j = rhs.begin();
            auto endj = rhs.end();

            while (i != endi && j != endj)
            {
                if (j->min < i->min)
                {
                    std::swap(i, j);
                    std::swap(endi, endj);
                }
                // the interval starts at i->min
                m_scratch.emplace_back();
                Interval<DataT>& interval = m_scratch.back();
                interval.min = i->min;
                interval.minData = i->minData;

                for (;;)
                {
                    while (j != endj && j->max <= i->max)
                    {
                        ++j;
                        // move j to the first interval that extends past i
                    }
                    if (j == endj || j->min > i->max)
                    {
                        // i is the last interval 
                        // or j is only after i
                        // cannot be extended => i ends the interval
                        interval.max = i->max;
                        interval.maxData = i->maxData;
                        ++i;
                        break;
                    }
                    else
                    {
                        // can be extended by j
                        // make j current and try to extend it in the next one
                        ++i;
                        std::swap(i, j);
                        std::swap(endi, endj);
                    }
                }
            }

            // handle intervals that were left
            // rhs can't leave any, but *this can
            // but we don't know which iter is which
            if (i != endi) m_scratch.insert(m_scratch.end(), i, endi);
            else if (j != endj) m_scratch.insert(m_scratch.end(), j, endj);

            m_intervals.swap(m_scratch);

            return *this;
        }

        IntervalSet<DataT>& operator&=(const IntervalSet<DataT>& rhs)
        {
            if (rhs.isEmpty() || isEmpty() || rhs.front().min > max())
            {
                m_intervals.clear();
                return *this;
            }

            m_scratch.clear();

            auto i = begin();
            auto endi = end();
            auto j = rhs.begin();
            auto endj = rhs.end();

            while (i != endi && j != endj)
            {
                if (j->min > i->min)
                {
                    std::swap(i, j);
                    std::swap(endi, endj);
                }
                // the interval starts at i->min
                // j->min is before i->min
                while (j != endj && i->min > j->max) ++j;
                if (j == endj) break;
                if (j->min <= i->min)
                {
                    // we have an intersection [i->min, ...]
                    // intersection can't span more then 2 intervals
                    m_scratch.emplace_back();
                    Interval<DataT>& interval = m_scratch.back();
                    interval.min = i->min;
                    interval.minData = i->minData;

                    // check what ends first
                    if (i->max < j->max)
                    {
                        // ends on i->max
                        interval.max = i->max;
                        interval.maxData = i->maxData;
                        ++i;
                    }
                    else
                    {
                        // ends on j->max
                        interval.max = j->max;
                        interval.maxData = j->maxData;
                        ++j;
                    }
                }
            }

            m_intervals.swap(m_scratch);

            return *this;
        }

        IntervalSet<DataT>& operator-=(const IntervalSet<DataT>& rhs)
        {
            if (rhs.isEmpty())
            {
                return *this;
            }
            if (isEmpty() || rhs.front().min > max())
            {
                return *this;
            }

            m_scratch.clear();

            auto i = m_intervals.begin();
            auto endi = m_intervals.end();
            auto j = rhs.begin();
            auto endj = rhs.end();

            while (i != endi && j != endj)
            {
                while (j != endj && j->max < i->min) ++j;
                if (j == endj) break;
                // j ends after the start of i
                if (j->min > i->min)
                {
                    // something is cut
                    m_scratch.emplace_back();
                    Interval<DataT>& interval = m_scratch.back();
                    interval.min = i->min;
                    interval.minData = i->minData;

                    // decide what ends the cut
                    if (j->min > i->max)
                    {
                        // i->max ends the interval
                        interval.max = i->max;
                        interval.maxData = i->maxData;
                    }
                    else
                    {
                        // j->min ends the interval
                        interval.max = j->min;
                        interval.maxData = j->minData;
                    }
                }
                if (j->max >= i->max)
                {
                    // j covers whole i
                    ++i;
                }
                else
                {
                    // j->max is inside i and may start another interval
                    // truncate i
                    i->min = j->max;
                    i->minData = j->maxData;
                    ++j;
                }
            }

            // add whatever is left in lhs, nothing can subtract it
            if (i != endi) m_scratch.insert(m_scratch.end(), i, endi);

            m_intervals.swap(m_scratch);

            return *this;
        }

    private:
        std::vector<Interval<DataT>> m_intervals;
        std::vector<Interval<DataT>> m_scratch;
    };

    template <typename DataT>
    IntervalSet<DataT> operator|(const IntervalSet<DataT>& lhs, const IntervalSet<DataT>& rhs)
    {
        auto result = lhs;
        result |= rhs;
        return result;
    }
    template <typename DataT>
    IntervalSet<DataT> operator|(IntervalSet<DataT>&& lhs, const IntervalSet<DataT>& rhs)
    {
        lhs |= rhs;
        return std::move(lhs);
    }
    template <typename DataT>
    IntervalSet<DataT> operator|(const IntervalSet<DataT>& lhs, IntervalSet<DataT>&& rhs)
    {
        return rhs | lhs;
    }
    template <typename DataT>
    IntervalSet<DataT> operator|(IntervalSet<DataT>&& lhs, IntervalSet<DataT>&& rhs)
    {
        lhs |= rhs;
        return std::move(lhs);
    }

    template <typename DataT>
    IntervalSet<DataT> operator&(const IntervalSet<DataT>& lhs, const IntervalSet<DataT>& rhs)
    {
        auto result = lhs;
        result &= rhs;
        return result;
    }
    template <typename DataT>
    IntervalSet<DataT> operator&(IntervalSet<DataT>&& lhs, const IntervalSet<DataT>& rhs)
    {
        lhs &= rhs;
        return std::move(lhs);
    }
    template <typename DataT>
    IntervalSet<DataT> operator&(const IntervalSet<DataT>& lhs, IntervalSet<DataT>&& rhs)
    {
        return rhs & lhs;
    }
    template <typename DataT>
    IntervalSet<DataT> operator&(IntervalSet<DataT>&& lhs, IntervalSet<DataT>&& rhs)
    {
        lhs &= rhs;
        return std::move(lhs);
    }

    template <typename DataT>
    IntervalSet<DataT> operator-(const IntervalSet<DataT>& lhs, const IntervalSet<DataT>& rhs)
    {
        auto result = lhs;
        result -= rhs;
        return result;
    }
    template <typename DataT>
    IntervalSet<DataT> operator-(IntervalSet<DataT>&& lhs, const IntervalSet<DataT>& rhs)
    {
        lhs -= rhs;
        return std::move(lhs);
    }
}
