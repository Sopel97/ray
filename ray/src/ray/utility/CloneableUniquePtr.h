#pragma once

#include <memory>

namespace ray
{
    template <typename T>
    struct CloneableUniquePtr
    {
        template <typename U>
        CloneableUniquePtr(std::unique_ptr<U>&& ptr) :
            m_ptr(std::move(ptr))
        {

        }

        template <typename U>
        CloneableUniquePtr(const CloneableUniquePtr<U>& other) :
            m_ptr(other.m_ptr->clone())
        {

        }

        CloneableUniquePtr(const CloneableUniquePtr<T>& other) :
            m_ptr(other.m_ptr->clone())
        {

        }

        template <typename U>
        CloneableUniquePtr& operator=(const CloneableUniquePtr<U>& other)
        {
            m_ptr = other.m_ptr->clone();
        }
        CloneableUniquePtr& operator=(const CloneableUniquePtr<T>& other)
        {
            m_ptr = other.m_ptr->clone();
        }

        [[nodiscard]] T* operator->()
        {
            return m_ptr.get();
        }

        [[nodiscard]] const T* operator->() const
        {
            return m_ptr.get();
        }

    private:
        std::unique_ptr<T> m_ptr;
    };
}
