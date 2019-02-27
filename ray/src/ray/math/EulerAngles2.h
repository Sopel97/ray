#pragma once

#include "Angle2.h"

namespace ray
{
    template <typename T>
    struct EulerAngles3
    {
    public:
        Angle2<T> pitch;
        Angle2<T> yaw;
        Angle2<T> roll;

        constexpr EulerAngles3() noexcept = default;
        constexpr EulerAngles3(const Angle2<T>& pitch, const Angle2<T>& yaw, const Angle2<T>& roll) :
            pitch(pitch),
            yaw(yaw),
            roll(roll)
        {

        }
        constexpr EulerAngles3(const EulerAngles3<T>&) = default;
        constexpr EulerAngles3(EulerAngles3<T>&&) noexcept = default;
        constexpr EulerAngles3<T>& operator=(const EulerAngles3<T>&) = default;
        constexpr EulerAngles3<T>& operator=(EulerAngles3<T>&&) noexcept = default;
    };

    using EulerAngles3d = EulerAngles3<double>;
    using EulerAngles3f = EulerAngles3<float>;
}
