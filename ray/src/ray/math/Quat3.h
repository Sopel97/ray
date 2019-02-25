#pragma once

#include "detail/M128Math.h"

#include "Angle2.h"
#include "AxisAngle3f.h"
#include "EulerAngles2.h"

#include <cmath>

namespace ray
{
    template <typename T>
    struct Quat3
    {
    public:
        float w, x, y, z;

        constexpr static Quat3<T> identity()
        {
            return {};
        }

        constexpr Quat3() noexcept :
            w(1),
            x(0),
            y(0),
            z(0)
        {

        }

        constexpr Quat3(const T& w, const T& x, const T& y, const T& z) :
            w(w),
            x(x),
            y(y),
            z(z)
        {

        }

        Quat3(EulerAngles3f a)
        {
            a.pitch *= T(0.5);
            a.yaw *= T(0.5);
            a.roll *= T(0.5);

            const T [sp, cp] = a.pitch.sincos();
            const T [sy, cy] = a.yaw.sincos();
            const T [sr, cr] = a.roll.sincos();
            
            z = cp * cy * sr - sp * sy * cr;
            w = cp * cy * cr + sp * sy * sr;
            x = sp * cy * cr - cp * sy * sr;
            y = cp * sy * cr + sp * cy * sr;
        }

        Quat3(const AxisAngle3<T>& a)
        {
            const Angle2<T> halfAngle = a.angle() * T(0.5);

            const T [s, c] = halfAngle.sincos();

            w = c;
            x = a.axis().x * s;
            y = a.axis().y * s;
            z = a.axis().z * s;
        }

        constexpr Quat3(const Quat3<T>&) = default;
        constexpr Quat3(Quat3<T>&&) noexcept = default;
        constexpr Quat3<T>& operator=(const Quat3<T>&) = default;
        constexpr Quat3<T>& operator=(Quat3<T>&&) noexcept = default;

        constexpr Quat3<T>& operator+=(const Quat3<T>& rhs)
        {
            w += rhs.w;
            x += rhs.x;
            y += rhs.y;
            z += rhs.z;

            return *this;
        }

        constexpr Quat3<T>& operator-=(const Quat3<T>& rhs)
        {
            w -= rhs.w;
            x -= rhs.x;
            y -= rhs.y;
            z -= rhs.z;

            return *this;
        }

        constexpr Quat3<T>& operator*=(const Quat3<T>& rhs)
        {
            const T newW = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
            const T newX = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
            const T newY = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
            const T newZ = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;

            w = newW;
            x = newX;
            y = newY;
            z = newZ;

            return *this;
        }

        constexpr Quat3<T>& operator*=(const T& rhs)
        {
            w *= rhs;
            x *= rhs;
            y *= rhs;
            z *= rhs;

            return *this;
        }

        constexpr Quat3<T>& operator/=(const T& rhs)
        {
            w /= rhs;
            x /= rhs;
            y /= rhs;
            z /= rhs;

            return *this;
        }

        Quat3<T> normalized() const
        {
            Quat3<T> result(*this);
            result.normalize();
            return result;
        }

        void normalize()
        {
            const T scale = T(1) / length();
            (*this) *= scale;
        }

        constexpr Quat3<T> conjugate() const
        {
            return Quat3<T>(w, -x, -y, -z);
        }

        void invert()
        {
            const T scale = T(1) / lengthSquared();

            (*this) = conjugate() * scale;
        }

        Quat3<T> inverse() const
        {
            Quat3<T> result(*this);
            result.invert();
            return result;
        }

        constexpr T dot(const Quat3<T>& rhs) const
        {
            return w * rhs.w + x * rhs.x + y * rhs.y + z * rhs.z;
        }

        // http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
        constexpr Vec3<T> apply(const Vec3<T>& v) const
        {
            const Vec3<T> q(x, y, z);
            const Vec3<T> t = T(2) * q.cross(v);
            const Vec3<T> r = v + (w * t) + q.cross(t);
            return r;
        }

        constexpr T lengthSquared() const
        {
            return w * w + x * x + y * y + z * z;
        }

        T length() const
        {
            using std::sqrt;

            return sqrt(lengthSquared());
        }

        // e^q
        Quat3<T> exp() const
        {
            static constexpr T eps = T(0.00001);

            using std::sqrt;
            using std::sin;
            using std::cos;

            const T vlen = sqrt(x*x + y * y + z * z);

            if (vlen > eps)
            {
                const T s = sin(vlen);
                const T c = cos(vlen);
                const T scale = s / vlen;
                return Quat3<T>(
                    c,
                    scale * x,
                    scale * y,
                    scale * z
                    );
            }
            else
            {
                return Quat3<T>(T(1), T(0), T(0), T(0));
            }
        }

        Quat3<T> log() const
        {
            using std::atan2;
            using std::log;

            static constexpr T eps = T(0.00001);

            const T vlen = sqrt(x*x + y * y + z * z);

            if (vlen > eps)
            {
                const T t = atan2(vlen, w) / vlen;
                const T len = vlen * vlen + w * w;
                return Quat3<T>(T(0.5) * log(len), t * x, t * y, t * z);
            }
            else
            {
                if (w > T(0))
                {
                    return Quat3<T>(log(w), T(0), T(0), T(0));
                }
                else if (w < T(0))
                {
                    return Quat3<T>(log(-w), pi<T>, T(0), T(0));
                }
                else
                {
                    return Quat3<T>(std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity(), std::numeric_limits<T>::infinity());
                }
            }
        }


    private:
    };

    using Quat3d = Quat3<double>;
    using Quat3f = Quat3<float>;

    template <typename T>
    Quat3<T> operator+(const Quat3<T>& lhs, const Quat3<T>& rhs)
    {
        Quat3<T> result(lhs);
        return (result += rhs);
    }

    template <typename T>
    Quat3<T> operator-(const Quat3<T>& lhs, const Quat3<T>& rhs)
    {
        Quat3<T> result(lhs);
        return (result -= rhs);
    }

    template <typename T>
    Quat3<T> operator*(const Quat3<T>& lhs, const Quat3<T>& rhs)
    {
        Quat3<T> result(lhs);
        return (result *= rhs);
    }

    template <typename T>
    Quat3<T> operator*(const Quat3<T>& lhs, const T& rhs)
    {
        Quat3<T> result(lhs);
        return (result *= rhs);
    }

    template <typename T>
    Quat3<T> operator/(const Quat3<T>& lhs, const T& rhs)
    {
        Quat3<T> result(lhs);
        return (result /= rhs);
    }

    template <typename T>
    Vec3<T> operator*(const Vec3<T>& lhs, const Quat3<T>& rhs)
    {
        return rhs.apply(lhs);
    }


    // https://en.wikipedia.org/wiki/Slerp
    template <typename T>
    constexpr Quat3<T> slerp(const Quat3<T>& lhs, const Quat3<T>& rhs, const T& t)
    {
        using std::sin;
        using std::cos;
        using std::acos;

        static constexpr T eps = T(0.01);

        Quat3<T> q = rhs;
        T cosTheta = std::clamp(lhs.dot(rhs), T(-1.0), T(1.0));

        // take shorter path
        if (cosTheta < T(0))
        {
            q = -rhs;
            cosTheta = -cosTheta;
        }

        // avoid division by 0, use lerp if sin(theta) ~ 0
        if (cosTheta > T(1) - eps)
        {
            return nlerp(lhs, rhs, t);
        }
        else
        {
            //const T theta = acos(cosTheta);
            // return (sin((T(1) - t) * theta) * lhs + sin(t * theta) * q) / sin(theta);

            const T theta = acos(cosTheta) * t;
            q -= lhs * cosTheta;
            q.normalize();
            // { lhs, q } now an orthonormal basis
            return lhs * cos(theta) + q * sin(theta);
        }
    }
}