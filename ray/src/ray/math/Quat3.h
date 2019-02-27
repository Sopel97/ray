#pragma once

#include "detail/M128Math.h"

#include "Angle2.h"
#include "AxisAngle3f.h"
#include "EulerAngles2.h"

#include <cmath>

namespace ray
{
    template <typename T>
    struct Quat3;

    template<>
    struct Quat3<float>
    {
    public:
        union {
            struct {
                float x, y, z, w;
            };
            __m128 xmm;
        };


        static Quat3<float> identity()
        {
            return {};
        }

        Quat3() noexcept :
            x(0),
            y(0),
            z(0),
            w(1)
        {

        }

        Quat3(float x, float y, float z, float w) :
            x(x),
            y(y),
            z(z),
            w(w)
        {

        }

        Quat3(EulerAngles3<float> a)
        {
            a.pitch *= 0.5f;
            a.yaw *= 0.5f;
            a.roll *= 0.5f;

            const auto [sp, cp] = a.pitch.sincos();
            const auto [sy, cy] = a.yaw.sincos();
            const auto [sr, cr] = a.roll.sincos();

            x = sp * cy * cr - cp * sy * sr;
            y = cp * sy * cr + sp * cy * sr;
            z = cp * cy * sr - sp * sy * cr;
            w = cp * cy * cr + sp * sy * sr;
        }

        Quat3(const AxisAngle3<float>& a)
        {
            const Angle2<float> halfAngle = a.angle() * 0.5f;

            const auto [s, c] = halfAngle.sincos();

            w = c;
            x = a.axis().x * s;
            y = a.axis().y * s;
            z = a.axis().z * s;
        }

        Quat3(const Quat3<float>&) = default;
        Quat3(Quat3<float>&&) noexcept = default;
        Quat3<float>& operator=(const Quat3<float>&) = default;
        Quat3<float>& operator=(Quat3<float>&&) noexcept = default;

        Quat3<float>& operator+=(const Quat3<float>& rhs)
        {
            xmm = detail::add(xmm, rhs.xmm);

            return *this;
        }

        Quat3<float>& operator-=(const Quat3<float>& rhs)
        {
            xmm = detail::sub(xmm, rhs.xmm);

            return *this;
        }

        Quat3<float>& operator*=(const Quat3<float>& rhs)
        {
            /*
            const float newX = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
            const float newY = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
            const float newZ = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;
            const float newW = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
            */

            __m128 xxxx, yyyy, zzzz, wwww;
            detail::spill(xmm, xxxx, yyyy, zzzz, wwww);

            xmm = 
                detail::add(
                    detail::mul(wwww, rhs.xmm),
                    detail::neg(detail::mul(xxxx, _mm_shuffle_ps(rhs.xmm, rhs.xmm, _MM_SHUFFLE(3, 2, 1, 0))), detail::mask128(false, true, false, true)),
                    detail::neg(detail::mul(yyyy, _mm_shuffle_ps(rhs.xmm, rhs.xmm, _MM_SHUFFLE(2, 3, 0, 1))), detail::mask128(false, false, true, true)),
                    detail::neg(detail::mul(zzzz, _mm_shuffle_ps(rhs.xmm, rhs.xmm, _MM_SHUFFLE(1, 0, 3, 2))), detail::mask128(true, false, false, true))
                );

            return *this;
        }

        constexpr Quat3<float>& operator*=(float rhs)
        {
            xmm = detail::mul(xmm, rhs);

            return *this;
        }

        constexpr Quat3<float>& operator/=(float rhs)
        {
            xmm = detail::div(xmm, rhs);

            return *this;
        }

        Quat3<float> normalized() const
        {
            Quat3<float> result(*this);
            result.normalize();
            return result;
        }

        void normalize()
        {
            (*this) *= invLength();
        }

        Quat3<float> conjugate() const
        {
            return Quat3<float>(w, -x, -y, -z);
        }

        void invert()
        {
            (*this) = conjugate() * invLength();
        }

        Quat3<float> inverse() const
        {
            Quat3<float> result(*this);
            result.invert();
            return result;
        }

        float dot(const Quat3<float>& rhs) const
        {
            return detail::dot(xmm, rhs.xmm);
        }

        // http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
        Vec3<float> apply(const Vec3<float>& v) const
        {
            const Vec3<float> q(x, y, z);
            const Vec3<float> t = 2.0f * cross(q, v);
            const Vec3<float> r = v + (w * t) + cross(q, t);
            return r;
        }

        float lengthSqr() const
        {
            return detail::dot(xmm, xmm);
        }

        float length() const
        {
            return std::sqrt(lengthSqr());
        }

        float length3() const
        {
            return std::sqrt(detail::dot3(xmm, xmm));
        }

        float invLength() const
        {
            return 1.0f / std::sqrt(detail::dot(xmm, xmm));
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

    // e^q
    template <typename T>
    Quat3<T> exp(const Quat3<T>& q)
    {
        static constexpr T eps = T(0.00001);

        using std::sqrt;
        using std::sin;
        using std::cos;

        const T vlen = sqrt(q.length3());

        if (vlen > eps)
        {
            const T s = sin(vlen);
            const T c = cos(vlen);
            const T scale = s / vlen;
            return Quat3<T>(
                scale * x,
                scale * y,
                scale * z,
                c
                );
        }
        else
        {
            return Quat3<T>{};
        }
    }

    template <typename T>
    Quat3<T> log(const Quat3<T>& q)
    {
        using std::atan2;
        using std::log;
        using std::sqrt;

        static constexpr T eps = T(0.00001);

        const T vlen = sqrt(q.length3());

        if (vlen > eps)
        {
            const T t = atan2(vlen, w) / vlen;
            const T len = vlen * vlen + w * w;
            return Quat3<T>(t * x, t * y, t * z, T(0.5) * log(len));
        }
        else
        {
            if (w > T(0))
            {
                return Quat3<T>(T(0), T(0), T(0), log(w));
            }
            else if (w < T(0))
            {
                return Quat3<T>(static_cast<T>(pi), T(0), T(0), log(-w));
            }
            else
            {
                return Quat3<T>(
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity()
                    );
            }
        }
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