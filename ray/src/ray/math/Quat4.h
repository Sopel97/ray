#pragma once

#include "m128/M128Math.h"

#include "Angle2.h"
#include "AxisAngle3f.h"
#include "EulerAngles2.h"

#include <cmath>

#include "m128/M128MemberSwizzleGeneratorDef.h"

namespace ray
{
    template <typename T>
    struct Quat4;

    template<>
    struct Quat4<float>
    {
    public:
        union {
            struct {
                float x, y, z, w;
            };
            __m128 xmm;
        };

        RAY_GEN_MEMBER_SWIZZLE4_ALL(Quat4<float>)

        static Quat4<float> identity()
        {
            return {};
        }

        Quat4(__m128 xmm) noexcept :
            xmm(xmm)
        {

        }

        Quat4() noexcept :
            x(0),
            y(0),
            z(0),
            w(1)
        {

        }

        Quat4(float x, float y, float z, float w) :
            x(x),
            y(y),
            z(z),
            w(w)
        {

        }

        explicit Quat4(EulerAngles3<float> a)
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

        explicit Quat4(const AxisAngle3<float>& a)
        {
            const Angle2<float> halfAngle = a.angle() * 0.5f;

            const auto [s, c] = halfAngle.sincos();

            w = c;
            x = a.axis().x * s;
            y = a.axis().y * s;
            z = a.axis().z * s;
        }

        Quat4(const Quat4<float>&) = default;
        Quat4(Quat4<float>&&) noexcept = default;
        Quat4<float>& operator=(const Quat4<float>&) = default;
        Quat4<float>& operator=(Quat4<float>&&) noexcept = default;

        Quat4<float>& operator+=(const Quat4<float>& rhs)
        {
            xmm = m128::add(xmm, rhs.xmm);

            return *this;
        }

        Quat4<float>& operator-=(const Quat4<float>& rhs)
        {
            xmm = m128::sub(xmm, rhs.xmm);

            return *this;
        }

        Quat4<float>& operator*=(const Quat4<float>& rhs)
        {
            /*
            const float newX = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
            const float newY = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
            const float newZ = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;
            const float newW = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
            */

            __m128 xxxx, yyyy, zzzz, wwww;
            m128::spill(xmm, xxxx, yyyy, zzzz, wwww);

            xmm = 
                m128::add(
                    m128::mul(wwww, rhs.xmm),
                    m128::neg(m128::mul(xxxx, rhs.wzyx().xmm), m128::mask_yw()),
                    m128::neg(m128::mul(yyyy, rhs.zwxy().xmm), m128::mask_zw()),
                    m128::neg(m128::mul(zzzz, rhs.yxwz().xmm), m128::mask_xw())
                );

            return *this;
        }

        constexpr Quat4<float>& operator*=(float rhs)
        {
            xmm = m128::mul(xmm, rhs);

            return *this;
        }

        constexpr Quat4<float>& operator/=(float rhs)
        {
            xmm = m128::div(xmm, rhs);

            return *this;
        }

        Quat4<float> normalized() const
        {
            Quat4<float> result(*this);
            result.normalize();
            return result;
        }

        void normalize()
        {
            (*this) *= invLength();
        }

        Quat4<float> conjugate() const
        {
            return Quat4<float>(m128::neg(xmm, m128::mask(true, true, true, false)));
        }

        void invert()
        {
            const float invLen = invLength();
            (*this) = conjugate();
            (*this) *= invLen; // conjugate doesn't change it but we don't need to have a dependency
        }

        Quat4<float> inverse() const
        {
            Quat4<float> result(*this);
            result.invert();
            return result;
        }

        float dot(const Quat4<float>& rhs) const
        {
            return m128::dot(xmm, rhs.xmm);
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
            return m128::dot(xmm, xmm);
        }

        float length() const
        {
            return std::sqrt(lengthSqr());
        }

        float length3() const
        {
            return std::sqrt(m128::dot3(xmm, xmm));
        }

        float invLength() const
        {
            return 1.0f / std::sqrt(m128::dot(xmm, xmm));
        }
    private:
    };

    using Quat3d = Quat4<double>;
    using Quat3f = Quat4<float>;

    template <typename T>
    Quat4<T> operator+(const Quat4<T>& lhs, const Quat4<T>& rhs)
    {
        Quat4<T> result(lhs);
        return (result += rhs);
    }

    template <typename T>
    Quat4<T> operator-(const Quat4<T>& lhs, const Quat4<T>& rhs)
    {
        Quat4<T> result(lhs);
        return (result -= rhs);
    }

    template <typename T>
    Quat4<T> operator*(const Quat4<T>& lhs, const Quat4<T>& rhs)
    {
        Quat4<T> result(lhs);
        return (result *= rhs);
    }

    template <typename T>
    Quat4<T> operator*(const Quat4<T>& lhs, const T& rhs)
    {
        Quat4<T> result(lhs);
        return (result *= rhs);
    }

    template <typename T>
    Quat4<T> operator/(const Quat4<T>& lhs, const T& rhs)
    {
        Quat4<T> result(lhs);
        return (result /= rhs);
    }

    // e^q
    template <typename T>
    Quat4<T> exp(const Quat4<T>& q)
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
            return Quat4<T>(
                scale * q.x,
                scale * q.y,
                scale * q.z,
                c
                );
        }
        else
        {
            return Quat4<T>{};
        }
    }

    template <typename T>
    Quat4<T> log(const Quat4<T>& q)
    {
        using std::atan2;
        using std::log;
        using std::sqrt;

        static constexpr T eps = T(0.00001);

        const T vlen = sqrt(q.length3());

        if (vlen > eps)
        {
            const T t = atan2(vlen, q.w) / vlen;
            const T len = vlen * vlen + q.w * q.w;
            return Quat4<T>(t * q.x, t * q.y, t * q.z, T(0.5) * log(len));
        }
        else
        {
            if (q.w > T(0))
            {
                return Quat4<T>(T(0), T(0), T(0), log(q.w));
            }
            else if (q.w < T(0))
            {
                return Quat4<T>(static_cast<T>(pi), T(0), T(0), log(-q.w));
            }
            else
            {
                return Quat4<T>(
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
    constexpr Quat4<T> slerp(const Quat4<T>& lhs, const Quat4<T>& rhs, const T& t)
    {
        using std::sin;
        using std::cos;
        using std::acos;

        static constexpr T eps = T(0.01);

        Quat4<T> q = rhs;
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

#include "m128/M128MemberSwizzleGeneratorUndef.h"
