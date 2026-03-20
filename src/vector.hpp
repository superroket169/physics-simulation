#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <cstddef>
#include <cmath>

namespace inert {

    // ==========================================
    //                  VECTOR
    // ==========================================

    template<typename T, size_t N>
    class vector {
    private:
        T data[N];

    public:

        // Default constructor — fills with zero
        vector() {
            for (size_t i = 0; i < N; i++) data[i] = T(0);
        }

        // ==========================================
        //              INDEX OPERATORS
        // ==========================================

        T&       operator[](size_t i)       { return data[i]; }
        const T& operator[](size_t i) const { return data[i]; }

        // ==========================================
        //          ARITHMETIC OPERATORS
        // ==========================================

        vector operator+(const vector& other) const {
            vector result;
            for (size_t i = 0; i < N; i++)
                result[i] = data[i] + other[i];
            return result;
        }

        vector operator-(const vector& other) const {
            vector result;
            for (size_t i = 0; i < N; i++)
                result[i] = data[i] - other[i];
            return result;
        }

        vector operator-() const {
            vector result;
            for (size_t i = 0; i < N; i++)
                result[i] = -data[i];
            return result;
        }

        vector operator*(T scalar) const {
            vector result;
            for (size_t i = 0; i < N; i++)
                result[i] = data[i] * scalar;
            return result;
        }

        vector operator/(T scalar) const {
            vector result;
            for (size_t i = 0; i < N; i++)
                result[i] = data[i] / scalar;
            return result;
        }

        // ==========================================
        //        COMPOUND ASSIGNMENT OPERATORS
        // ==========================================

        vector& operator+=(const vector& other) {
            for (size_t i = 0; i < N; i++) data[i] += other[i];
            return *this;
        }

        vector& operator-=(const vector& other) {
            for (size_t i = 0; i < N; i++) data[i] -= other[i];
            return *this;
        }

        vector& operator*=(T scalar) {
            for (size_t i = 0; i < N; i++) data[i] *= scalar;
            return *this;
        }

        vector& operator/=(T scalar) {
            for (size_t i = 0; i < N; i++) data[i] /= scalar;
            return *this;
        }

        // ==========================================
        //                  GETTERS
        // ==========================================

        T getDotProduct(const vector& other) const {
            T result = T(0);
            for (size_t i = 0; i < N; i++)
                result += data[i] * other[i];
            return result;
        }

        T getLengthSqr() const { return getDotProduct(*this); }
        T getLength() const { return static_cast<T>(sqrt(getLengthSqr())); }

        vector getNormalized() const {
            T len = getLength();
            if (len < T(0.0001)) return *this;
            return *this / len;
        }

        T x() const { static_assert(N >= 1, "Vector must have at least 1 component"); return data[0]; }
        T y() const { static_assert(N >= 2, "Vector must have at least 2 components"); return data[1]; }
        T z() const { static_assert(N >= 3, "Vector must have at least 3 components"); return data[2]; }
        T w() const { static_assert(N >= 4, "Vector must have at least 4 components"); return data[3]; }

        T& x() { static_assert(N >= 1, "Vector must have at least 1 component"); return data[0]; }
        T& y() { static_assert(N >= 2, "Vector must have at least 2 components"); return data[1]; }
        T& z() { static_assert(N >= 3, "Vector must have at least 3 components"); return data[2]; }
        T& w() { static_assert(N >= 4, "Vector must have at least 4 components"); return data[3]; }

    };

    // ==========================================
    //       CROSS PRODUCT — vec3 ONLY
    // ==========================================

    template<typename T>
    vector<T, 3> getCrossProduct(const vector<T, 3>& a, const vector<T, 3>& b) {
        vector<T, 3> result;
        result[0] = a[1] * b[2] - a[2] * b[1];
        result[1] = a[2] * b[0] - a[0] * b[2];
        result[2] = a[0] * b[1] - a[1] * b[0];
        return result;
    }

    template<typename T, size_t N>
    vector<T, N> operator*(T scalar, const vector<T, N>& v) {
        return v * scalar;
    }

    // ==========================================
    //              QUATERNION
    // ==========================================

    // Represents a rotation in 3D space
    // Stored as (x, y, z, w) where w is the scalar part
    template<typename T>
    struct quaternion {
        T x, y, z, w;

        // Identity quaternion — no rotation
        quaternion() : x(0), y(0), z(0), w(1) {}
        quaternion(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}

        T getLengthSqr() const {
            return x*x + y*y + z*z + w*w;
        }

        T getLength() const {
            return static_cast<T>(sqrt(getLengthSqr()));
        }

        quaternion getNormalized() const {
            T len = getLength();
            if (len < T(0.0001)) return *this;
            return { x/len, y/len, z/len, w/len };
        }

        quaternion operator*(const quaternion& other) const {
            return {
                w*other.x + x*other.w + y*other.z - z*other.y,
                w*other.y - x*other.z + y*other.w + z*other.x,
                w*other.z + x*other.y - y*other.x + z*other.w,
                w*other.w - x*other.x - y*other.y - z*other.z
            };
        }

        quaternion& operator*=(const quaternion& other) {
            *this = *this * other;
            return *this;
        }
    };

    // Rotate a vec3 by a quaternion
    template<typename T>
    vector<T, 3> rotate(const vector<T, 3>& v, const quaternion<T>& q) {
        vector<T, 3> qv;
        qv[0] = q.x; qv[1] = q.y; qv[2] = q.z;

        vector<T, 3> t  = getCrossProduct(qv, v) * T(2);
        vector<T, 3> t2 = getCrossProduct(qv, t);

        return v + t * q.w + t2;
    }

    // ==========================================
    //              ALIASES
    // ==========================================

    using vec2f = vector<float,  2>;
    using vec3f = vector<float,  3>;
    using vec4f = vector<float,  4>;
    using vec2i = vector<int,    2>;
    using vec3i = vector<int,    3>;
    using vec2d = vector<double, 2>;
    using vec3d = vector<double, 3>;

    using quatf = quaternion<float>;
    using quatd = quaternion<double>;

} // namespace inert

#endif // !VECTOR_HPP
