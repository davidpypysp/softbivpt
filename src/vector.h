#ifndef SOFTBIVPT_VECTOR_H_
#define SOFTBIVPT_VECTOR_H_

#include <cmath>

namespace softbivpt {

struct vec3 {
    double x, y, z;

    vec3(const vec3 &b) : x(b.x), y(b.y), z(b.z) {}
    vec3(const double x = 0.0, const double y = 0.0, const double z = 0.0) : x(x), y(y), z(z) {}
    vec3 operator + (const vec3 &b) const { return vec3(x + b.x, y + b.y, z + b.z); }
    vec3 operator - (const vec3 &b) const { return vec3(x - b.x, y - b.y, z - b.z); }
    vec3 operator * (const double c) const { return vec3(x * c, y * c, z * c); }
    friend vec3 operator * (const double c, const vec3 &b) { return b * c; }
    vec3 operator / (const double c) const { return vec3(x / c, y / c, z / c); }
    vec3& operator = (const vec3 &b) {x = b.x; y = b.y; z = b.z; return *this; }
	bool operator == (const vec3 &b) const { return (x == b.x && y == b.y && z == b.z); }
    vec3 MultComponents(const vec3 &b) const { return vec3(x * b.x, y * b.y, z * b.z); }
    vec3 Normalized() const { return vec3(x, y, z) / sqrt(x * x + y * y + z * z); }
    const double Dot(const vec3 &b) const { return x * b.x + y * b.y + z * b.z; }
    const vec3 Cross(const vec3 &b) const { return vec3((y * b.z) - (z * b.y), (z * b.x) - (x * b.z), (x * b.y) - (y * b.x)); }
    const double LengthSquared() const { return x * x + y * y + z * z; }
    const double Length() const { return sqrt(LengthSquared()); }
    const double Max() { return fmax(x, fmax(y, z)); }

    vec3 &clamp() {
        x = x < 0 ? 0.0 : x > 1.0 ? 1.0 : x;
        y = y < 0 ? 0.0 : y > 1.0 ? 1.0 : y;
        z = z < 0 ? 0.0 : z > 1.0 ? 1.0 : z;
        return *this;
    }
};

typedef vec3 Color;

} // namespace softbivpt


#endif // SOFTBIVPT_VECTOR_H_
