#ifndef SOFTBIVPT_TRIANGLE_H_
#define SOFTBIVPT_TRIANGLE_H_

#include "shape.h"

namespace softbivpt {

class Triangle : public Shape {
public:
    vec3 a, b, c;

    Triangle(const vec3 &a, const vec3 &b, const vec3 &c, const Color &emission, const Color &color, ReflectionType refl) :
        a(a), b(b), c(c) {
        this->emission = emission;
		this->color = color;
		this->refl = refl;

        vec3 temp_v0 = b - a, temp_v1 = c - a;
        normal = temp_v0.Cross(temp_v1);
        normal = normal.Normalized();
        center = (a+b+c) / 3.0;

        //farthest point away from center has to be vertice
        double length_to_a = (center - a).Length();
        double length_to_b = (center - b).Length();
        double length_to_c = (center - c).Length();
        radius = std::fmax(std::fmax(length_to_a, length_to_b), length_to_c);
    }

    /* triangle-ray intersection */
    double Intersect(const Ray &ray, double *tin = nullptr, double *tout = nullptr) const {
        vec3 edge1 = b - a;
        vec3 edge2 = c - a;
        vec3 dir = ray.dir;
        vec3 orig = ray.org;
        vec3 pvec = dir.Cross(edge2);
        double det = edge1.Dot(pvec);

        if (det == 0) return 0.0;

        double invDet = 1.0 / det;
        vec3 tvec = orig - a;
        double u = tvec.Dot(pvec) * invDet;

        if (u < 0 || u > 1) return 0.0;

        vec3 qvec = tvec.Cross(edge1);
        double v = dir.Dot(qvec) * invDet;

        if (v < 0 || u + v > 1) return 0.0;

        double t = edge2.Dot(qvec) * invDet;

        if (t <= 0.00000001) return 0.0;
        return t;
    }

    const vec3& GetPosition() const {
        return center;
    }

    vec3 GetNormal(const vec3 &hitpoint) const {
        return normal;
    }

private:
    vec3 center;
    vec3 normal;

};

} // namespace softbivpt

#endif // SOFTBIVPT_TRIANGLE_H_
