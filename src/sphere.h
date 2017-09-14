#ifndef SOFTBIVPT_SPHERE_H_
#define SOFTBIVPT_SPHERE_H_

#include "shape.h"

namespace softbivpt {

class Sphere : public Shape {
public:
    Sphere(const double radius, const vec3 &position, const vec3 &emission, const vec3 &color, ReflectionType refl) : 
        position(position) {
            this->radius = radius;
            this->position = position;
            this->emission = emission;
            this->color = color;
            this->refl = refl;
    }

    double Intersect(const Ray &ray, double *tin = nullptr, double *tout = nullptr) const { // Jiyu
        /* Check for ray-sphere intersection by solving for t:
            t^2*d.d + 2*t*(o-p).d + (o-p).(o-p) - R^2 = 0 */
        vec3 op = position - ray.org;
        double eps = 1e-4;
        double b = op.Dot(ray.dir);
        double radicant = b * b - op.Dot(op) + radius * radius;
        if (radicant < 0.0) return 0.0;      /* No intersection */
        else radicant = sqrt(radicant);

        //Jiyu
        if (tin && tout) {
            *tin = (b-radicant<=0) ? 0 : b-radicant;
            *tout=b+radicant;
        }
        //////

        double t;
        t = b - radicant;    /* Check smaller root first */
        if (t > eps) return t;

        t = b + radicant;
        if (t > eps) return t;     /* Check second root */

        return 0.0;          /* No intersection in ray direction */
    }

    const vec3& GetPosition() const {
        return position;
    }

    vec3 GetNormal(const vec3 &hitpoint) const {
        return (hitpoint - position).Normalized();  /* Normal at intersection */
    }

private:
    vec3 position;
};

} // namespace softbivpt

#endif // SOFTBIVPT_SPHERE_H_
