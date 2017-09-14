#ifndef SOFTBIVPT_SHAPE_H_
#define SOFTBIVPT_SHAPE_H_

#include "ray.h"

namespace softbivpt {

typedef vec3 Color;

enum ReflectionType {
    DIFF,
    SPEC, 
    REFR, 
    GLOSSY, 
    TRANSL 
};

class Shape {
public:
    virtual double Intersect(const Ray &ray, double *tin = nullptr, double *tout = nullptr) const  = 0;
    virtual const vec3& GetPosition() const  = 0;
    virtual vec3 GetNormal(const vec3 &hitpoint) const = 0;

    double radius;
    Color emission, color;
    ReflectionType refl;
};

} // namespace softbivpt

#endif // SOFTBIVPT_SHAPE_H_
