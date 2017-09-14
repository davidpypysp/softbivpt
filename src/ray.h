#ifndef SOFTBIVPT_RAY_H_
#define SOFTBIVPT_RAY_H_

#include "vector.h"

namespace softbivpt {

class Ray {
public:
    vec3 org, dir;

    /* Origin and direction */
    Ray(const vec3 org, const vec3 &dir) : org(org), dir(dir){}
	Ray() : org(vec3(0, 0, 0)), dir(vec3(0, 0, 0)) {}
};

} // namespace softbivpt

#endif // SOFTBIVPT_RAY_H_
