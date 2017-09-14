#ifndef SOFTBIVPT_SCENE_H_
#define SOFTBIVPT_SCENE_H_

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>

#include "sphere.h"
#include "triangle.h"
#include "ray.h"

namespace softbivpt {

class Scene {
private:
    std::vector<Shape*> scene_shapes;

public:
    const std::vector<Shape*>& GetShapes() const;
    void LoadScene(const std::string &filename, const std::string &matfiles);
    void AddSphere(Sphere *new_sphere);
};

} // namespace softbivpt

#endif // SOFTBIVPT_SCENE_H_
