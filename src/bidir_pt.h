#ifndef SOFTBIVPT_BIDIR_PT_H_
#define SOFTBIVPT_BIDIR_PT_H_


#include "pt.h"
#include "material_bdrf.h"
#include <map>

namespace softbivpt {

struct LightPath {
	vec3 hit_point;
	Color absorbed_color;
	int light_id;
	int object_id;

	LightPath() {}

	LightPath(const vec3 &hit_point, const Color &absorbed_color, const int light_id, const int object_id) :
		hit_point(hit_point), 
		absorbed_color(absorbed_color), 
		light_id(light_id), 
		object_id(object_id){
	}


};

class BidirPT : public PathTracer {
public:
	//functions
	BidirPT(const std::string &scene_name, unsigned int light_bounces, bool volumetric, double sigma_s, double sigma_a); //Jiyu
	BidirPT(const Scene &scene, unsigned int light_bounces);
	BidirPT(const std::string &scene_name);
	BidirPT(const Scene &scene);
	Color CalculatePixelColor(const Ray &ray) const;

private:
	//variables:
	//std::vector<Shape*> lightEmitters;
	std::map<int, Shape*> light_emitters;
	unsigned int no_lights;
	unsigned int light_bounces;

	//functions:
	void GetLightEmitters();
	std::vector<LightPath> TraceLightRays(const int bounces) const;
	Color Radiance(const Ray &ray, const std::vector<LightPath> &light_paths) const;
	vec3 UniformSampleSphere() const; 
	double DiffusePdf(const vec3 &nl, const vec3 &out_ray);
	/*
	vec3 diffuseBRDF(const vec3 & nl) const;
	vec3 glossyBRDF(const Ray & ray, const vec3 & nl) const;
	vec3 translBRDF(const Ray & ray, const vec3 & n, const vec3 & nl, Color & cf) const;
	vec3 refrBRDF(const Ray & ray, const vec3 & n, const vec3 & nl, Color & cf) const;
	vec3 specularBRDF(const Ray & ray, const vec3 & nl) const;
	*/
	bool CheckVisibility(const vec3 &p1, const vec3 &p2, int id) const;
	Color ShootShadowRay(const std::vector<LightPath> &light_paths, const Shape &object, const vec3 &hit_point, int id) const;
	Color ExplicitComputationOfDirectLight(const vec3 &hit_point, const Shape &object, const vec3 &nl) const;
};

} // namespace softbivpt

#endif // SOFTBIVPT_BIDIR_PT_H_