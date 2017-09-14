#ifndef SOFTBIVPT_PT_H_
#define SOFTBIVPT_PT_H_

#include "shape.h"
#include "scene.h"

namespace softbivpt {

class PathTracer {
public:
	//functions
	PathTracer(const std::string &scene_name, bool sample_light, bool volumetric = false, double sigma_s = 0.009, double sigma_a = 0.006); //Jiyu
	PathTracer(const Scene &scene, bool sample_light);
	virtual Color CalculatePixelColor(const Ray &ray) const;

	//Jiyu Peng
	bool volumetric_;
	Sphere *homogeneous_medium_ = nullptr;
	double sigma_s_, sigma_a_, sigma_t_;
	///////////

private:
	//functions:
	Color Radiance(const Ray &ray, int depth, int E) const;
	Color RefrBRDF(const Shape &obj, const Ray &ray, const vec3 &hitpoint, const vec3 &nl, int depth, Color f, Color Le, double scaleBy) const; //Jiyu
	Color TranslBRDF(const Shape &obj, const Ray & ray, const vec3 & hitpoint, const vec3 & nl, int depth, Color f, Color Le, double scaleBy) const; //Jiyu
	Color GlossyBRDF(const Shape &obj, const Ray & ray, const vec3 & hitpoint, const vec3 & nl, int depth, int E, Color f, Color Le, double scaleBy) const; //Jiyu
	Color DiffuseBRDF(const Shape &obj, const Ray & ray, const vec3 & hitpoint, const vec3 & nl, int depth, int E, Color f, Color Le, double scaleBy) const; //Jiyu
	Color SpecularBRDF(const Shape &obj, const Ray & ray, const vec3 & hitpoint, const vec3 & nl, int depth, Color f, Color Le, double scaleBy) const; //Jiyu
	//
	bool sample_light_;

protected:
	//variables:
	Scene my_scene_;
	unsigned int objects_size_;
	//functions:
	bool Intersect(const Ray &ray, double &t, int &id, double tmax = 1e20) const;

	//Jiyu
	double Scatter(const Ray &r, Ray *sray, double tin, float tout, double &s) const; //Jiyu
	double SampleSegment(double epsilon, float sigma, float smax) const;
	vec3 SampleHG(double g, double e1, double e2) const;
	vec3 SampleSphere(double e1, double e2);
	//

	void GenerateOrthoBasis(vec3 &u, vec3 &v, vec3 w) const;
};

} // namespace softbivpt

#endif //  SOFTBIVPT_PT_H_
