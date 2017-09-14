#include "bidir_pt.h"

namespace softbivpt {

/* constructor with scene name and the number of time the lightray should bounce through the scene */
BidirPT::BidirPT(const std::string &scene_name, unsigned int light_bounces,  bool volumetric, double sigma_s, double sigma_a) : 
	PathTracer(scene_name, true, volumetric, sigma_s, sigma_a),
	light_bounces(light_bounces) {
	GetLightEmitters();
}


/* constructor with the builded scene and the number of time the lightray should bounce through the scene */
BidirPT::BidirPT(const Scene & scene, unsigned int light_bounces) : 
	PathTracer(scene, true),
	light_bounces(light_bounces) {
	GetLightEmitters();
}

BidirPT::BidirPT(const std::string & scene_name) : 
	PathTracer(scene_name, true),
	light_bounces(2) {
	GetLightEmitters();
}


BidirPT::BidirPT(const Scene &scene) : 
	PathTracer(scene, true),
	light_bounces(2) {
	GetLightEmitters();
}

/* save all the light emitters of the scene in an array */
void BidirPT::GetLightEmitters() {
	//save all light emitters and determine their size
	for (int i = 0; i < objects_size_; i++) {
		Shape * cur_obj = my_scene_.GetShapes()[i];
		if (cur_obj->emission.x <= 0 && cur_obj->emission.y <= 0 && cur_obj->emission.z <= 0)
			continue; /* Skip objects that are not light sources */
		else {
			light_emitters[i] = cur_obj;
		}
	}
	no_lights = light_emitters.size();
}

/*  start tracing with path tracing */
Color BidirPT::CalculatePixelColor(const Ray &ray) const {
	std::vector<LightPath> all_light_paths = TraceLightRays(light_bounces);
	return Radiance(ray, all_light_paths);
}


/* Shoot a ray from every light in the scene
 * and let it bounce  off a few objects in the scene
 * the accumulated color, the hitted object are stored in the LightPath array 
 *
 * This method looks similar to the Radiance() function.
 * */
std::vector<LightPath> BidirPT::TraceLightRays(const int bounces) const
{
	std::vector<LightPath> light_paths;
	light_paths.reserve(bounces*no_lights);

	//for(int i = 0; i < no_lights; i++)
	std::map<int,Shape*>::const_iterator it;
	//c++11
	//for(const auto & light : light_emitters) 
	for(it = light_emitters.begin(); it != light_emitters.end(); it++) {
		//get current light
		Shape* cur_light = it->second;
		vec3 origin = cur_light->GetPosition();
		vec3 nl = cur_light->GetNormal(origin);

		//create Random direction vector on a sphere
		vec3 dir = UniformSampleSphere() ;
		//Shoot Ray from Lightsource with small eps
		Ray r = Ray(origin + dir * (cur_light->radius + 0.1), dir);
		double t;
		int id = 0;
		vec3 cf(1,1,1);
		vec3 cl(0,0,0);

		
		for(int j = 0; j < bounces; j++) {
			if(!Intersect(r, t, id)) {
				//that didn't hit anything, We should propably ignore it, but I'm not quite sure
				//light_paths[(no_lights*i) + j] = LightPath(vec3(0,0,0), Color(0,0,0));
				continue;
			}
			//
			//so we hit something, lets find out what it is
			const Shape &hit_object = *my_scene_.GetShapes()[id];	

			//where did we hit it?
			vec3 hit_point = r.org + r.dir *t;	

			//Normal of hitpoint
			vec3 n = hit_object.GetNormal(hit_point);
			vec3 nl = n;

			//flip it if we hit from within
			if (n.Dot(r.dir) >= 0)
				nl = nl * -1.0;

			//TODO Russian Roulette, it is biased right now
			Color col = hit_object.color;
			cf = cf.MultComponents(col);
			//cl = cl + cf.MultComponents(hit_object.emission);
			cl = cf.MultComponents(cur_light->emission);

			//save the hitpoint and the color of the light ray
			//and let it bounce again :)
			vec3 dir;
			if (hit_object.refl == DIFF) {
				light_paths.push_back(LightPath(hit_point, cl, it->first, id));
				dir = MaterialBRDF::DiffuseBRDF(nl);
			}
			else if (hit_object.refl == SPEC) {
				dir = MaterialBRDF::SpecularBRDF(r, nl);
			}
			else if(hit_object.refl == GLOSSY) {
				dir = MaterialBRDF::GlossyBRDF(r, nl);
			}
			else if(hit_object.refl == TRANSL) {
				dir = MaterialBRDF::TranslBRDF(r, n, nl, cf);
			}
			else if(hit_object.refl == REFR) {
				dir = MaterialBRDF::RefrBRDF(r, n, nl, cf);
			}
			else {
				//This case should not exist!!!
				dir = vec3(0,0,0);
			}

			r = Ray(hit_point, dir);
		}
	}

	return light_paths;
}

/* iterative version of the Path Tracing algorithm, the recursive version with more documentation is in PathTracer.cpp */
Color BidirPT::Radiance(const Ray &ray, const std::vector<LightPath> &light_paths) const
{
	int depth = 0;
	double t;	//distance to intersection
	int id = 0; //id of intersected object
	bool sample_lights = true; // if last hit was diffuse, don't sample lights
	Ray r = ray;
	Color cl(0,0,0); //accumulated color
	Color cf(1,1,1); //accumulated reflectance

	for(int i = 0;; i++) {
        //Jiyu
        double tnear = 0, tfar = 0, scale=1.0, absorption=1.0;
        bool intrsctmd = homogeneous_medium_->Intersect(r, &tnear, &tfar) > 0;
        if (volumetric_ && intrsctmd) {
            Ray sray;
            double s, ms = Scatter(r, &sray, tnear, tfar, s), prob_s = ms;
            scale = 1.0/(1.0-prob_s);
            if (drand48() <= prob_s) {// Sample surface or volume?
                if (!Intersect(r, t, id, tnear + s)) {
                    //return Radiance(sray, depth, sample_lights) * ms * (1.0 / prob_s);
                    cf = cf * ms * (1.0 / prob_s);
                    r = sray;
                    ++depth;
                    continue;
                }
                scale = 1.0;
            }
            else
            if (!Intersect(r, t, id)) return cl;
            if (t >= tnear) {
                double dist = (t > tfar ? tfar - tnear : t - tnear);
                absorption=exp(-sigma_t_ * dist);
            }
        }
        else
        ///////
		if (!Intersect(r, t, id))   /* No intersection with scene */
			return cl;

		const Shape &obj = *my_scene_.GetShapes()[id];

		vec3 hit_point = r.org + r.dir * t;	/* Intersection point */

		vec3 n = obj.GetNormal(hit_point);

		vec3 nl = n;
		/* Obtain flipped normal, if object hit from inside */
		if (n.Dot(r.dir) >= 0)
			nl = nl * -1.0;

		Color col = obj.color;
        Color Le = obj.emission; //Jiyu



		/* Maximum RGB reflectivity for Russian Roulette */
		double p = col.Max();

        //Jiyu
        if (n.Dot(nl)>0 || obj.refl != REFR) {col = col * absorption; Le = obj.emission * absorption;}// no absorption inside glass
        else scale=1.0;

        cf = cf * scale;

        ///////


		//when we didn't hit a diff surface save the color
		if(sample_lights || obj.refl != DIFF)
			cl = cl + cf.MultComponents(Le); //Jiyu
		if (++depth > 5 || !p) {  /* After 5 bounces or if max reflectivity is zero */
			if (drand48() < p)			  /* Russian Roulette */
				col = col * (1 / p);		/* Scale estimator to remain unbiased */
			else
				return cl;  /* No further bounces, only return potential emission */
		}



		cf = cf.MultComponents(col);



		//direction vec3, calculated with the BRDFs
		vec3 dir;

		if (obj.refl == DIFF) {
			dir = MaterialBRDF::DiffuseBRDF(nl);
			bool isNotLight = light_emitters.find(id) == light_emitters.end();
			if(!isNotLight && i == 0)
				cl = cl + cf.MultComponents(Le); //Jiyu
			else if(isNotLight || sample_lights) 
				cl = cl + cf.MultComponents(ExplicitComputationOfDirectLight(hit_point, obj, nl));
			sample_lights = false;
			cl = cl + cf.MultComponents(ShootShadowRay(light_paths, obj, hit_point, id));
		}

		else if (obj.refl == SPEC) {
			dir = MaterialBRDF::SpecularBRDF(r, nl);
			sample_lights = true;
		}
		else if(obj.refl == GLOSSY) {
			dir = MaterialBRDF::GlossyBRDF(r, nl);
			sample_lights = true;
		}
		else if(obj.refl == TRANSL) {
			dir = MaterialBRDF::TranslBRDF(r, n, nl, cf);
			sample_lights = true;
		}
		else if(obj.refl == REFR) {
			dir = MaterialBRDF::RefrBRDF(r, n, nl, cf);
			sample_lights = true;
		}
		else {
			//This case should not exist!!!
			dir = vec3(0,0,0);
			sample_lights = true;
		}

		r = Ray(hit_point, dir);
	}
}

/* Direct Light Sample method */
Color BidirPT::ExplicitComputationOfDirectLight(const vec3 &hit_point, const Shape &object, const vec3 &nl) const {
	vec3 e(0,0,0);	
	//for(int i = 0; i < no_lights; i++)
	std::map<int,Shape*>::const_iterator it;
	//c++11
	for(it = light_emitters.begin(); it != light_emitters.end(); it++) {
		const Shape &cur_light = *((it)->second);	
		/* Randomly sample spherical light source from surface intersection */
		/* Set up local orthogonal coordinate system su,sv,sw towards light source */
		vec3 cur_center = cur_light.GetPosition();
		vec3 sw = (cur_center - hit_point).Normalized(); 

		vec3 su;

		if (fabs(sw.x) > 0.1)
			su = vec3(0.0, 1.0, 0.0);
		else
			su = vec3(1.0, 0.0, 0.0);

		su = (sw.Cross(su)).Normalized();
		vec3 sv = sw.Cross(su);

		// next event estimation
		/* Create random sample direction l towards spherical light source */
		double cos_a_max = sqrt(1.0 - cur_light.radius * cur_light.radius /
									  (hit_point - cur_center).Dot(hit_point - cur_center));

		double eps1 = drand48();
		double eps2 = drand48();
		double cos_a = 1.0 - eps1 + eps1 * cos_a_max;
		double sin_a = sqrt(1.0 - cos_a * cos_a);
		double phi = 2.0 * M_PI * eps2;
		vec3 l = su * cos(phi) * sin_a +
				   sv * sin(phi) * sin_a +
				   sw * cos_a;
		l = l.Normalized();
		/* Shoot shadow ray, check if intersection is with light source */
		double t;
		int id = 0;
		if (Intersect(Ray(hit_point,l), t, id) && id == it->first) {
			double omega = 2.0 * M_PI * (1.0 - cos_a_max);
			/* Add diffusely reflected light from light source; note constant BRDF 1/PI */
			e = e + cur_light.emission * l.Dot(nl) * omega * M_1_PI;
		}
	}

	return e;
}

/* this function returns a directional vector distributed on a sphere*/
vec3 BidirPT::UniformSampleSphere() const {
	//random angle between 0 and 2pi
	double phi = drand48() * 2 * M_PI;
	//random angle between -1 and 1
	double costheta = 2 * drand48() - 1; 

	double theta = std::acos(costheta);

	return vec3( std::sin(theta) * std::cos(phi),
				 std::sin(theta) * std::sin(phi),
				 std::cos(theta));
}


/* returns the probability that diffuse material reflects in a ceratin direction
 * nl : the normal of the Material
 * outRay : the outgoing direction
 */
double BidirPT::DiffusePdf(const vec3 &nl, const vec3 &out_ray ) {
	return (nl.Dot(out_ray) / M_PI);
}



/* check if a point and an object see each other
 * no intersection between other things in the scene
 * p1: Point (used for ray origin)
 * p2: the object which is checked
 * id: the id of the object // bad design shouldn't be needed
 * */
bool BidirPT::CheckVisibility(const vec3 &p1, const vec3 &p2, int id) const {

	Ray r(p1, p2);
	int test_id = 0;
	//not needed...
	double d;
	if(Intersect(r, d, test_id)) {
		return (test_id == id);
	}
	//this should never happen, couse we shoot a ray to an object in the scene
	return false;
}

//TODO pdf for all materials


/* Shoot shadow ray for each hitobject from camera (and bounces of camera) to each Path of the Light Ray
 * light_paths: The path which was calculated by the traceLightRays() function
 * object: the object that was hit by a camera ray
 * hit_point: The point where the object was hit
 * id: the id where the object is stored in the scene
 * returns Color influence to Ray
 * */
Color BidirPT::ShootShadowRay(const std::vector<LightPath> &light_paths, const Shape &object, const vec3 &hit_point, int id) const {
	vec3 e(0,0,0);

	if(object.refl == DIFF) {
		for(int i = 0; i < light_paths.size(); i++) {
			//create directional vector between light hitpoint and camera hitpoint
			vec3 dir = hit_point - light_paths[i].hit_point ;
			vec3 dir_norm = dir.Normalized();
			//is there a shadowray?
			if(CheckVisibility(light_paths[i].hit_point, dir_norm, id)) {
				//get normal of enlightend Object
				const vec3 & enlighten_obj_normal = my_scene_.GetShapes()[light_paths[i].object_id]->GetNormal(light_paths[i].hit_point);

				//the diffuse PDF for the enlightend object
				double diff_enlight_pdf = std::max(0.0, dir_norm.Dot(enlighten_obj_normal)) * M_1_PI;
				//the diffuse PDF for the camera object
				double diff_eye_pdf = std::max(0.0,dir_norm.Dot(object.GetNormal(hit_point))) * M_1_PI;

				double distance_sqr = dir.Dot(dir);
				//First light constant which has to be added
				e = e + light_paths[i].absorbed_color * diff_enlight_pdf * diff_eye_pdf / distance_sqr;
				//e = e + (light_paths[i].absorbed_color * (dir.Dot(object.GetNormal(hit_point) * diffPdf))) / M_PI;

			}
			else {
				//Ignore if they don't see eachother
				continue;	
			}

		}
	}
	return e;
}

} // namespace softbivpt

