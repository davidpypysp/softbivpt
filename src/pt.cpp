#include "pt.h"
#include "scene_reader.h"

namespace softbivpt {

PathTracer::PathTracer(const std::string &scene_name, bool sample_light, bool volumetric, double sigma_s, double sigma_a) :
	sample_light_(sample_light),
	volumetric_(volumetric),
	sigma_s_(sigma_s),
	sigma_a_(sigma_a) { 

	if(scene_name != "") { // if set scene_name, load scene from file
		my_scene_.LoadScene(std::string("../resource/" + scene_name + ".obj"), std::string("../resource/"));
		SceneReader s;
		s.ReadScene(my_scene_, std::string("../resource/" + scene_name + ".sce"));
	}
	else {
		Sphere *sp0 = new Sphere(1e5, vec3( 1e5-50,40.8,81.6), vec3(),vec3(.75,.25,.25),DIFF);//Left
		Sphere *sp1 = new Sphere(1e5, vec3(-1e5+50,40.8,81.6),vec3(),vec3(.25,.25,.75),DIFF);//Rght
		Sphere *sp2 = new Sphere(1e5, vec3(0,40.8, 1e5),     vec3(),vec3(.75,.75,.75),DIFF);//Back
		Sphere *sp3 = new Sphere(1e5, vec3(0,40.8,-1e5+170), vec3(),vec3(),           DIFF);//Frnt
		Sphere *sp4 = new Sphere(1e5, vec3(0, 1e5, 81.6),    vec3(),vec3(.75,.75,.75),DIFF);//Botm
		Sphere *sp5 = new Sphere(1e5, vec3(0,-1e5+81.6,81.6),vec3(),vec3(.75,.75,.75),DIFF);//Top
		Sphere *sp6 = new Sphere(16.5,vec3(-23,16.5,47),       vec3(),vec3(1,1,1)*.999, SPEC);//Mirr
		Sphere *sp7 = new Sphere(16.5,vec3(23,16.5,78),       vec3(),vec3(1,1,1)*.999, REFR);//Glas
		Sphere *sp8 = new Sphere(1.5, vec3(0,81.6-16.5,81.6),vec3(4,4,4)*100,  vec3(), DIFF);//Lite
		my_scene_.AddSphere(sp0);
		my_scene_.AddSphere(sp1);
		my_scene_.AddSphere(sp2);
		my_scene_.AddSphere(sp3);
		my_scene_.AddSphere(sp4);
		my_scene_.AddSphere(sp5);
		my_scene_.AddSphere(sp6);
		my_scene_.AddSphere(sp7);
		my_scene_.AddSphere(sp8);
	}
/*
	Sphere *sp0 = new Sphere(16.5,	vec3(27, 16.5, -10), vec3(), vec3(1.0, 1.0, 1.0) * .999, REFR);
	Sphere *sp1 = new Sphere(16.5,  vec3(-20, 16.5, 10), vec3(), vec3(1.0, 1.0, 1.0) * .999, REFR);
	Sphere *sp2 = new Sphere(8.5,  vec3(-20, 8.5, 25), vec3(), vec3(1.0, 1.0, 0.0) * .999, DIFF);
	Sphere *sp3 = new Sphere(2.0, vec3(20, 50, -15), vec3(80,80,80), vec3(1.0,1.0,1.0) * .999, DIFF);
	my_scene_.add_sphere(sp0);
	my_scene_.add_sphere(sp1);
	my_scene_.add_sphere(sp2);
	my_scene_.add_sphere(sp3);
*/


    //Sphere *medium = new Sphere(300, vec3(0,50,80), vec3(), vec3(), DIFF);
    Sphere *medium = new Sphere(300, vec3(50,50,80), vec3(), vec3(), DIFF);

    homogeneous_medium_ = medium;
    sigma_t_ = sigma_s+sigma_a;
    /////////////
	objects_size_ = my_scene_.GetShapes().size();
}


PathTracer::PathTracer(const Scene &scene, bool sample_light) : 
	my_scene_(scene),
	sample_light_(sample_light) {
	objects_size_ = scene.GetShapes().size();
	SceneReader s;
	s.ReadScene(my_scene_, "../data/scene1.sce");

    //Jiyu
    Sphere *medium = new Sphere(300, vec3(50,50,80), vec3(), vec3(), DIFF);
    homogeneous_medium_ = medium;
    sigma_s_ = 0.01, sigma_a_ = 0.01, sigma_t_ = sigma_s_ + sigma_a_; //default: sigma_s = 0.009, sigma_a = 0.006
    //////////
}


Color PathTracer::CalculatePixelColor(const Ray &ray) const {
	return Radiance(ray, 0, 1);
}


/******************************************************************
* Recursive path tracing for computing radiance via Monte-Carlo
* integration; only considers perfectly diffuse, specular or
* transparent materials;
* after 5 bounces Russian Roulette is used to possibly terminate rays;
* emitted light from light source only included on first direct hit
* (possibly via specular reflection, refraction), controlled by
* parameter E = 0/1;
* on diffuse surfaces light sources are explicitely sampled;
* for transparent objects, Schlick's approximation is employed;
* for first 3 bounces obtain reflected and refracted component,
* afterwards one of the two is chosen randomly
*******************************************************************/
Color PathTracer::Radiance(const Ray &ray, int depth, int E) const {
	depth++;

	double t;
	int id = 0;

	//Jiyu
	double tnear = 0, tfar = 0, scaleBy=1.0, absorption=1.0;
	bool intrsctmd = homogeneous_medium_->Intersect(ray, &tnear, &tfar) > 0;
	if (intrsctmd) {
		Ray sRay;
		double s, ms = Scatter(ray, &sRay, tnear, tfar, s), prob_s = ms;
		scaleBy = 1.0/(1.0-prob_s);
		if (drand48() <= prob_s) {// Sample surface or volume?
			if (!Intersect(ray, t, id, tnear + s))
				return Radiance(sRay, depth, E) * ms * (1.0/prob_s);
			scaleBy = 1.0;
		}
		else
		if (!Intersect(ray, t, id)) return Color(0,0,0);
		if (t >= tnear) {
			double dist = (t > tfar ? tfar - tnear : t - tnear);
			absorption=exp(-sigma_t_ * dist);
		}
	}
	else
	///////
	if (!Intersect(ray, t, id))   /* No intersection with scene */
		return Color(0,0,0);

	const Shape &obj = *my_scene_.GetShapes()[id];

	vec3 hitpoint = ray.org + ray.dir * t;	/* Intersection point */

	vec3 nl = obj.GetNormal(hitpoint);
	vec3 n = nl; //Jiyu
	/* Obtain flipped normal, if object hit from inside */
	if (nl.Dot(ray.dir) >= 0)
		nl = nl * -1.0;

	Color col = obj.color;
	Color Le = obj.emission; //Jiyu

	/* Maximum RGB reflectivity for Russian Roulette */
	double p = col.Max();

	if (depth > 5 || !p) {  /* After 5 bounces or if max reflectivity is zero */
		if (drand48() < p)			  /* Russian Roulette */
			col = col * (1 / p);		/* Scale estimator to remain unbiased */
		else
			return obj.emission * E;  /* No further bounces, only return potential emission */
	}

	//Jiyu
	if (n.Dot(nl)>0 || obj.refl != REFR) {col = col * absorption; Le = obj.emission * absorption;}// no absorption inside glass
    else scaleBy=1.0;
	///////

	if (obj.refl == DIFF) {
		return DiffuseBRDF(obj, ray, hitpoint, nl, depth, E, col, Le, scaleBy); //Jiyu
	}
	else if (obj.refl == SPEC) {
		return SpecularBRDF(obj, ray, hitpoint, nl, depth, col, Le, scaleBy); //Jiyu
	}
	else if(obj.refl == GLOSSY) {
		return GlossyBRDF(obj, ray, hitpoint, nl, depth, E, col, Le, scaleBy); //Jiyu
	}
	else if(obj.refl == TRANSL) {
		return TranslBRDF(obj, ray, hitpoint, nl, depth, col, Le, scaleBy); //Jiyu
	}
	else if(obj.refl == REFR) {
		return RefrBRDF(obj, ray, hitpoint, nl, depth, col, Le, scaleBy); //Jiyu
	}
	
	return Color(0,0,0);

}



/******************************************************************
* Check for closest intersection of a ray with the scene;
* returns true if intersection is found, as well as ray parameter
* of intersection and id of intersected object
*******************************************************************/
bool PathTracer::Intersect(const Ray &ray, double &t, int &id, double tmax) const //Jiyu Peng
{
	//t = 1e20;
	double inf = t = tmax; //Jiyu
	for (int i = 0; i < objects_size_; i++)
	{
		double d = my_scene_.GetShapes()[i]->Intersect(ray);
		if (d > 0.0 && d < t)
		{
			t = d;
			id = i;
		}
	}
	return t < inf; //Jiyu
}

//Jiyu
double PathTracer::SampleSegment(double epsilon, float sigma, float smax) const {
	return -log(1.0 - epsilon * (1.0 - exp(-sigma * smax))) / sigma;
}
vec3 PathTracer::SampleSphere(double e1, double e2) {
	double z = 1.0 - 2.0 * e1, sint = sqrt(1.0 - z * z);
	return vec3(cos(2.0 * M_PI * e2) * sint, sin(2.0 * M_PI * e2) * sint, z);
}
vec3 PathTracer::SampleHG(double g, double e1, double e2)const {
	//double s=2.0*e1-1.0, f = (1.0-g*g)/(1.0+g*s), cost = 0.5*(1.0/g)*(1.0+g*g-f*f), sint = sqrt(1.0-cost*cost);
	double s = 1.0-2.0*e1, cost = (s + 2.0*g*g*g * (-1.0 + e1) * e1 + g*g*s + 2.0*g*(1.0 - e1+e1*e1))/((1.0+g*s)*(1.0+g*s)), sint = sqrt(1.0-cost*cost);
	return vec3(cos(2.0 * M_PI * e2) * sint, sin(2.0 * M_PI * e2) * sint, cost);
}

void PathTracer::GenerateOrthoBasis(vec3 &u, vec3 &v, vec3 w)const {
	vec3 coVec = w;
	if (fabs(w.x) <= fabs(w.y))
		if (fabs(w.x) <= fabs(w.z)) coVec = vec3(0,-w.z,w.y);
		else coVec = vec3(-w.y,w.x,0);
	else if (fabs(w.y) <= fabs(w.z)) coVec = vec3(-w.z,0,w.x);
	else coVec = vec3(-w.y,w.x,0);
	coVec.Normalized();
	u = w.Cross(coVec),
	v = w.Cross(u);
}

double PathTracer::Scatter(const Ray &r, Ray *sray, double tin, float tout, double &s) const {
	s = SampleSegment(drand48(), sigma_s_, tout - tin);
	vec3 x = r.org + r.dir *tin + r.dir * s;
	//Vec dir = SampleSphere(drand48(), drand48()); //Sample a direction ~ uniform phase function
	vec3 dir = SampleHG(-0.5, drand48(), drand48()); //Sample a direction ~ Henyey-Greenstein's phase function
	vec3 u,v;
	GenerateOrthoBasis(u,v,r.dir);
	dir = u*dir.x+v*dir.y+r.dir*dir.z;
	if (sray)	*sray = Ray(x, dir);
	return (1.0 - exp(-sigma_s_ * (tout - tin)));
}
/////////////

Color PathTracer::DiffuseBRDF(const Shape &obj, const Ray & ray, const vec3 & hitpoint, const vec3 & nl, int depth, int E, Color f, Color Le, double scaleBy) const {
	/* Compute random reflection vector on hemisphere */
	double r1 = 2.0 * M_PI * drand48();
	double r2 = drand48();
	double r2s = sqrt(r2);

	Color col = obj.color;
	vec3 e(0,0,0);

	/* Set up local orthogonal coordinate system u,v,w on surface */
	vec3 w = nl;
	vec3 u;

	if (fabs(w.x) > .1)
		u = vec3(0.0, 1.0, 0.0).Cross(w).Normalized();
	else
		u = (vec3(1.0, 0.0, 0.0).Cross(w)).Normalized();

	vec3 v = w.Cross(u);

	/* Random reflection vector d */
	vec3 d = (u * cos(r1) * r2s +
				v * sin(r1) * r2s +
				w * sqrt(1 - r2)).Normalized();

	if(sample_light_) {
		/* Explicit computation of direct lighting */
		for (int i = 0; i < objects_size_; i++) {
			const Shape &cur_obj = *my_scene_.GetShapes()[i];
			if (cur_obj.emission.x <= 0 && cur_obj.emission.y <= 0 && cur_obj.emission.z <= 0)
				continue; /* Skip objects that are not light sources */

			/* Randomly sample spherical light source from surface intersection */

			/* Set up local orthogonal coordinate system su,sv,sw towards light source */
			vec3 cur_cent = cur_obj.GetPosition();
			vec3 sw = (cur_cent - hitpoint).Normalized(); 

			vec3 su;

			if (fabs(sw.x) > 0.1)
				su = vec3(0.0, 1.0, 0.0);
			else
				su = vec3(1.0, 0.0, 0.0);

			su = (sw.Cross(su)).Normalized();
			vec3 sv = sw.Cross(su);

			// next event estimation
			/* Create random sample direction l towards spherical light source */
			double cos_a_max = sqrt(1.0 - cur_obj.radius * cur_obj.radius /
										  (hitpoint - cur_cent).Dot(hitpoint - cur_cent));

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
			if (Intersect(Ray(hitpoint,l), t, id) && id==i) {
				double omega = 2 * M_PI * (1 - cos_a_max);

				/* Add diffusely reflected light from light source; note constant BRDF 1/PI */
				e = e + cur_obj.emission * l.Dot(nl) * omega * M_1_PI;
			}
		}
		return (Le * E + f.MultComponents(e) + f.MultComponents(Radiance(Ray(hitpoint, d), depth, 0))) * scaleBy;
	}
	else
		return (Le + f.MultComponents(Radiance(Ray(hitpoint, d), depth, 0))) * scaleBy;

	
	/* Return potential light emission, direct lighting, and indirect lighting (via
	   recursive call for Monte-Carlo integration */
}

Color PathTracer::GlossyBRDF(const Shape &obj, const Ray &ray, const vec3 &hitpoint, const vec3 &nl, int depth, int E, Color f, Color Le, double scaleBy) const {
	//cosine distribution??
	const double r1 = 2.0 * M_PI * drand48();
	const double r2 = pow(drand48(), 1.0 / (100.0 + 1.0));
	const double r2s = sqrt(1.0 - r2 * r2);

	//same as above
	vec3 sw = (ray.dir - nl * 2.0 * nl.Dot(ray.dir)).Normalized();
	vec3 su;
	if (fabs(sw.x) > 0.1)
		su = vec3(0.0, 1.0, 0.0);
	else
		su = vec3(1.0, 0.0, 0.0);

	su = su.Cross(sw).Normalized();

	vec3 sv = sw.Cross(su);

	vec3 d = (su * cos(r1) * r2s +
				sv * sin(r1) * r2s +
				sw * r2).Normalized();

	double orientation = nl.Dot(d);

	//if ray is beneath surface , reflect the resulting ray around the perfect mirror ray
	if(orientation < 0)
		d = d - sw * 2.0 * sw.Dot(d);

	Color col = obj.color;

	return (Le + f.MultComponents(Radiance(Ray(hitpoint, d), depth, E))) * scaleBy;
}

Color PathTracer::TranslBRDF(const Shape &obj, const Ray & ray, const vec3 & hitpoint, const vec3 & nl, int depth, Color f, Color Le, double scaleBy) const {
	//cosine distribution??
	const double r1 = 2.0 * M_PI * drand48();
	//the exponent is the Transluency Factor, should be changable
	const double r2 = pow(drand48(), 1.0 / (1.0 + 100.0));
	const double r2s = sqrt(1.0 - r2 * r2);

	//same as above
	vec3 sw_g = (ray.dir - nl * 2.0 * nl.Dot(ray.dir)).Normalized();
	vec3 su_g;
	if (fabs(sw_g.x) > 0.1)
		su_g = vec3(0.0, 1.0, 0.0);
	else
		su_g = vec3(1.0, 0.0, 0.0);

	su_g = su_g.Cross(sw_g).Normalized();

	vec3 sv_g = sw_g.Cross(su_g);
	vec3 glos_refl_d = (su_g * cos(r1) * r2s +
				sv_g * sin(r1) * r2s +
				sw_g * r2).Normalized();
	
	vec3 obj_normal = obj.GetNormal(hitpoint);
	
	/* Otherwise object transparent, i.e. assumed dielectric glass material */
	Ray reflRay = Ray(hitpoint, glos_refl_d);
	bool into = obj_normal.Dot(nl) > 0;		  /* Bool for checking if ray from outside going in */
	double nc = 1;						  /* Index of refraction of air (approximately) */
	double nt = 1.5;					  /* Index of refraction of glass (approximately) */
	double nnt;

	Color col = obj.color;
	double Re, RP, TP, Tr;
	vec3 tdir;
	
	if (into)	   /* Set ratio depending on hit from inside or outside */
		nnt = nc / nt;
	else
		nnt = nt / nc;
	
	double ddn = ray.dir.Dot(nl);
	double cos2t = 1 - nnt * nnt * (1 - ddn * ddn);
	
	/* Check for total internal reflection, if so only reflect */
	if (cos2t < 0) {
		return (Le + f.MultComponents(Radiance(reflRay, depth, 1))) * scaleBy;
	}
	
	/* Otherwise reflection and/or refraction occurs */

	//create Translucent refraction vec3
	const double r1p = 2.0 * M_PI * drand48();
	const double r2p= pow(drand48(), 1.0 / (1.0 + 100.0));
	const double r2sp = sqrt(1.0 - r2 * r2);

	vec3 sw_p = (ray.dir);
	vec3 su_p;
	if (fabs(sw_p.x) > 0.1)
		su_p = vec3(0.0, 1.0, 0.0);
	else
		su_p = vec3(1.0, 0.0, 0.0);

	su_p = su_p.Cross(sw_p).Normalized();

	vec3 sv_p = sw_p.Cross(su_p);
	vec3 transl_d = (su_p * cos(r1) * r2sp +
				sv_p * sin(r1p) * r2sp +
				sw_p * r2p).Normalized();


	/* Determine transmitted ray direction for refraction */
	if (into)
		tdir = (transl_d * nnt - obj_normal * (ddn * nnt + sqrt(cos2t))).Normalized();
	else
		tdir = (transl_d * nnt + obj_normal * (ddn * nnt + sqrt(cos2t))).Normalized();

	/* Determine R0 for Schlick's approximation */
	double a = nt - nc;
	double b = nt + nc;
	double R0 = a * a / (b * b);

	/* Cosine of correct angle depending on outside/inside */
	double c;
	if (into)
		c = 1 + ddn;
	else
		c = 1 - tdir.Dot(obj_normal);

	/* Compute Schlick's approximation of Fresnel equation */
	Re = R0 + (1 - R0) * c * c * c * c * c;	 /* Reflectance */
	Tr = 1 - Re;						/* Transmittance */

	/* Probability for selecting reflectance or transmittance */
	double P = .25 + .5 * Re;
	RP = Re / P;			/* Scaling factors for unbiased estimator */
	TP = Tr / (1 - P);

	if (depth < 3) {
		/* Initially both reflection and trasmission */
		return (Le + f.MultComponents(Radiance(reflRay, depth, 1) * Re +
												 Radiance(Ray(hitpoint, tdir), depth, 1) * Tr)) * scaleBy;
	}
	else if (drand48() < P) {
		return (Le + f.MultComponents(Radiance(reflRay, depth, 1) * RP)) * scaleBy;
	}
	
	return (Le + f.MultComponents(Radiance(Ray(hitpoint, tdir), depth, 1) * TP)) * scaleBy;
}


Color PathTracer::RefrBRDF(const Shape &obj, const Ray & ray, const vec3 & hitpoint, const vec3 & nl, int depth, Color f, Color Le, double scaleBy) const {
	vec3 obj_normal = obj.GetNormal(hitpoint);

	/* Otherwise object transparent, i.e. assumed dielectric glass material */
	Ray reflRay = Ray(hitpoint, ray.dir - obj_normal * 2 * obj_normal.Dot(ray.dir));	/* Prefect reflection */
	bool into = obj_normal.Dot(nl) > 0;		  /* Bool for checking if ray from outside going in */
	double nc = 1;						  /* Index of refraction of air (approximately) */
	double nt = 1.5;					  /* Index of refraction of glass (approximately) */
	double nnt;

	Color col = obj.color;
	double Re, RP, TP, Tr;
	vec3 tdir;

	if (into)	   /* Set ratio depending on hit from inside or outside */
		nnt = nc / nt;
	else
		nnt = nt / nc;

	double ddn = ray.dir.Dot(nl);
	double cos2t = 1 - nnt * nnt * (1 - ddn * ddn);

	/* Check for total internal reflection, if so only reflect */
	if (cos2t < 0) {
		return (Le + f.MultComponents(Radiance(reflRay, depth, 1))) * scaleBy;
	}
	/* Otherwise reflection and/or refraction occurs */

	/* Determine transmitted ray direction for refraction */
	if (into)
		tdir = (ray.dir * nnt - obj_normal * (ddn * nnt + sqrt(cos2t))).Normalized();
	else
		tdir = (ray.dir * nnt + obj_normal * (ddn * nnt + sqrt(cos2t))).Normalized();

	/* Determine R0 for Schlick�s approximation */
	double a = nt - nc;
	double b = nt + nc;
	double R0 = a * a / (b * b);

	/* Cosine of correct angle depending on outside/inside */
	double c;
	if (into)
		c = 1 + ddn;
	else
		c = 1 - tdir.Dot(obj_normal);

	/* Compute Schlick�s approximation of Fresnel equation */
	Re = R0 + (1 - R0) * c * c * c * c * c;	 /* Reflectance */
	Tr = 1 - Re;						/* Transmittance */

	/* Probability for selecting reflectance or transmittance */
	double P = .25 + .5 * Re;
	RP = Re / P;			/* Scaling factors for unbiased estimator */
	TP = Tr / (1 - P);

	if (depth < 3) {
		/* Initially both reflection and trasmission */
		return (Le + f.MultComponents(Radiance(reflRay, depth, 1) * Re +
												 Radiance(Ray(hitpoint, tdir), depth, 1) * Tr)) * scaleBy;
	}
	else if (drand48() < P) {
		return (Le + f.MultComponents(Radiance(reflRay, depth, 1) * RP)) * scaleBy;
	}
	
	return (Le + f.MultComponents(Radiance(Ray(hitpoint, tdir), depth, 1) * TP)) * scaleBy;
}

Color PathTracer::SpecularBRDF(const Shape &obj, const Ray & ray, const vec3 & hitpoint, const vec3 & nl, int depth, Color f, Color Le, double scaleBy) const {
	Color col = obj.color;
	/* Return light emission mirror reflection (via recursive call using perfect reflection vector) */
	return (Le + f.MultComponents(Radiance(Ray(hitpoint, ray.dir - nl * 2 * nl.Dot(ray.dir)), depth, 1))) * scaleBy;
}


} // namespace softbivpt