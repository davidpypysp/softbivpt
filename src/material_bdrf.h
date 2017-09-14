#ifndef SOFTBIVPT_MATERIAL_BDRF_H_
#define SOFTBIVPT_MATERIAL_BDRF_H_

#include <stdlib.h>

#include "vector.h"
#include "ray.h"

namespace softbivpt {

class MaterialBRDF {
public:
	static vec3 DiffuseBRDF(const vec3 &nl) {
		/* Compute random reflection vector on hemisphere */
		double r1 = 2.0 * M_PI * drand48();
		double r2 = drand48();
		double r2s = sqrt(r2);

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

		/* Return potential light emission, direct lighting, and indirect lighting (via
			recursive call for Monte-Carlo integration */
		//return obj.emission * E + e + col.MultComponents(Radiance(Ray(hitpoint, d), depth, 0));
		return d;
	}

	static vec3 GlossyBRDF(const Ray &ray, const vec3 &nl) {
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

		return d;
	}
	
	static vec3 TranslBRDF(const Ray &ray, const vec3 &n, const vec3 &nl, Color &cf) {
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
		
		/* Otherwise object transparent, i.e. assumed dielectric glass material */
		bool into = n.Dot(nl) > 0;		  /* Bool for checking if ray from outside going in */
		double nc = 1;						  /* Index of refraction of air (approximately) */
		double nt = 1.5;					  /* Index of refraction of glass (approximately) */
		double nnt;

		double Re, RP, TP, Tr;
		vec3 tdir;
		
		if (into)	   /* Set ratio depending on hit from inside or outside */
			nnt = nc / nt;
		else
			nnt = nt / nc;
		
		double ddn = ray.dir.Dot(nl);
		double cos2t = 1 - nnt * nnt * (1 - ddn * ddn);
		
		/* Check for total internal reflection, if so only reflect */
		if (cos2t < 0) return glos_refl_d;
		
		/* Otherwise reflection and/or refraction occurs */

		//create Translucent refraction vector
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
			tdir = (transl_d * nnt - n * (ddn * nnt + sqrt(cos2t))).Normalized();
		else
			tdir = (transl_d * nnt + n * (ddn * nnt + sqrt(cos2t))).Normalized();

		/* Determine R0 for Schlick's approximation */
		double a = nt - nc;
		double b = nt + nc;
		double R0 = a * a / (b * b);

		/* Cosine of correct angle depending on outside/inside */
		double c;
		if (into)
			c = 1 + ddn;
		else
			c = 1 - tdir.Dot(n);

		/* Compute Schlick's approximation of Fresnel equation */
		Re = R0 + (1 - R0) * c * c * c * c * c;	 /* Reflectance */
		Tr = 1 - Re;						/* Transmittance */

		/* Probability for selecting reflectance or transmittance */
		double P = .25 + .5 * Re;
		RP = Re / P;			/* Scaling factors for unbiased estimator */
		TP = Tr / (1 - P);

		if (drand48() < P) {
			cf = cf * (RP);
			return glos_refl_d;
		}
		
		cf = cf * (TP);
		return  tdir;
	}

	static vec3 RefrBRDF(const Ray &ray, const vec3 &n, const vec3 &nl, Color &cf) {

		/* Object is transparent, i.e. assumed dielectric glass material */
		vec3 reflectionV =  ray.dir - n * 2 * n.Dot(ray.dir);	/* Prefect reflection */
		bool into = n.Dot(nl) > 0;		  /* Bool for checking if ray from outside going in */
		double nc = 1;						  /* Index of refraction of air (approximately) */
		double nt = 1.5;					  /* Index of refraction of glass (approximately) */
		double nnt;

		double Re, RP, TP, Tr;
		vec3 tdir;

		if (into)	   /* Set ratio depending on hit from inside or outside */
			nnt = nc / nt;
		else
			nnt = nt / nc;

		double ddn = ray.dir.Dot(nl);
		double cos2t = 1 - nnt * nnt * (1 - ddn * ddn);

		/* Check for total internal reflection, if so only reflect */
		if (cos2t < 0) return reflectionV;
		/* Otherwise reflection and/or refraction occurs */

		/* Determine transmitted ray direction for refraction */
		if (into)
			tdir = (ray.dir * nnt - n * (ddn * nnt + sqrt(cos2t))).Normalized();
		else
			tdir = (ray.dir * nnt + n * (ddn * nnt + sqrt(cos2t))).Normalized();

		/* Determine R0 for Schlick's approximation */
		double a = nt - nc;
		double b = nt + nc;
		double R0 = a * a / (b * b);

		/* Cosine of correct angle depending on outside/inside */
		double c;
		if (into)
			c = 1 + ddn;
		else
			c = 1 - tdir.Dot(n);

		/* Compute Schlick's approximation of Fresnel equation */
		Re = R0 + (1 - R0) * c * c * c * c * c;	 /* Reflectance */
		Tr = 1 - Re;						/* Transmittance */

		/* Probability for selecting reflectance or transmittance */
		double P = .25 + .5 * Re;
		RP = Re / P;			/* Scaling factors for unbiased estimator */
		TP = Tr / (1 - P);

		if (drand48() < P) {
			cf = cf * (RP);
			return reflectionV;
		}
		
		cf = cf * (TP);
		return tdir;
	}

	static vec3 SpecularBRDF(const Ray & ray, const vec3 & nl) {
		return (ray.dir - nl * 2 * nl.Dot(ray.dir));
	}

};

} // namespace softbivpt


#endif // SOFTBIVPT_MATERIAL_BDRF_H_