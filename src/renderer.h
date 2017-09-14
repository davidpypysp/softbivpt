#ifndef SOFTBIVPT_RENDERER_H_
#define SOFTBIVPT_RENDERER_H_

#include "ray.h"
#include "pt.h"
#include "image.h"

namespace softbivpt {

class Renderer {
public: 
	/* Initialize Renderer used for Rendering the image.
		* width_ Heigt: pixels of the image
		* samples_: # of samples used per pixel
		* enable_dof_: enable depth of field: Needs samples_ > 1
		* */
	Renderer(const int width, const int height, const int samples, const bool enable_dof = false) :
		width_(width), height_(height), samples_(samples), enable_dof_(enable_dof) {
		/* Set camera_ origin and viewing direction (negative z direction) */
		camera_ = Ray(vec3(0.0, 52.0, 295.6), vec3(0.0, -0.042612, -1.0).Normalized());

		/* Set depth of field variables aperture_, focal_length_ and number of samples */
		aperture_ = 4;
		focal_length_ = 160;

		/* Image edge vectors for pixel sampling */
		cx_ = vec3(width_ * 0.5135 / height_);
		cy_ = (cx_.Cross(camera_.dir)).Normalized() * 0.5135;
	}

	//Render the scene with the pathtracer p
	Image RenderScene(PathTracer *p) {
		Image img(width_, height_);

		/* Loop over image rows */
		int count_y = 0;
//#pragma omp parallel for
		for (int y = 0; y < height_; y++) {
			srand(y * y * y);

			/* Loop over row pixels */
			for (int x = 0; x < width_; x++) {
				img.SetColor(x, y, Color());

				/* 2x2 subsampling per pixel */
				for (int sy = 0; sy < 2; sy++) {
					for (int sx = 0; sx < 2; sx++) {
						Color accumulated_radiance = Color();

						/* Compute radiance at subpixel using multiple samples */
						for (int s = 0; s < samples_; s++) {
							const double r1 = 2.0 * drand48();
							const double r2 = 2.0 * drand48();

							/* Transform uniform into non-uniform filter samples */
							double dx;
							if (r1 < 1.0)
								dx = sqrt(r1) - 1.0;
							else
								dx = 1.0 - sqrt(2.0 - r1);

							double dy;
							if (r2 < 1.0)
								dy = sqrt(r2) - 1.0;
							else
								dy = 1.0 - sqrt(2.0 - r2);

							/* Ray direction into scene from camera_ through sample */
							vec3 dir = cx_ * ((x + (sx + 0.5 + dx) / 2.0) / width_ - 0.5) +
											cy_ * ((y + (sy + 0.5 + dy) / 2.0) / height_ - 0.5) +
											camera_.dir;

							/* Extend camera_ ray to start inside box */
							vec3 start = camera_.org + dir * 130.0;

							dir = dir.Normalized();

							Ray ray = Ray(start, dir);

							if(enable_dof_) {
								//DoF
								double u1 = (drand48() * 2.0) - 1.0;
								double u2 = (drand48() * 2.0) - 1.0;

								double fac = (double) (2 * M_PI * u2);

								vec3 offset = aperture_ * vec3(u1 * cos(fac), u1 * sin(fac), 0.0);
								vec3 focalPlaneIntersection = ray.org + ray.dir * (focal_length_ / camera_.dir.Dot(ray.dir));
								ray.org = ray.org + offset;
								ray.dir = (focalPlaneIntersection - ray.org).Normalized();

							}
								/* Accumulate radiance */
								accumulated_radiance = accumulated_radiance +
								p->CalculatePixelColor(ray) / samples_;// / dof_samples;
						}

						accumulated_radiance = accumulated_radiance.clamp() * 0.25;

						img.AddColor(x, y, accumulated_radiance);
					}
				}
			}
			float rate = 100. * (++count_y) / (height_-1);
			fprintf(stderr,"\rRendering (%d spp) %5.2f%%", samples_*4, rate <= 100. ? rate : 100.); 
		}

		return img;
	}

private:
	int width_, height_;
	int samples_;
	bool enable_dof_;
	Ray camera_;
	double aperture_;
	double focal_length_;
	vec3 cx_, cy_;

};

} // namespace softbivpt


/*
 * Global illumination via unbiased Monte Carlo path tracing
 *
 * Specular, Diffuse, and Glass BRDFs
 */

#endif // SOFTBIVPT_RENDERER_H_
