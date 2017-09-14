#ifndef SOFTBIVPT_IMAGE_H_
#define SOFTBIVPT_IMAGE_H_

#include <fstream>

#include "vector.h"
/*------------------------------------------------------------------
| Struct holds pixels/colors of rendered image
------------------------------------------------------------------*/


namespace softbivpt {

class Image {
public:
	int width, height;
	Color *pixels;

	Image(const int width, const int height) : width(width), height(height) {
		pixels = new Color[width * height];
	}

	Color GetColor(const int x, const int y) {
		int image_index = (height - y - 1) * width + x;
		return pixels[image_index];
	}

	void SetColor(const int x, const int y, const Color &c) {
		int image_index = (height - y - 1) * width + x;
		pixels[image_index] = c;
	}

	void AddColor(const int x, const int y, const Color &c) {
		int image_index = (height - y - 1) * width + x;
		pixels[image_index] = pixels[image_index] + c;
	}

	int ToInteger(double x) {
		/* Clamp to [0,1] */
		if (x < 0.0) x = 0.0;
		else if (x > 1.0) x = 1.0;

		/* Apply gamma correction and convert to integer */
		return int(pow(x, 1 / 2.2) * 255 + .5);
	}

	void Save(const std::string &filename) {
		/* Save image in PPM format */
		FILE *f = fopen(filename.c_str(), "wb");
		fprintf(f, "P3\n%d %d\n%d\n", width, height, 255);
		for (int i = 0; i < width * height; i++)
			fprintf(f, "%d %d %d ", ToInteger(pixels[i].x),
					ToInteger(pixels[i].y),
					ToInteger(pixels[i].z));
		fclose(f);
	}
};

} // namespace softbivpt


#endif // SOFTBIVPT_IMAGE_H_
