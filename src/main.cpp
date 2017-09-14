
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>

#include "renderer.h"
#include "shape.h"
#include "image.h"
#include "pt.h"
#include "bidir_pt.h"


/******************************************************************
* Main routine: Computation of path tracing image (2x2 subpixels)
* Key parameters
* - Image dimensions: width, height
* - Number of samples per subpixel (non-uniform filtering): samples
* Rendered result saved as PPM image file
*******************************************************************/
int main(int argc, char *argv[]) {
	std::string scene_filename = ""; 
	int samples = 1;
	time_t start, end;
	unsigned int light_bounces = 2;

	if (argc == 2) {
		samples = atoi(argv[1]);
	}
	else if (argc == 3) {
		samples = atoi(argv[1]);
		light_bounces = atoi(argv[2]);
	}
	else if (argc == 4) {
		samples = atoi(argv[1]);
		light_bounces = atoi(argv[2]);
		scene_filename = argv[3];
	}
	else {
		std::cout << "Paramaters: <Nr. of Subsamples> <LightSamples> <SceneName>"  << std::endl << "Usage Example: ./softbivpt.exe 16 4 scene1" << std::endl;
	}

    // volumetric setting
    double sigma_s = 0.012, sigma_a = 0.012;
    bool volumetric = false;
	std::string input;

	std::cout << "Volumetric path tracing? (y/n): ";
	std::cin >> input;
	if(input == "y") {
		volumetric = true;
	}

	softbivpt::PathTracer *pt = nullptr;
	std::cout << "Bidirectional path tracing? (y/n):";
	std::cin >> input;
	if(input == "y") {
		pt = new softbivpt::BidirPT(scene_filename, light_bounces, volumetric, sigma_s, sigma_a);
	}
	else {
		pt = new softbivpt::PathTracer(scene_filename, light_bounces, volumetric, sigma_s, sigma_a);
	}

	int width = 1024, height = 768;
	std::string result_name = "result.ppm";
	softbivpt::Renderer r(width, height, samples);
	std::cout << "Definition: " << std::to_string(width) << " * " << std::to_string(height) << std::endl;
	std::cout << "Rendering..." << std::endl;
	time(&start);
	r.RenderScene(pt).Save(result_name);
	time(&end);
	std::cout << "\nDone rendering to '" << result_name << "', took: " << difftime(end, start) << " seconds." << std::endl;
}
