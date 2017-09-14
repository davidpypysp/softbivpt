#ifndef SOFTBIVPT_SCENE_READER_H_
#define SOFTBIVPT_SCENE_READER_H_

#include "scene.h"
#include "sphere.h"

#include <iostream>
#include <fstream>
#include <string>

namespace softbivpt {

/*
Read .sce file
*/

class SceneReader {
public:
	void ReadScene(Scene &scene, const std::string &name) {
		std::ifstream f(name);
		std::cout << "Loading " << name << "..." << std::endl;
		if(f.is_open()) {
			int i = 0;
			while ( !f.eof() ) {
				i++;
				double radius;
				double posx, posy, posz;
				double emir, emig, emib;
				double colr, colg, colb;
				std::string reflection;

				f >> radius;
				f >> posx;
				f >> posy;
				f >> posz;
				f >> emir;
				f >> emig;
				f >> emib;
				f >> colr;
				f >> colg;
				f >> colb;
				f >> reflection;

				ReflectionType refl;
				if(reflection.compare("DIFF") == 0) refl = DIFF;
				else if(reflection.compare("SPEC") == 0) refl = SPEC;
				else if(reflection.compare("REFR") == 0) refl = REFR;
				else if(reflection.compare("GLOSSY") == 0) refl = GLOSSY;
				else if(reflection.compare("TRANSL") == 0) refl = TRANSL;
				else refl = DIFF;

				Sphere *sphere = new Sphere(radius, vec3(posx,posy,posz), vec3(emir, emig, emib), vec3(colr, colg, colb), refl);
				scene.AddSphere(sphere);
				int tmp;
				//fix for reading
			//	f >> tmp;
			}
			f.close();
			std::cout << "# of spheres: " << i-1 << std::endl;
		}
	}
};

} // namespace softbivpt

#endif // SOFTBIVPT_SCENE_READER_H_
