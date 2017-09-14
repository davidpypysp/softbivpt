# SoftBiVpt

SoftBiVpt is a tiny cpu-based C++ ray tracer for the computer graphic project:  
*Bidirectional Path Tracing for Rendering Participating Media*

Basically, it uses bidirectional path tracing algorithm of Monte Carlo method to do ray tracing. Also, it combined with volumetric path tracing Extension to produce light through homogeneous medium rendering. It would simulate inaccurate rendering of natural phenomena such as fog and smoke.

This project is based on smallpt -- 99 lines open source C++ code of path tracer implementation. 

## Rendering Results

* Bidirectional path tracing  

![bidir_pt](https://raw.githubusercontent.com/davidpypysp/softbivpt/master/files/bidirpt.png)

* Volumetric path tracing with bi-dir 

![vpt](https://raw.githubusercontent.com/davidpypysp/softbivpt/master/files/vpt.png)

* Different volumetric parameter results

![diff_vpt](https://raw.githubusercontent.com/davidpypysp/softbivpt/master/files/diff_vpt.png)

## How to use:

* Use Cmake as the building tool of the program.  

* Run softbivpt.exe in shell with parameters.  

    ./softbivpt.exe N L scene#

    N = number of Samples

    L = Light Bounces

    scene# scene located at scenes/  
    Available scenes: default, scene, scene1, scene2, scene3, scene4, scene5  

    Example: ./softbivpt 16 4 scene5  

    Then you can choose rendering method such as volumetric path tracing and bidirectional path tracing

    (temp)  
    parameters sigma_s and sigma_a to control the scattering and absorption of light.
    you can modify them in src/main.cpp: line 68 to generate different volumetric effect to test.

    In src/Renderer.h: line 41, I ignore this line "#pragma omp parallel for" because openmp has some problem on my mac. You can reopen it if it works fine in your system.


## Test

* Rendering time on Mac   

![test](https://raw.githubusercontent.com/davidpypysp/softbivpt/master/files/test.png)  
