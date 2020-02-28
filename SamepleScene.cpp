#include <stdio.h>
#include "Ri.h"
//#include "Renderer.h"
#include <Eigen/Eigen>
#include <iostream>
//#include "Reyes_Impl.h"
#include <fstream>
#include <numeric>
#include "Shaders.h";


inline void parse_teapot_patches() {
	std::ifstream input("a.txt");
	std::string line;

	int num_patch;
	input >> num_patch;

	for (int i = 0; i < num_patch; ++i) {
		std::vector<Eigen::Vector3f> cp;
		cp.resize(16);

		int p, q;
		input >> p >> q;
		
		for (int j = 0; j < 16; ++j) {
			float x, y, z;
			input >> x >> y >> z;
			cp[j] = Eigen::Vector3f(x, y, z);
		}

		Ri_Patch(cp);
	}
}

void Tunnel()
{
	RiBegin(RI_NULL);
		//RiFormat(960, 720, 1.0);
		RiFormat(400, 300, 1.0);
		RiDisplay("D:\\a\\Tunnel.png", "", "");
		RiPixelSamples(2,2);

		RiFrameBegin(0);
			/* set the perspective transformation */
			float fov = 45.0;
			RiProjection(RI_PERSPECTIVE, "fov", fov, RI_NULL);
			//RiProjection(RI_ORTHOGRAPHIC);

			RiWorldBegin();
				RiTransformBegin();
					RtColor color = {1,0,0};
					RiColor(color);
					RiTranslate(0, 0.5, 7.0);
					RiRotate(60, 1, 0, 0);
					RiTorus(1, .25, 0, 360, 360, RI_NULL);
				RiTransformEnd();
				RiTransformBegin();
					color[0] = 0; color[1] = 1;
					RiColor(color);
					RiTranslate(0, 0, 8.0);
					RiRotate(60, 1, 0, 0);
					RiRotate(30, 0, 1, 0);
					RiCylinder(1, -1, 1, 360, RI_NULL);
				RiTransformEnd();
				RiTransformBegin();
					color[1] = 0; color[2] = 1;
					RiColor(color);
					RiTranslate(0, 1, 9.0);
					RiRotate(60, 1, 0, 0);
					RiSphere(1.0, -1.0, 1.0, 360, RI_NULL);
				RiTransformEnd();
				RiTransformBegin();
					color[0] = 1; color[1] = .4; color[2] = .4;
					RiColor(color);
					CHECK_SIZE_X = 40;
					CHECK_SIZE_Y = 40;
					RiSurface(CHECKERBOARD);
					RiTranslate(0, -1, 8.5);
					RiRotate(-160, 1, 0, 0);
					RiRotate(30, 0, 1, 0);
					RiCone(2, 1, 360, RI_NULL);
				RiTransformEnd();
				RiTransformBegin();
					CHECK_SIZE_X = 40;
					CHECK_SIZE_Y = 40;
					RiTranslate(0, 0, 7.0);
					RiCylinder(3, 0, 10, 360, RI_NULL);
				RiTransformEnd();
			RiWorldEnd();

		RiFrameEnd();

	RiEnd();
}

void Earth()
{
	RiBegin(RI_NULL);
		//RiFormat(960, 720, 1.0);
		RiFormat(400, 300, 1.0);
		//RiFormat(200, 150, 1.0);
		RiDisplay("D:\\a\\Earth.png", "", "");
		RiPixelSamples(2,2);

		RiFrameBegin(0);
			/* set the perspective transformation */
			float fov = 45.0;
			RiProjection(RI_PERSPECTIVE, "fov", fov, RI_NULL);
			//RiProjection(RI_ORTHOGRAPHIC);

			RiWorldBegin();
				RiTransformBegin();
					RiMakeTexture("textures\\earth_2.jpg", 0);
					//void (*earthShader)(void) = TextureMap0;
					RiSurface(texture_shader);
					RtColor blue = { 0, 0, 1 };
					RtColor opacity = { .9, .9, .9 };
					RiColor(blue);
					/*RiOpacity(opacity);
					BUMP_AMPLITUDE = .02;
					BUMP_MIN_FREQ_EXP = 14;
					BUMP_MAX_FREQ_EXP = 16;
					RiDisplacement(BUMPY);*/
					RiTranslate(0, 0, 5.0);
					RiRotate(-175, 0, 1, 0);
					RiRotate(110, 1, 0, 0);
					RiSphere(1, -1, 1, 360, RI_NULL);
				RiTransformEnd();
			RiWorldEnd();

		RiFrameEnd();

	RiEnd();
}

void SampleScene1(void) {
	int i;
	int nf;
	float slopex, slopey, slopez;
	char name[50];

	RtColor red = { 1,0,0 };
	RtColor green = { 0,1,0 };
	RtColor blue = { 0,0,1 };
	RtColor white = { 1,1,1 };


	RtPoint p1 = { 30,0,10 }; /* ball's initial position */
	RtPoint p2 = { 0,20,10 }; /* ball's final position  */


	RtFloat fov = 45;
	RtFloat intensity1 = 0.1;
	RtFloat intensity2 = 1.5;
	RtInt init = 0, end = 1;


	nf = 100; /* number of frames to output */
	slopex = (p2[0] - p1[0]) / nf;
	slopey = (p2[1] - p1[1]) / nf;
	slopez = (p2[2] - p1[2]) / nf;

	RiBegin(RI_NULL);
	RiFormat(320, 240, 1);
	RiPixelSamples(2, 2);
	RiShutter(0, 1);

	/* loop through all the frames */
	for (i = 1; i <= nf; i++) {
		RiFrameBegin(i);
		sprintf_s(name, "D:\\a\\image_%02d.tif", i - 1);
		RiDisplay(name, "file", "rgb", RI_NULL);

		RiProjection(RI_PERSPECTIVE, "fov", fov, RI_NULL);
		RiTranslate(0, -5, 60);
		RiRotate(-120, 1, 0, 0);
		RiRotate(25, 0, 0, 1);

		RiWorldBegin();
		RiColor(blue);
		RiTransformBegin();
		RiCylinder(1, 0, 20, 360, RI_NULL);
		RiTranslate(0, 0, 20);
		RiCone(2, 2, 360, RI_NULL);
		RiTransformEnd();

		RiColor(green);
		RiTransformBegin();
		RiRotate(-90, 1, 0, 0);
		RiCylinder(1, 0, 20, 360, RI_NULL);
		RiTranslate(0, 0, 20);
		RiCone(2, 2, 360, RI_NULL);
		RiTransformEnd();

		RiColor(red);
		RiTransformBegin();
		RiRotate(90, 0, 1, 0);
		RiCylinder(1, 0, 20, 360, RI_NULL);
		RiTranslate(0, 0, 20);
		RiCone(2, 2, 360, RI_NULL);
		RiTransformEnd();


		RiColor(white);
		RiTransformBegin();
		RiTranslate(p1[0] + slopex * (i - 1), p1[1] + slopey * (i - 1), p1[2] + slopez * (i - 1));
		RiSphere(5, -5, 5, 360, RI_NULL);
		RiTransformEnd();
		RiWorldEnd();

		/* when you hit this command you should output the final image for this frame */
		RiFrameEnd();
	}
	RiEnd();
};


int main(void) {
	// select which scene to render
	SampleScene1();
}