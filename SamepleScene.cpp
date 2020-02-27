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

	RiFormat(512, 512, 1);
	RiPixelSamples(2, 2);
	RiShutter(0, 1);

	/* new code*/
	RiFrameBegin(1);
	RiTranslate(0, 0, -90);

	RiWorldBegin();

	RiDisplay("D:\\stbjpg3.jpg", "file", "rgb", RI_NULL);

	//RiTransformBegin();
	////RiRotate(90.0f, -1.0, 0.0, 0.0);
	//RiColor(red);
	//Ri_GeometricShader(checker_explode);
	//RiSphere(30, 0, 0, 0);
	//RiTransformEnd();
	
	RiTransformBegin();
	RiRotate(90.0f, -1.0, 0.0, 0.0);
	RiColor(blue);
	//parse_teapot_patches();
	//Ri_Texture(earth);
	RiSphere(20, 0, 0, 0);
	RiTransformEnd();
	
	//RiTorus(40, 20, 0, 0, 0);
	RiTransformEnd();
	RiWorldEnd();
	RiFrameEnd();
	RiEnd();


}

//	/* loop through all the frames */
//	for (i = 1; i <= nf; i++) {
//		RiFrameBegin(i);
//		//sprintf(name, "image_%02d.tif", i - 1);
//		//RiDisplay(name, "file", "rgb", RI_NULL);
//
//		//RiProjection("perspective", "fov", &fov, RI_NULL);
//		RiTranslate(0, -5, 60);
//		RiRotate(-120, 1, 0, 0);
//		RiRotate(25, 0, 0, 1);
//
//		RiWorldBegin();
//		RiColor(blue);
//		RiTransformBegin();
//		RiCylinder(1, 0, 20, 360, RI_NULL);
//		RiTranslate(0, 0, 20);
//		RiCone(2, 2, 360, RI_NULL);
//		RiTransformEnd();
//
//		RiColor(green);
//		RiTransformBegin();
//		RiRotate(-90, 1, 0, 0);
//		RiCylinder(1, 0, 20, 360, RI_NULL);
//		RiTranslate(0, 0, 20);
//		RiCone(2, 2, 360, RI_NULL);
//		RiTransformEnd();
//
//		RiColor(red);
//		RiTransformBegin();
//		RiRotate(90, 0, 1, 0);
//		RiCylinder(1, 0, 20, 360, RI_NULL);
//		RiTranslate(0, 0, 20);
//		RiCone(2, 2, 360, RI_NULL);
//		RiTransformEnd();
//
//
//		RiColor(white);
//		RiTransformBegin();
//		RiTranslate(p1[0] + slopex * (i - 1), p1[1] + slopey * (i - 1), p1[2] + slopez * (i - 1));
//		RiSphere(5, -5, 5, 360, RI_NULL);
//		RiTransformEnd();
//		RiWorldEnd();
//
//		/* when you hit this command you should output the final image for this frame */
//		RiFrameEnd();
//	}
//	RiEnd();
//};


int main(void) {
	// select which scene to render
	SampleScene1();

	//Eigen::Vector4f p[3];
	//p[0] << -0.9, 0.9, 0, 1;
	//p[1] << -0.9, -0.9, 0, 1;
	////p[2] << 20, 2, 0, 1;


	//std::vector<Eigen::Vector4f> points;
	//for (int i = 0; i < 2; ++i) {
	//	float screen_x = (p[i].x() + 1.0f) * 0.5f * 100;
	//	float screen_y = (1 - (p[i].y() + 1) * 0.5) * 100;

	//	std::cout << screen_x << "," << screen_y << "\n";
	//	points.push_back(Eigen::Vector4f(screen_x, screen_y, p[i].z(), p[i].w()));
	//}

	//
	//pimage(points);
	//render_perspective_projection();
}