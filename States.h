#include "Ri.h"
#include <Eigen/Eigen>

class CameraState {
public:
	// Related to the times at which the shutter opens and closes
	// https://renderman.pixar.com/resources/RenderMan_20/options.html#rishutter
	float shutter_min;
	float shutter_max;
};

class GraphicState {
public:
	// Set by RiColor
	RtColor color;

	// Set by RiPixelSamples
	// https://renderman.pixar.com/resources/RenderMan_20/options.html#ripixelsamples
	float xsamples, ysamples;
};

class ImageState {
public:
	int x_resolution, y_resolution;
	float pixelaspectratio;
	const char* filename;

	ImageState();
};

class FrameState {
public:
	// https://renderman.pixar.com/resources/RenderMan_20/graphicsState.html#ribegin
	int f_no;
	Eigen::Matrix4f current_transformation;
	FrameState();
};