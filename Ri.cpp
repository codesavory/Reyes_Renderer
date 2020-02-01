#include "Ri.h"
#include "States.h"

FrameState frame_state;
ImageState image_state;
CameraState camera_state;
GraphicState graphics_state;

void RiBegin(RtToken token) {
	frame_state = FrameState();
	camera_state = CameraState();
};

void RiFormat(int x_resolution, int y_resolution, float pixelaspectratio) {
	image_state.x_resolution = x_resolution;
	image_state.y_resolution = y_resolution;
	image_state.pixelaspectratio = pixelaspectratio;
}

void RiPixelSamples(float xsamples, float ysamples) {
	graphics_state.xsamples = xsamples;
	graphics_state.ysamples = ysamples;
}

void RiShutter(float shutter_min, float shutter_max) {
	camera_state.shutter_min = shutter_min;
	camera_state.shutter_max = shutter_max;
}

void RiFrameBegin(int i) {
	frame_state.f_no = i;
}

template<typename T, typename... Args>
void RiProjection(T projetion_type, Args... args) {

}