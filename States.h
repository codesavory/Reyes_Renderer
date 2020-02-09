#pragma once
#include "Ri.h"
#include "Primitives.h"
#include <Eigen/Eigen>

class ImageState {
public:
	int x_resolution, y_resolution;
	float pixelaspectratio;
	const char* filename;

	ImageState();
};

class CameraState {
public:
	// Related to the times at which the shutter opens and closes
	// https://renderman.pixar.com/resources/RenderMan_20/options.html#rishutter
	float shutter_min;
	float shutter_max;
};

class TransformationState {
public:
	// This represents the current matrix used for model-to-world transformation
	// Defaults to Identity
	// Set using Ri* transformation functions when inside an RiTRansformBegin block
	Eigen::Matrix4f current_transformation;
	int is_in_transform_block;

	TransformationState();
};

class WorldState {
public:
	std::vector<Primitive> objects;
	std::vector<Eigen::Vector4f> world_mesh;
};

class RenderState {
public:

	//RtToken projection_type;
	float zNear, zFar;

	// https://renderman.pixar.com/resources/RenderMan_20/attributes.html#ricolor
	Color current_color;
	// Set by RiOpacity

	// Set by RiPixelSamples
	// https://renderman.pixar.com/resources/RenderMan_20/options.html#ripixelsamples
	float xsamples, ysamples;

	// https://renderman.pixar.com/resources/RenderMan_20/options.html#camera
	// If it is not set by RiFrameAspectRatio, then it is calculated using ImageState
	float frame_aspect_ratio;

	// https://renderman.pixar.com/resources/RenderMan_20/graphicsState.html#ribegin
	int f_no;
	int is_in_world_block;

	// This matrix represent world-to-camera and camera-to-screen projection combined
	Eigen::Matrix4f transformation;

	//std::vector<Eigen::Vector4f> world_mesh;
	RenderState();
};