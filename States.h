#pragma once
#include "Ri.h"
#include "Primitives.h"
#include <Eigen/Eigen>

class ImageState {
public:
	int x_resolution, y_resolution;
	float pixelaspectratio;
	const char* filename;

	ImageState() {
		filename = nullptr;
	}
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

	TransformationState() {
		is_in_transform_block = 0;
		current_transformation = Eigen::Matrix4f::Identity();
	}
};

class WorldState {
public:
	std::vector<std::unique_ptr<Primitive>> object_ptrs;
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

	void (*surface_shade)(FragmentShaderPayload p);
	std::shared_ptr<Texture> texture;

	RenderState() {
		//projection_type = RI_ORTHOGRAPHIC;
		zNear = 0.0;
		zFar = 100.0;
		transformation = Eigen::Matrix4f::Identity();
	}
};