#include "Ri.h"
#include "States.h"


ImageState::ImageState() {
	filename = nullptr;
}

RenderState::RenderState() {
	//projection_type = RI_ORTHOGRAPHIC;
	zNear = 0.0;
	zFar = 100.0;
	transformation = Eigen::Matrix4f::Identity();
}

TransformationState::TransformationState() {
	is_in_transform_block = 0;
	current_transformation = Eigen::Matrix4f::Identity();
}