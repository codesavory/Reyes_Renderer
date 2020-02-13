#include "Ri.h"
#include "Primitives.h"
#include "States.h"
#include "Renderer.h"
#include <iostream>

#define PI 3.141592653589793238462643383279502884197

ImageState image_state;
CameraState camera_state;
RenderState render_state;
TransformationState transformation_state;
WorldState world_state;

Eigen::Matrix4f get_ortho_projection_matrix(float width, float height, float zFar, float zNear)
{
	Eigen::Matrix4f ortho_projection = Eigen::Matrix4f::Identity();
	ortho_projection << 1 / width, 0, 0, 0,
		0, 1 / height, 0, 0,
		0, 0, -(2 / zFar - zNear), 0,
		0, 0, 0, 1;
	//std::cout << "Ortho Projection:	" << ortho_projection;
	return ortho_projection;
}

Eigen::Matrix4f get_perspective_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	const float ar = aspect_ratio;
	const float zRange = zNear - zFar;
	const float eye_fov_in_rads = eye_fov * PI / 180;
	const float tanHalfFOV = tanf(eye_fov_in_rads / 2.0);

	Eigen::Matrix4f m = Eigen::Matrix4f::Identity();

	m(0, 0) = 1.0f / (tanHalfFOV * ar);
	m(0, 1) = 0.0f;
	m(0, 2) = 0.0f;
	m(0, 3) = 0.0f;

	m(1, 0) = 0.0f;
	m(1, 1) = 1.0f / tanHalfFOV;
	m(1, 2) = 0.0f;
	m(1, 3) = 0.0f;

	m(2, 0) = 0.0f;
	m(2, 1) = 0.0f;
	m(2, 2) = (-zNear - zFar) / zRange;
	m(2, 3) = 2.0f * zFar * zNear / zRange;

	m(3, 0) = 0.0f;
	m(3, 1) = 0.0f;
	m(3, 2) = 1.0f;
	m(3, 3) = 0.0f;

	return m;
}

void RiBegin(RtToken token) {
	// TODO add reyes defaults
	render_state = RenderState();
	camera_state = CameraState();
}

void RiEnd() {

}

void RiFormat(int x_resolution, int y_resolution, float pixelaspectratio) {
	image_state.x_resolution = x_resolution;
	image_state.y_resolution = y_resolution;
	image_state.pixelaspectratio = pixelaspectratio;

	//set default frame aspect ratio as image aspect ratio
	render_state.frame_aspect_ratio = (x_resolution * pixelaspectratio) / y_resolution;
}

void RiPixelSamples(float xsamples, float ysamples) {
	render_state.xsamples = xsamples;
	render_state.ysamples = ysamples;
}

void RiShutter(float shutter_min, float shutter_max) {
	camera_state.shutter_min = shutter_min;
	camera_state.shutter_max = shutter_max;
}

void RiFrameBegin(int i) {
	render_state.f_no = i;
	//render_state.projection_type = RI_ORTHOGRAPHIC;
}

void RiColor(RtColor color) {
	float r = color[0];
	float g = color[1];
	float b = color[2];

	r *= 255.0f;
	g *= 255.0f;
	b *= 255.0f;
	Color c = Color(r, g, b, 1.0f);
	render_state.current_color = c;
}

void RiFrameAspectRatio(float aspect_ratio) {
	render_state.frame_aspect_ratio = aspect_ratio;
}

/*
template<typename T, typename... Args>
void RiProjection(T projetion_type, Args... args) {
	//https://renderman.pixar.com/resources/RenderMan_20/options.html#riprojection
}
*/

void RiWorldBegin() {
	// Set the transformation matrix in render state according to the defined projection
	//auto perspective = render_state.projection_type;
	

}

void RiWorldEnd() {
}


void RiFrameEnd() {
	render_frame(world_state, render_state, image_state);
}

void RiTransformBegin() {
	transformation_state.current_transformation = Eigen::Matrix4f::Identity();
	transformation_state.is_in_transform_block = 1;
}

void RiTransformEnd() {
	transformation_state.current_transformation = Eigen::Matrix4f::Identity();
	transformation_state.is_in_transform_block = 0;
}

void RiTranslate(RtFloat dx, RtFloat dy, RtFloat dz) {
	// Use actual matrix here
	Eigen::Matrix4f model;
	model << 
		1, 0, 0, dx,
		0, 1, 0, dy,
		0, 0, 1, dz,
		0, 0, 0, 1;
	
	if (transformation_state.is_in_transform_block) {
		transformation_state.current_transformation = model * transformation_state.current_transformation;
	}
	else {
		render_state.transformation = model * render_state.transformation;
	}
}

void RiSphere(float radius, float zmin, float zmax, float tmax) {
	auto model_to_world_matrix = transformation_state.current_transformation;
	auto world_to_view_transformation = render_state.transformation;
	//auto view_to_frame_transformation = get_perspective_projection_matrix(45.0, 1.0, 1, 100);
	auto view_to_frame_transformation = get_ortho_projection_matrix(50, 50, 1, 100);

	std::unique_ptr<Primitive> sphere_ptr = std::make_unique<Sphere>(radius, -1* radius, radius, 360.0f);
	sphere_ptr -> mvp = view_to_frame_transformation * world_to_view_transformation * model_to_world_matrix;
	world_state.object_ptrs.push_back(std::move(sphere_ptr));
	
}

void RiCylinder(float radius, float zmin, float zmax, float tmax) {
	auto model_to_world_matrix = transformation_state.current_transformation;
	auto world_to_view_transformation = render_state.transformation;
	auto view_to_frame_transformation = get_perspective_projection_matrix(90.0f, 1.0, 1, 100);
	//auto view_to_frame_transformation = get_ortho_projection_matrix(50, 50, 1, 100);

	std::unique_ptr<Primitive> c_ptr = std::make_unique<Cylinder>(radius, zmin, zmax, tmax);
	c_ptr->mvp = view_to_frame_transformation * world_to_view_transformation * model_to_world_matrix;
	c_ptr->primitive_color = render_state.current_color;
	world_state.object_ptrs.push_back(std::move(c_ptr));
}



void RiTorus(float majorRadius, float minorRadius, float phimin, float phimax, float tmax) {
	auto model_to_world_matrix = transformation_state.current_transformation;
	auto world_to_view_transformation = render_state.transformation;
	auto view_to_frame_transformation = get_perspective_projection_matrix(90.0f, 1.0, 1, 100);
	//auto view_to_frame_transformation = get_ortho_projection_matrix(50, 50, 1, 100);

	std::unique_ptr<Primitive> c_ptr = std::make_unique<Torus>(majorRadius, minorRadius, phimin, phimax, tmax);
	c_ptr->mvp = view_to_frame_transformation * world_to_view_transformation * model_to_world_matrix;
	c_ptr->primitive_color = render_state.current_color;
	world_state.object_ptrs.push_back(std::move(c_ptr));
}



void RiRotate(float rotation_angle, float dx, float dy, float dz) {

	Eigen::Matrix4f model = Eigen::Matrix4f::Zero();
	float angle = rotation_angle * PI / 180.0f;

	float x = dx;
	float y = dy;
	float z = dz;
	
	model(0,0) = x * x * (1 - cos(angle)) + cos(angle);
	model(1,0) = x * y * (1 - cos(angle)) + z * sin(angle);
	model(2,0) = x * z * (1 - cos(angle)) - y * sin(angle);
	model(0,1) = y * x * (1 - cos(angle)) - z * sin(angle);
	model(1,1) = y * y * (1 - cos(angle)) + cos(angle);
	model(2,1) = y * z * (1 - cos(angle)) + x * sin(angle);
	model(0,2) = z * x * (1 - cos(angle)) + y * sin(angle);
	model(1,2) = z * y * (1 - cos(angle)) - x * sin(angle);
	model(2,2) = z * z * (1 - cos(angle)) + cos(angle);
	model(3,3) = 1.0f;

	/*
	if (dz == 1.0f)
	{
		model << cos(rotation_angle), -sin(rotation_angle), 0, 0,
			sin(rotation_angle), cos(rotation_angle), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1; //rotation about z-axis by rotation_angle
	}
	else if (dx == 1.0f)
	{
		model << 1, 0, 0, 0,
			0, cos(rotation_angle), -sin(rotation_angle), 0,
			0, sin(rotation_angle), cos(rotation_angle), 0,
			0, 0, 0, 1;
	}
	else if (dy == 1.0f)
	{
		model << cos(rotation_angle), 0, sin(rotation_angle), 0,
			0, 1, 0, 0,
			-sin(rotation_angle), 0, cos(rotation_angle), 0,
			0, 0, 0, 1;
	}
	*/


	if (transformation_state.is_in_transform_block) {
		transformation_state.current_transformation = model * transformation_state.current_transformation;
	}
	else {
		render_state.transformation = model * render_state.transformation;
	}
}


