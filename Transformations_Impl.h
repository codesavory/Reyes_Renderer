#pragma once
#include<iostream>
#include "Matrix_Impl.h"
#include <Eigen/Eigen>

constexpr double MY_PI = 3.1415926;

/*This file implements 3D transformation, where it takes
a 3D model data structure and applies the respective 3D
transformations and returns the 2D coordinates after applying
Model-View-Projection Transformations*/

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
		-eye_pos[2], 0, 0, 0, 1;

	view = translate * view;

	return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle, char axis)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

	// TODO: Implement this function
	// Create the model matrix for rotating the triangle around the Z axis.
	// Then return it.
	rotation_angle *= MY_PI / 180;//degrees to radians
	std::cout << "Rotation Angle in radians:	" << rotation_angle;

	if (axis == 'z')
	{
		model << cos(rotation_angle), -sin(rotation_angle), 0, 0,
			sin(rotation_angle), cos(rotation_angle), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1; //rotation about z-axis by rotation_angle
	}
	else if (axis == 'x')
	{
		model << 1, 0, 0, 0,
			0, cos(rotation_angle), -sin(rotation_angle), 0,
			0, sin(rotation_angle), cos(rotation_angle), 0,
			0, 0, 0, 1;
	}
	else if (axis == 'y')
	{
		model << cos(rotation_angle), 0, sin(rotation_angle), 0,
			0, 1, 0, 0,
			-sin(rotation_angle), 0, cos(rotation_angle), 0,
			0, 0, 0, 1;
	}
	std::cout << "Model:	" << model;

	return model;
}

Eigen::Matrix4f get_ortho_projection_matrix(float width, float height,float zFar, float zNear)
{
	Eigen::Matrix4f ortho_projection = Eigen::Matrix4f::Identity();
	ortho_projection << 1 / width, 0, 0, 0,
		0, 1 / height, 0, 0,
		0, 0, -(2 / zFar - zNear), 0,
		0, 0, 0, 1;
	std::cout << "Ortho Projection:	" << ortho_projection;
	return ortho_projection;
}

Eigen::Matrix4f get_perspective_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	const float ar = aspect_ratio;
	const float zRange = zNear - zFar;
	const float eye_fov_in_rads = eye_fov * MY_PI / 180;
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

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
	float zNear, float zFar)
{
	// Students will implement this function
	float height = 2 * tan(eye_fov / 2) * zFar;
	float bottom = height / 2;
	float top = -height / 2;

	float width = aspect_ratio * height;
	float right = -width / 2;
	float left = width / 2;

	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

	// TODO: Implement this function
	// Create the projection matrix for the given parameters.
	// Then return it.
	Eigen::Matrix4f ortho_right = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f ortho_left = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f ortho = Eigen::Matrix4f::Identity();

	ortho_right << 1, 0, 0, -(right + left) / 2,
		0, 1, 0, -(top + bottom) / 2,
		0, 0, 1, -(zNear + zFar) / 2,
		0, 0, 0, 1;
	ortho_left << 2 / (right - left), 0, 0, 0,
		0, 2 / (top - bottom), 0, 0,
		0, 0, 2 / (zNear = zFar), 0,
		0, 0, 0, 1;
	ortho = ortho_left * ortho_right;

	Eigen::Matrix4f ortho_to_projection = Eigen::Matrix4f::Identity();
	ortho_to_projection << zNear, 0, 0, 0,
		0, zNear, 0, 0,
		0, 0, zNear + zFar, -zNear * zFar,
		0, 0, 1, 0;

	projection = ortho * ortho_to_projection;
	std::cout << "Projection:	" << projection;

	return projection;
}