#pragma once

#include "Types.h"
#include "States.h"
#include <Eigen/Eigen>
#include "Buffers.h"

//#define _USE_MATH_DEFINES
//#include <OpenMesh/Core/IO/MeshIO.hh>
//#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
//typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh;


inline
float edge_function(const Eigen::Vector4f& a, const Eigen::Vector4f& b, const Eigen::Vector4f& c);

inline UVTuple interpolate_uv(float w0, float w1, float w2, UVTuple uv[3]);

inline Eigen::Vector4f interpolate_vector(float w0, float w1, float w2, Eigen::Vector4f normals[3]);
inline
void sample(int x, int y, int xsamples, int ysamples, FrameBuffer& fb, ZBuffer& zb,
    Triangle t, void (*surface_shader)(FragmentShaderPayload& p),
    std::shared_ptr<Texture> texture);


void pimage(FrameBuffer frame_buffer, const char* filename);

void render_frame(WorldState& world_state, RenderState& render_state, ImageState& image_state);