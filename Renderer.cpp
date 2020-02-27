#pragma once
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <Eigen/Eigen>

#include "Types.h"
#include "Primitives.h"
#include "States.h"
#include "Buffers.h"
#include "Shaders.h"
#include "Renderer.h"
//#define _USE_MATH_DEFINES
//#include <OpenMesh/Core/IO/MeshIO.hh>
//#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
//typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh;


inline
float edge_function(const Eigen::Vector4f& a, const Eigen::Vector4f& b, const Eigen::Vector4f& c)
{
    return ((c.x() - a.x()) * (b.y() - a.y()) - (c.y() - a.y()) * (b.x() - a.x()));
}

inline UVTuple interpolate_uv(float w0, float w1, float w2, UVTuple uv[3]) {
    auto u = (w0 * uv[0].u + w1 * uv[1].u + w2 * uv[2].u);
    auto v = (w0 * uv[0].v + w1 * uv[1].v + w2 * uv[2].v);

    UVTuple r;
    r.u = u;
    r.v = v;
    return r;
}

inline Eigen::Vector4f interpolate_vector(float w0, float w1, float w2, Eigen::Vector4f normals[3]) {
    auto interpolated = w0 * normals[0] + w1 * normals[1] + w2 * normals[2];
    return interpolated;
}

inline
void sample(int x, int y, int xsamples, int ysamples, FrameBuffer& fb, ZBuffer& zb,
    Triangle t, void (*surface_shader)(FragmentShaderPayload& p),
    std::shared_ptr<Texture> texture) {

    // default background color
    Eigen::Vector3f pixel_color(0, 0, 0);
    for (float m = 0; m < xsamples; ++m) {
        for (float n = 0; n < ysamples; ++n) {

            float mMin = (float)m / xsamples + (0.5f / xsamples);
            float nMin = (float)n / ysamples + (0.5f / ysamples);
            float mMax = (float)(m + 1) / xsamples;
            float nMax = (float)(n + 1) / ysamples;

            //using random sampling values makes stuff noisy
            //float x_delta = ((float)rand() / RAND_MAX) * (mMax - mMin) + mMin;
            //float y_delta = ((float)rand() / RAND_MAX) * (nMax - nMin) + nMin;

            float sample_x = x + mMin;
            float sample_y = y + nMin;

            Eigen::Vector4f sample(sample_x, sample_y, 0.0, 0.0);
            auto v0 = t.screen_coordinates[0];
            auto v1 = t.screen_coordinates[1];
            auto v2 = t.screen_coordinates[2];

            float area = edge_function(v0, v1, v2);
            float w0 = edge_function(v1, v2, sample);
            float w1 = edge_function(v2, v0, sample);
            float w2 = edge_function(v0, v1, sample);

            bool is_inside = (w0 >= 0 && w1 >= 0 && w2 >= 0);

            if (is_inside) {
                // compute barycentriic coordinates
                w0 /= area;
                w1 /= area;
                w2 /= area;

                /*--------------------------*/
                float w_reciprocal = 1.0 / (w0 / v0.w() + w1 / v1.w() + w2 / v2.w());
                float z_interpolated = w0 * v0.z() / v0.w() + w1 * v1.z() / v1.w() + w2 * v2.z() / v2.w();
                z_interpolated *= w_reciprocal;
                float z = z_interpolated;

                // linearly interpolate sample depth
                //float oneOverZ = v0.z() * w0 + v1.z() * w1 + v2.z() * w2;
                //float z = 1 / oneOverZ;

                if (z < zb(x, y, m, n).front()) {
                    zb(x, y, m, n).push_front(z);

                    auto c = t.colors;

                    Color c0 = w0 * c[0];
                    Color c1 = w1 * c[1];
                    Color c2 = w2 * c[2];
                    Color c_interpolated = c0 + c1 + c2;

                    UVTuple uv = interpolate_uv(w0, w1, w2, t.uv_tups);
                    auto interpolated_xyz = interpolate_vector(w0, w1, w2, t.world_coordinates);
                    auto interpolated_normal = interpolate_vector(w0, w1, w2, t.normals);

                    FragmentShaderPayload p;
                    p.u = uv.u;
                    p.v = uv.v;
                    p.texture = texture;
                    p.pos = interpolated_xyz;
                    p.normal = interpolated_normal;
                    p.c = c_interpolated;


                    //surface_shader(p);
                    //blinn_phong_modded(p);
                    c_interpolated = p.c;
                    fb(x, y, m, n) = c_interpolated;

                }
            }

        }
    }

}


void pimage(FrameBuffer frame_buffer, const char* filename) {
    //const char* f = "D:\\stbjpg3.jpg";
    const int CHANNEL_NUM = 3;

    int width = frame_buffer.w;
    int height = frame_buffer.h;

    //grey background
    uint8_t* out_buffer = new uint8_t[width * height * CHANNEL_NUM]();

    // Set background color
    // For the following framebuffer, (0,0) pixel is at the top left
    int index = 0;
    for (int x = 0; x < width; ++x)
    {
        for (int y = 0; y < height; ++y)
        {
            index = (y * width * 3) + x * 3;
            Color pix_color = frame_buffer.collapse(x, y);
            out_buffer[index++] = pix_color.ir();
            out_buffer[index++] = pix_color.ig();
            out_buffer[index++] = pix_color.ib();
        }
    }

    stbi_write_jpg(filename, width, height, 3, out_buffer, 100);
    delete[] out_buffer;
}

void render_frame(WorldState& world_state, RenderState& render_state, ImageState& image_state) {
    const int xsamples = render_state.xsamples;
    const int ysamples = render_state.ysamples;
    const int width = image_state.x_resolution;
    const int height = image_state.y_resolution;

    std::shared_ptr<Texture> texture = std::make_shared<Texture>();
    void (*surface_shader)(FragmentShaderPayload & p) = earth;
    //void (*geometric_shader)(GeometricShaderPayload & p) = checker_explode;

    FrameBuffer frame_buffer(width, height, xsamples, ysamples);
    ZBuffer z_buffer(width, height, xsamples, ysamples);

    std::vector<std::unique_ptr<Primitive>>& world_obj_ptrs = world_state.object_ptrs;

    for (int i = 0; i < world_obj_ptrs.size(); ++i) {

        std::unique_ptr<Primitive> objPtr = std::move(world_obj_ptrs[i]);
        objPtr->build(width, height);
        objPtr->sh();
        surface_shader = objPtr->surface_shader;

        auto& points = objPtr->points;
        for (int j = 0; j < points.size(); ++j) {
            auto p = objPtr->points[j];

            Eigen::Vector4f sp = objPtr->mvp * p;
            // For some reason x axis and y axis are flipped, so multiplying by -1
            sp.x() = sp.x() / sp.w() * 1;
            sp.y() = sp.y() / sp.w() * 1;
            sp.z() = sp.z() / sp.w();

            Eigen::Vector4f wp = objPtr->m * p;
            //wp.x() = wp.x() * 1;
            //wp.y() = wp.y() * 1;

            Eigen::Vector4f pixel_coordinate_space;
            float screen_x = (sp.x() + 1.0f) * 0.5f * width;
            float screen_y = (1.0 - (sp.y() + 1.0f) * 0.5f) * height;
            sp.x() = screen_x;
            sp.y() = screen_y;

            points[j] = sp;
            //objPtr->normals[j] = wp.normalized();
            objPtr->world_points[j] = wp;

            Eigen::Vector4f n = objPtr->normals[j];
            auto m_n = objPtr->m.inverse().transpose();
            n = m_n * n;
            //n.normalize();
            objPtr->normals[j] = n;
        }


        // Sample triangles
        std::vector<TriangleVerts> triangle_vert_groups = objPtr->triangle_verts;
        for (int j = 0; j < triangle_vert_groups.size(); ++j) {

            auto triangle_vert_ids = triangle_vert_groups[j].ids;

            std::vector<Eigen::Vector4f> v({
                points[triangle_vert_ids[0]],
                points[triangle_vert_ids[1]],
                points[triangle_vert_ids[2]]
            });

            float bb_left_x = width, bb_right_x = 0, bb_top_y = 0, bb_bottom_y = height;
            for (int vertex_no = 0; vertex_no < 3; vertex_no++)
            {
                Eigen::Vector4f triangle_vertex = v[vertex_no];
                //std::cout << "\n" << triangle_vertex << "\n";
                if (triangle_vertex.x() < bb_left_x)
                    bb_left_x = triangle_vertex.x();
                if (triangle_vertex.x() > bb_right_x)
                    bb_right_x = triangle_vertex.x();
                if (triangle_vertex.y() < bb_bottom_y)
                    bb_bottom_y = triangle_vertex.y();
                if (triangle_vertex.y() > bb_top_y)
                    bb_top_y = triangle_vertex.y();
            }

            // clamp to left edge
            bb_left_x = std::max(bb_left_x, 0.0f);
            //clamp to right edge
            bb_right_x = std::min(bb_right_x, (float)width);
            //clamp to bottom edge
            bb_bottom_y = std::max(bb_bottom_y, 0.0f);
            //clamp to top edge
            bb_top_y = std::min(bb_top_y, (float)height);


            for (int x = bb_left_x; x < bb_right_x; x++)
            {
                for (int y = bb_bottom_y; y < bb_top_y; y++)
                {

                    Triangle tri;
                    for (int k = 0; k < 3; ++k) {
                        tri.screen_coordinates[k] = v[k];
                        tri.world_coordinates[k] = objPtr->world_points[triangle_vert_ids[k]];
                        tri.colors[k] = objPtr->vertex_colors[triangle_vert_ids[k]];
                        tri.uv_tups[k] = objPtr->get_uv(triangle_vert_ids[k]);
                        tri.normals[k] = objPtr->normals[triangle_vert_ids[k]];
                    }

                    sample(
                        x, y, xsamples, ysamples,
                        frame_buffer, z_buffer,
                        tri, surface_shader, texture
                    );
                }
            }



        }


    }


    pimage(frame_buffer, image_state.filename.c_str());

}