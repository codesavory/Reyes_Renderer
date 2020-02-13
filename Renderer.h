#pragma once
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "Primitives.h"
#include "States.h"
#include <Eigen/Eigen>
#include <iostream>
#include <numeric>
#include "Buffers.h"

inline
float edgeFunction(const Eigen::Vector4f& a, const Eigen::Vector4f& b, const Eigen::Vector4f& c)
{
    return ((c.x() - a.x()) * (b.y() - a.y()) - (c.y() - a.y()) * (b.x() - a.x()));
}





inline
void sample(int x, int y, int xsamples, int ysamples, FrameBuffer& fb, ZBuffer& zb,
    Triangle t) {

    Eigen::Vector3f pixel_color(0, 0, 0);
    for (float m = 0; m < xsamples; ++m) {
        for (float n = 0; n < ysamples; ++n) {

            float mMin = (float)m / xsamples + (0.5f / xsamples);
            float nMin = (float)n / ysamples + (0.5f / ysamples);
            float mMax = (float)(m + 1) / xsamples;
            float nMax = (float)(n + 1) / ysamples;

            // using random sampling values makes stuff noisy
            //float x_delta = ((float)rand() / RAND_MAX) * (mMax - mMin) + mMin;
            //float y_delta = ((float)rand() / RAND_MAX) * (nMax - nMin) + nMin;

            float sample_x = x + mMin;
            float sample_y = y + nMin;

            Eigen::Vector4f sample(sample_x, sample_y, 0.0, 0.0);
            auto v0 = t.screen_coordinates[0];
            auto v1 = t.screen_coordinates[1];
            auto v2 = t.screen_coordinates[2];

            float area = edgeFunction(v0, v1, v2);
            float w0 = edgeFunction(v1, v2, sample);
            float w1 = edgeFunction(v2, v0, sample);
            float w2 = edgeFunction(v0, v1, sample);

            bool is_inside = (w0 >= 0 && w1 >= 0 && w2 >= 0);

            if (is_inside) {
                // compute barycentriic coordinates
                w0 /= area;
                w1 /= area;
                w2 /= area;
                // linearly interpolate sample depth
                float oneOverZ = v0.z() * w0 + v1.z() * w1 + v2.z() * w2;
                float z = 1 / oneOverZ;

                if (z < zb(x, y, m, n).front()) {
                    zb(x, y, m, n).push_front(z);

                    auto c = t.colors;

                    float r = w0 * c[0].r + w1 * c[1].r + w2 * c[2].r;
                    float g = w0 * c[0].g + w1 * c[1].g + w2 * c[2].g;
                    float b = w0 * c[0].b + w1 * c[1].b + w2 * c[2].b;
                    Color z(r, g, b, 1.0);
                    fb(x, y, m, n) = z;

                    


                }
            }

        }
    }

}


void pimage(FrameBuffer frame_buffer) {
    const char* filename = "D:\\stbjpg3.jpg";
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
            out_buffer[index++] = pix_color.r;
            out_buffer[index++] = pix_color.g;
            out_buffer[index++] = pix_color.b;
            
            
            /*for (int m = 0; m < 2; ++m) {
                for (int n = 0; n < 2; ++n) {

                    index = (y * width * 3) + x * 3;
                    out_buffer[index++] = frame_buffer(x, y,m,n).r;
                    out_buffer[index++] = frame_buffer(x, y, m, n).g;
                    out_buffer[index++] = frame_buffer(x, y, m, n).b;

                }
            }*/
            
            
        }
    }

    stbi_write_jpg(filename, width, height, 3, out_buffer, 100);
    delete[] out_buffer;
    //return 0;
}



void render_frame(WorldState& world_state, RenderState& render_state, ImageState& image_state) {
	const int xsamples = render_state.xsamples;
	const int ysamples = render_state.ysamples;
	const int width = image_state.x_resolution;
	const int height = image_state.y_resolution;

	// responsible for storing color
	FrameBuffer frame_buffer(width, height, xsamples, ysamples);
    ZBuffer z_buffer(width, height, xsamples, ysamples);

    std::vector<std::unique_ptr<Primitive>>& world_obj_ptrs = world_state.object_ptrs;

    
	for (int i = 0; i < world_obj_ptrs.size(); ++i) {
        std::unique_ptr<Primitive> objPtr = std::move(world_obj_ptrs[i]);
        objPtr->build(width, height);


        //Primitive& obj = *objPtr;
        auto& points = objPtr->points;

        for (int j = 0; j < points.size(); ++j) {
            auto p = objPtr->points[j];
            p = objPtr -> mvp * p;
            // For some reason x axis and y axis are flipped, so multiplying by -1
            p.x() = p.x() / p.w() * -1;
            p.y() = p.y() / p.w() * -1;
            p.z() = p.z() / p.w();

			Eigen::Vector4f pixel_coordinate_space;

			float screen_x = (p.x() + 1.0f) * 0.5f * width;
			float screen_y = (1.0 - (p.y() + 1.0f) * 0.5f) * height;

            p.x() = screen_x;
            p.y() = screen_y;

			points[j] = p;

            //frame_buffer(p.x(), p.y(), 0, 0) = Color(255.0f, 255.0f, 255.0f, 1.0f);
		}
        

        // Sample triangles
        std::vector<TriangleVerts> triangle_vert_groups = objPtr->triangle_verts;
        for (int j = 0; j < triangle_vert_groups.size(); ++j){

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
                    tri.screen_coordinates[0] = v[0];
                    tri.screen_coordinates[1] = v[1];
                    tri.screen_coordinates[2] = v[2];

                    tri.colors[0] = objPtr->vertex_colors[triangle_vert_ids[0]];
                    tri.colors[1] = objPtr->vertex_colors[triangle_vert_ids[1]];
                    tri.colors[2] = objPtr->vertex_colors[triangle_vert_ids[2]];

                    sample(x, y, xsamples, ysamples, frame_buffer, z_buffer, tri);   
                }
            }

            

        }
        
	}

    pimage(frame_buffer);
    
}