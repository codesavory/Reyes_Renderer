#pragma once
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "Primitives.h"
#include "States.h"
#include <Eigen/Eigen>
#include <functional>
#include <iostream>
#include <tuple>
#include <numeric>

 std::tuple<float, float, float> computeBarycentric2D(float x, float y, Eigen::Vector4f* v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return { c1,c2,c3 };
}


inline
float edgeFunction(const Eigen::Vector4f& a, const Eigen::Vector4f& b, const Eigen::Vector4f& c)
{
    return ((c.x() - a.x()) * (b.y() - a.y()) - (c.y() - a.y()) * (b.x() - a.x()));
}

//inline
//bool isInside(float x, float y, const Eigen::Vector4f* _v) {
//    Eigen::Vector4f p(x, y, 0.0, 0.0);
//    bool inside = true;
//    inside &= edgeFunction(_v[0], _v[1], p);
//    inside &= edgeFunction(_v[1], _v[2], p);
//    inside &= edgeFunction(_v[2], _v[0], p);
//    return inside;
//}


template <class T>
class Buffer {
public:
    int w;
    int h;
    int xsamples, ysamples;
    std::vector<T> buff;

    Buffer(int width, int heigth, int xsamples, int ysamples) {
        this->w = width;
        this -> h = heigth;
        this->xsamples = xsamples;
        this->ysamples = ysamples;
        buff.resize(width * xsamples * heigth * ysamples);
        //buff.resize(width * heigth);
    }

    T operator() (int x, int y, int m, int n) const {
        int id = (y * xsamples * ysamples * w + x * xsamples * ysamples + n * xsamples + m);
        //int id = (y * xsamples * ysamples * w + x * xsamples * ysamples + 0);
        //int id = y * w + x;
        return buff[id];
    }

    T& operator() (int x, int y, int m, int n) {
        int id = (y * xsamples * ysamples * w + x * xsamples * ysamples + n * xsamples + m);
        //int id = (y * xsamples * ysamples * w + x * xsamples * ysamples + 0);
        //int id = y * w + x;
        return buff[id];
    }
};


class FrameBuffer : public Buffer<Color> {
public:
    FrameBuffer(int width, int heigth, int xsamples, int ysamples)
        : Buffer(width, heigth, xsamples, ysamples) {}

    Color collapse(int x, int y) {

        Eigen::Vector4f pix_color = Eigen::Vector4f::Zero();

        for (int m = 0; m < xsamples; ++m) {
            for (int n = 0; n < ysamples; ++n) {
                Color sc = this->operator()(x, y, m, n);
                pix_color += Eigen::Vector4f(sc.r, sc.g, sc.b, sc.a);
            }
        }

        pix_color /= (xsamples * ysamples);
        Color c(pix_color[0], pix_color[1], pix_color[2], pix_color[3]);
        return c;
    }
};



class ZBuffer : public Buffer<std::list<float>> {
public:
    ZBuffer(int width, int heigth, int xsamples, int ysamples)
        : Buffer(width, heigth, xsamples, ysamples) 
    {}

    std::list<float> operator() (int x, int y, int m, int n) const {
        std::list<float> buff_at_loc = Buffer<std::list<float>>::operator()(x, y, m, n);
        if (buff_at_loc.empty()) {
            buff_at_loc.push_front(std::numeric_limits<float>::infinity());
            return buff_at_loc;
        }
        else {
            return buff_at_loc;
        }
    }

    std::list<float>& operator() (int x, int y, int m, int n) {
        std::list<float>& buff_at_loc = Buffer<std::list<float>>::operator()(x, y, m, n);
        if (buff_at_loc.empty()) {
            buff_at_loc.push_front(std::numeric_limits<float>::infinity());
            return buff_at_loc;
        }
        else {
            return buff_at_loc;
        }
    }

};


void sample(int x, int y, int xsamples, int ysamples, FrameBuffer& fb, ZBuffer& zb, 
    Triangle t) {

    Eigen::Vector3f pixel_color(0, 0, 0);
    for (float m = 0; m < xsamples; ++m) {
        for (float n = 0; n < ysamples; ++n) {

            float mMin = (float)m / xsamples + (0.5f / xsamples);
            float nMin = (float)n / ysamples + (0.5f / ysamples);
            float mMax = (float)(m + 1) / xsamples;
            float nMax = (float)(n + 1) / ysamples;

            float x_delta = ((float)rand() / RAND_MAX) * (mMax - mMin) + mMin;
            float y_delta = ((float)rand() / RAND_MAX) * (nMax - nMin) + nMin;

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
                    fb(x, y, m, n) = t.color[0];
                }
            }

            //if (is_inside) {
            //    // z buff test
            //    std::vector<Eigen::Vector4f> v = t.screen_coordinates;
            //    std::tie(alpha, beta, gamma) = computeBarycentric2D(x, y, v.data());
            //    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            //    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            //    z_interpolated *= w_reciprocal;


            //    float curr_depth = zb(x, y, m, n).front();
            //    //fb(x, y, m, n) = t.color[0]
            //    if (z_interpolated <= curr_depth) {
            //        zb(x, y, m, n).push_front(z_interpolated);

            //        auto c = t.color;

            //        //c[0].r /= v[0].w(), c[0].g /= v[0].w(), c[0].b /= v[0].w();
            //        //c[1].r /= v[1].w(), c[1].g /= v[1].w(), c[1].b /= v[1].w();
            //        //c[2].r /= v[2].w(), c[2].g /= v[2].w(), c[2].b /= v[2].w();

            //        float r = alpha * c[0].r + beta * c[1].r + gamma * c[2].r;
            //        float g = alpha * c[0].g + beta * c[1].g + gamma * c[2].g;
            //        float b = alpha * c[0].b + beta * c[1].b + gamma * c[2].b;

            //        Color z(r, g, b, 1.0);
            //        fb(x, y, m, n) = z;
            //    }
            //    else {
            //        pixel_color += Eigen::Vector3f(color.r, color.g, color.b);
            //    }
            //   
            //    
            //}
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




void render_frame(std::vector<Primitive> world_objects, RenderState render_state, ImageState image_state) {
	const int xsamples = render_state.xsamples;
	const int ysamples = render_state.ysamples;
	const int width = image_state.x_resolution;
	const int height = image_state.y_resolution;

	// responsible for storing color
	FrameBuffer frame_buffer(width, height, 2, 2);
    ZBuffer z_buffer(width, height, 2, 2);

    //std::vector<Eigen::Vector4f> pix;

	for (int i = 0; i < world_objects.size(); ++i) {
		Primitive& obj = world_objects[i];
		auto ndc_points = obj.transformed_points;

        // Convert ndc points to pixel coordinates
		for (int j = 0; j < ndc_points.size(); ++j) {
			auto ndc_p = ndc_points[j];
			Eigen::Vector4f pixel_coordinate_space;

			float screen_x = (ndc_p.x() + 1.0f) * 0.5f * width;
			float screen_y = (1.0 - (ndc_p.y() + 1.0f) * 0.5f) * height;

			pixel_coordinate_space << screen_x, screen_y, ndc_p.z(), ndc_p.w();
			obj.transformed_points[j] = pixel_coordinate_space;

            //if (screen_x < 0 || screen_x > width || screen_y< 0 || screen_y > height) continue;
            //frame_buffer(screen_x, screen_y) = Color(255, 255, 255, 1);
		}
        
        // Sample triangles
        auto& triangles = obj.triangles;
        auto& pixel_points = obj.transformed_points;
        for (auto t = triangles.begin(); t != triangles.end(); ++t) {

            std::vector<Eigen::Vector4f> v({ 
                pixel_points[t->vert_ids[0]], 
                pixel_points[t->vert_ids[1]], 
                pixel_points[t->vert_ids[2]] 
            });

            t->screen_coordinates = v;

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

            Triangle tri = *t;
            for (int x = bb_left_x; x < bb_right_x; x++)
            {
                for (int y = bb_bottom_y; y < bb_top_y; y++)
                {
                    sample(x, y, xsamples, ysamples, frame_buffer, z_buffer, *t);   
                }
            }

            

        }
        

        //break;
        
	}

    pimage(frame_buffer);
}