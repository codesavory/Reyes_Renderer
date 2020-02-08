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


float area(int x1, int y1, int x2, int y2, int x3, int y3)
{
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
}

bool isInside(float x, float y, const Eigen::Vector4f* _v)
{
    float ar = area(_v[0].x(), _v[0].y(), _v[1].x(), _v[1].y(), _v[2].x(), _v[2].y());

    float ar1 = area(x, y, _v[1].x(), _v[1].y(), _v[2].x(), _v[2].y());
    float ar2 = area(_v[0].x(), _v[0].y(), x, y, _v[2].x(), _v[2].y());
    float ar3 = area(_v[0].x(), _v[0].y(), _v[1].x(), _v[1].y(), x, y);
    bool res = (ar == ar1 + ar2 + ar3);
    return ar;

}




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
    }

    T operator() (int x, int y) const {
        //int id = (y * xsamples * ysamples * w + x * xsamples * ysamples + n * xsamples + m);
        int id = (y * xsamples * ysamples * w + x * xsamples * ysamples + 0);
        return buff[id];
    }

    T& operator() (int x, int y) {
        //int id = (y * xsamples * ysamples * w + x * xsamples * ysamples + n * xsamples + m);
        int id = (y * xsamples * ysamples * w + x * xsamples * ysamples + 0);
        return buff[id];
    }
};


class FrameBuffer : public Buffer<Color> {
public:
    FrameBuffer(int width, int heigth, int xsamples, int ysamples)
        : Buffer(width, heigth, xsamples, ysamples) {}


    void sample(int x, int y, std::function< std::tuple<bool, Color>(float, float) > checker) {

        Eigen::Vector3f pixel_color(0, 0, 0);
        for (float m = 0; m < xsamples; ++m) {
            for (float n = 0; n < ysamples; ++n) {

                float mMin = (float)m / xsamples;
                float nMin = (float)n / ysamples;
                float mMax = (float)(m + 1) / xsamples;
                float nMax = (float)(n + 1) / ysamples;

                float x_delta = ((float)rand() / RAND_MAX) * (mMax - mMin) + mMin;;
                float y_delta = ((float)rand() / RAND_MAX) * (nMax - nMin) + nMin;

                float sample_x = x + x_delta;
                float sample_y = y + y_delta;

                bool is_inside;
                Color color;
                std::tie(is_inside, color) = checker(sample_x, sample_y);
                if (is_inside) {
                    pixel_color += Eigen::Vector3f(color.r, color.g, color.b);
                }
            }
        }

        pixel_color /= (xsamples * ysamples);
        Color final(pixel_color[0], pixel_color[1], pixel_color[2], 1);
        this -> operator()(x, y) = final;

    }


    void sample2(int x, int y, std::function< std::tuple<bool, Color>(float, float) > checker) {

        // Sample corners first
        float xdelta = (1 / xsamples) * 0.5f;
        float ydelta = (1 / ysamples) * 0.5f;

        float corner_points[4][2] = {
            {x + xdelta, y + ydelta},               // bottom_left
            {x + (1 - xdelta), y + ydelta},         // top left
            {x + (1 - xdelta), y + (1 - ydelta)},   // top right
            {x + xdelta, y + (1 - ydelta)}          // bottom rihjt
        };

        bool all_corners = true;
        Color pix_color;
        for (int i = 0; i < 4; ++i) {
            
            bool is_inside;
            Color color;
            std::tie(is_inside, color) = checker(corner_points[i][0], corner_points[i][1]);
            all_corners = all_corners & is_inside;
            pix_color = color;
            
        }

        if (all_corners) {
            this -> operator()(x, y) = pix_color;
            return;
        }



    }
};



class ZBuffer : public Buffer<std::vector<float>> {
public:
    ZBuffer(int width, int heigth, int xsamples, int ysamples)
        : Buffer(width, heigth, xsamples, ysamples) {}
};


void pimage(Buffer<Color> frame_buffer) {
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
            out_buffer[index++] = frame_buffer(x, y).r;
            out_buffer[index++] = frame_buffer(x, y).g;
            out_buffer[index++] = frame_buffer(x, y).b;
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

            Eigen::Vector4f v[3] = { 
                pixel_points[t->vert_ids[0]], 
                pixel_points[t->vert_ids[1]], 
                pixel_points[t->vert_ids[2]] 
            };

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

            //std::cout << "\n --- " << bb_left_x << "," << bb_bottom_y << "," << bb_right_x << "," << bb_top_y << "\n";

            Color c = t->color;
            for (int x = bb_left_x; x < bb_right_x; x++)
            {
                for (int y = bb_bottom_y; y < bb_top_y; y++)
                {
                    
                    /*float sample_x = x + 0.5;
                    float sample_y = y + 0.5;

                    int screen_x = (int)floor(sample_x);
                    int screen_y = (int)floor(sample_y);
                    if (isInside(sample_x, sample_y, v)) {
                        frame_buffer(screen_x, screen_y, 0, 0) = t->color;
                    }*/
                    
                    

                    
                    auto checker = [v,c](float sample_x, float sample_y) {
                        bool inside = isInside(sample_x, sample_y, v); 
                        return std::tuple<bool, Color>{inside, c};
                    };

                    frame_buffer.sample(x, y, checker);
                    
                    
                    
                }
            }

            

        }
        

        //break;
        
	}

    pimage(frame_buffer);
}