#pragma once
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <Eigen/Eigen>
#include "States.h"


int save_image(std::vector<Eigen::Vector4f> ndc_points, image_state image_state) {
    const int width = image_state.x_resolution;
    const int height = image_state.y_resolution;
    const char* filename = image_state.filename;
    const int CHANNEL_NUM = 3;

    //grey background
    uint8_t* framebuffer = new uint8_t[width * height * CHANNEL_NUM];

    // Set background color
    // For the following framebuffer, (0,0) pixel is at the top left
    int index = 0;
    for (int i = 0; i < width; ++i)
    {
        for (int j = 0; j < height; ++j)
        {
            int r = 255;
            int g = 255;
            int b = 0;

            framebuffer[index++] = r;
            framebuffer[index++] = g;
            framebuffer[index++] = b;
        }
    }

    for (int i = 0; i < ndc_points.size(); ++i) {
        Eigen::Vector4f ndc_point = ndc_points[i];

        //Any point outside of NDC space chuck it.
        if (ndc_point.x() < -1 || ndc_point.x() > 1 || ndc_point.y() < -1 || ndc_point.y() > 1) continue;
        //Convert ndc coords to image coords
        //(-1,-1) ndc coordinates will be converted to (0,0) in image space
        int x = std::min(width - 1, (int)((ndc_point.x() + 1) * 0.5 * width));
        int y = std::min(height - 1, (int)((1 - (ndc_point.y() + 1) * 0.5) * height));

        // Since x and y are coordinates where origin is at the bottom left and the framebuffer
        // has the origin at top left, we have to transform the obtained pixel coordinates. 
        // This is essentially a 90 degree rotation which is the same as flipping the coordinates
        index = (y * width * 3) + x * 3;
        int r = 0;
        int g = 0;
        int b = 0;
        framebuffer[index++] = r;
        framebuffer[index++] = g;
        framebuffer[index++] = b;
    }

    stbi_write_jpg(filename, width, height, 3, framebuffer, 100);
    delete[] framebuffer;
    return 0;
}