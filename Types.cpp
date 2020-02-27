#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "Types.h"

Texture::Texture() {
    unsigned char* raw_data = stbi_load("textures\\earth_2.jpg", &w, &h, &n, 0);
    std::shared_ptr<unsigned char> data(raw_data);
    this->data = data;
}