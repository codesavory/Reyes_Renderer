#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "Types.h"

Texture::Texture(std::string filepath) {
    unsigned char* raw_data = stbi_load(filepath.c_str(), &w, &h, &n, 0);
    std::shared_ptr<unsigned char> data(raw_data);
    this->data = data;
}