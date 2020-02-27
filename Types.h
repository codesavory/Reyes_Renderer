#pragma once
#include <Eigen/Eigen>


inline float clamp(float v, float min = 0.0f, float max = 1.0f) {
    return std::max(min, std::min(v, max));
}


class Color {
private:
    float fr, fg, fb;

public:
    Color() { 
        fr = fg = fb = 0.0f;
    }

    Color(uint8_t ir, uint8_t ig, uint8_t ib) {
        fr = (float)ir / 255.0f;
        fg = (float)ig / 255.0f;
        fb = (float)ib / 255.0f;
    }

    Color(float r, float g, float b) {
        this->set(r, g, b);
    }

    void set(float r, float g, float b) {
        this->fr = clamp(r);
        this->fg = clamp(g);
        this->fb = clamp(b);
    }

    void scale(float t) {
        t = clamp(t);
        fr *= t;
        fg *= t;
        fb *= t;
    }

    float r() {
        return fr;
    }

    float g() {
        return fg;
    }

    float b() {
        return fb;
    }

    uint8_t ir() {
        return (uint8_t)std::floorf(fr * 255.0f);
    }

    uint8_t ig() {
        return (uint8_t)std::floorf(fg * 255.0f);
    }

    uint8_t ib() {
        return (uint8_t)std::floorf(fb * 255.0f);
    }

    Color operator + (Color& t) {
        float zr = t.r() + fr;
        float zg = t.g() + fg;
        float zb = t.b() + fb;

        Color z;
        z.set(zr, zg, zb);
        return z;
    }
};

inline Color operator * (float t, Color c) {
    c.scale(t);
    return c;
}

inline Color operator * (Color c, float t) {
    c.scale(t);
    return c;
}


struct TriangleVerts {
    int ids[3];
};


struct UVTuple {
    float u;
    float v;
};


class Triangle {
public:
    Color colors[3];
    Eigen::Vector4f world_coordinates[3];
    Eigen::Vector4f screen_coordinates[3];
    Eigen::Vector4f normals[3];
    UVTuple uv_tups[3];
};


class Texture {
public:
    std::shared_ptr<unsigned char> data;
    int w, h, n;

    Texture();

    Color get_at(float u, float v) {

        int img_x = u * (w);
        int img_y = v * (h);

        img_x = img_x % w;
        img_y = img_y % h;

        // we want image processing to start from top left corner
        // instead of bottom left corner. So flipping x and y in index value
        int id = (img_y * w + img_x) * 3;
        unsigned char* rd = data.get();
        uint8_t r = rd[id];
        uint8_t g = rd[id + 1];
        uint8_t b = rd[id + 2];

        return Color(r, g, b);
    }
};


struct VertexShaderPayload {
    float u,v;
    Color c;
};


struct FragmentShaderPayload {
    Eigen::Vector4f pos;
    Eigen::Vector4f normal;
    float u, v;
    Color c;

    std::shared_ptr<Texture> texture;
};

struct GeometricShaderPayload {
    int dice_factor;
    std::vector<Eigen::Vector4f> points;
    std::vector<Eigen::Vector4f> normals;
    std::vector<TriangleVerts> triangle_verts;
};