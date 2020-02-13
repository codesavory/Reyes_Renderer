#pragma once
#ifndef STB_IMAGE_IMPLEMENTATION
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#endif
#include<Eigen/Eigen>

#define PI 3.141592653589793238462643383279502884197
#define MICROPOLYGONS_PER_PIXEL 1



class Color {
public:
    int r, g, b, a;
    Color() { r = g = b = a = 0; }
    Color(int r, int g, int b, int a) {
        r = std::max(0, std::min(r, 255));
        g = std::max(0, std::min(g, 255));
        b = std::max(0, std::min(b, 255));

        this->r = r;
        this->g = g;
        this->b = b;
        this->a = a;
    }

    Color(float r, float g, float b, float a) {
        int ir = (int)std::floorf(r);
        int ig = (int)std::floorf(g);
        int ib = (int)std::floorf(b);
        int ia = (int)std::floorf(a);

        ir = std::max(0, std::min(ir, 255));
        ig = std::max(0, std::min(ig, 255));
        ib = std::max(0, std::min(ib, 255));

        this->r = ir;
        this->g = ig;
        this->b = ib;
        this->a = ia;
    }
};


class Triangle {
public:
    Color colors[3];
    Eigen::Vector4f screen_coordinates[3];
    int u_vals[3];
    int v_vals[3];
};


struct TriangleVerts {
    int ids[3];
};

struct UVPoint {
    int u;
    int v;
};


class Primitive {
public:
    Eigen::Vector4f b_cube_vertices[8];
    Eigen::Matrix4f mvp;
    int dice_factor;
    Color primitive_color;

    std::vector<Eigen::Vector4f> points;
    std::vector<Eigen::Vector4f> normals;
    std::vector<Color> vertex_colors;
    std::vector<TriangleVerts> triangle_verts;


    void set_primitive_color() {
        vertex_colors.resize(points.size());
        for (int i = 0; i < points.size(); ++i) {
            vertex_colors[i] = primitive_color;
        }
    }

    void add_triangle(int a, int b, int c) {
        TriangleVerts t;
        t.ids[0] = a;
        t.ids[1] = b;
        t.ids[2] = c;
        triangle_verts.push_back(t);
    }

    void add_quad(int a, int b, int c, int d) {
        TriangleVerts p;
        p.ids[0] = a;
        p.ids[1] = b;
        p.ids[2] = c;

        TriangleVerts q;
        q.ids[0] = a;
        q.ids[1] = c;
        q.ids[2] = d;

        triangle_verts.push_back(p);
        triangle_verts.push_back(q);
    }

    void build_mesh() {
        for (int u = 0; u < dice_factor - 1; ++u)
        {
            int aStart = u * dice_factor;
            int bStart = (u + 1) * dice_factor;
            for (int v = 0; v < dice_factor - 1; ++v)
            {
                int const a = aStart + v;
                int const a1 = aStart + (v + 1);
                int const b = bStart + v;
                int const b1 = bStart + (v + 1);
                add_quad(a, a1, b1, b);
            }
        }
    }


    int get_dice_factor(int width, int height) {

        float bb_left_x = std::numeric_limits<float>::infinity();
        float bb_right_x = -1 * std::numeric_limits<float>::infinity();
        float bb_top_y = -1 * std::numeric_limits<float>::infinity(); 
        float bb_bottom_y = std::numeric_limits<float>::infinity();

        for (int i = 0; i < 8; ++i) {
            auto p = b_cube_vertices[i];
            p = mvp * p;
            p.x() = p.x() / p.w() * -1;
            p.y() = p.y() / p.w() * -1;
            p.z() = p.z() / p.w();

            float screen_x = (p.x() + 1.0f) * 0.5f * width;
            float screen_y = (1.0 - (p.y() + 1.0f) * 0.5f) * height;

            if (screen_x < bb_left_x)
                bb_left_x = screen_x;
            if (screen_x > bb_right_x)
                bb_right_x = screen_x;
            if (screen_y < bb_bottom_y)
                bb_bottom_y = screen_y;
            if (screen_y > bb_top_y)
                bb_top_y = screen_y;
        }

        //if it will be on the screen, how many pixels will it take up?
        int xFactor = (bb_right_x - bb_left_x) * MICROPOLYGONS_PER_PIXEL;
        int yFactor = (bb_top_y - bb_bottom_y) * MICROPOLYGONS_PER_PIXEL;

        //choose the largest dimension
        int dice_factor = std::max(xFactor, yFactor);
        //can't be higher than screen resolution*MICROPOLYGONS_PER_PIXEL
        dice_factor = std::min(dice_factor, std::max(width, height) * MICROPOLYGONS_PER_PIXEL * 2);
        return dice_factor;
    }

    virtual void build(int width, int height) = 0;

    inline UVPoint get_uv(int id) {
        int u = id / dice_factor;
        int v = id - (u * dice_factor);;

        UVPoint p;
        p.u = u;
        p.v = v;
        return p;
    }

    void apply_checker_shade() {
        float CHECK_SIZE_X = 10.0f;
        float CHECK_SIZE_Y = 10.0f;

        for (int i = 0; i < points.size(); ++i) {
            UVPoint p = get_uv(i);
            auto u = (float)p.u / (float)(dice_factor - 1);
            auto v = (float)p.v / (float)(dice_factor - 1);
            


            if ((int)(floor(u * CHECK_SIZE_X) + floor(v * CHECK_SIZE_Y)) % 2 == 0) {
                vertex_colors[i] = Color(255.0f, 255.0f, 255.0f, 1.0f);
            }
            else {
                vertex_colors[i] = Color(0.0f, 0.0f, 0.0f, 1.0f);
            }
  
        }
    }

    void texture_map() {
        int x, y, n;
        unsigned char* data = stbi_load("2k_earth_nightmap.jpg", &x, &y, &n, 0);

        for (int i = 0; i < points.size(); ++i) {
            UVPoint p = get_uv(i);
            auto u = (float)p.u / (float)(dice_factor - 1);
            auto v = (float)p.v / (float)(dice_factor - 1);

            int img_x = u * (x);    
            int img_y = v * (y);

            img_x = img_x % x;
            img_y = img_y % y;

            // we want image processing to start from top left corner
            // instead of bottom left corner. So flipping x and y in index value
            int id = (img_y * x + img_x)*3;
            int r = data[id];
            int g = data[id + 1];
            int b = data[id + 2];

            vertex_colors[i] = Color(r, g, b, 1.0);
        }

        stbi_image_free(data);
    }
    
};


class Sphere : public Primitive {
public:
    float radius;
    float phimin, phimax, thetamax;

    Sphere(float r, float zmin, float zmax, float tmax) {
        this -> radius = r;
        
        this->phimin = (zmin > -radius) ? asin(zmin / radius) : -1 * PI / 2;
        this->phimax = (zmax < radius) ? asin(zmax / radius) : PI / 2;
        this->thetamax = tmax * PI / 180.0f;
    }

    void get_bounding_cube() {
        // top plane
        b_cube_vertices[0] = Eigen::Vector4f(-1 * radius,   -1 * radius,    radius,         1.0f);
        b_cube_vertices[1] = Eigen::Vector4f(   radius,     -1 * radius,    radius,         1.0f);

        b_cube_vertices[2] = Eigen::Vector4f(   radius,     radius,     radius,     1.0f);
        b_cube_vertices[3] = Eigen::Vector4f(-1 * radius,   radius,     radius,     1.0f);

        // bottom plane
        b_cube_vertices[4] = Eigen::Vector4f(-1 * radius,   -1 * radius,    -1 * radius,    1.0f);
        b_cube_vertices[5] = Eigen::Vector4f(   radius,     -1 * radius,    -1 * radius,    1.0f);

        b_cube_vertices[6] = Eigen::Vector4f(   radius,         radius,     -1 * radius,    1.0f);
        b_cube_vertices[7] = Eigen::Vector4f(-1 * radius,       radius,     -1 * radius,    1.0f);
    }

    virtual void build(int width, int height) {
        get_bounding_cube();
        int dicefactor = get_dice_factor(width, height);
        this->dice_factor = dicefactor;

        int const parallels = dicefactor;
        int const meridians = dicefactor;
        float const r = radius;

        points.resize(dicefactor * dicefactor);
        vertex_colors.resize(dicefactor * dicefactor);

        int i = 0;
        for (int u = 0; u < dice_factor; ++u) {
            for (int v = 0; v < dice_factor; ++v) {
                Eigen::Vector4f position;

                float u_n = (float)u / ((float)dice_factor - 1);
                float v_n = (float)v / ((float)dice_factor - 1);

                float phi = phimin + v_n * (phimax - phimin);
                float theta = u_n * thetamax;

                position.x() = radius * std::cos(theta) * std::cos(phi);
                position.y() = radius * std::sin(theta) * std::cos(phi);
                position.z() = radius * std::sin(phi);
                position.w() = 1.0f;

                points[i] = position;
                ++i;
            }
        }
        /*
        for (int u = 0; u < dice_factor - 1; ++u)
        {
            int aStart = u * dice_factor;
            int bStart = (u + 1) * dice_factor;
            for (int v = 0; v < dice_factor - 1; ++v)
            {
                int const a = aStart + v;
                int const a1 = aStart + (v + 1);
                int const b = bStart + v;
                int const b1 = bStart + (v + 1);
                add_quad(a, a1, b1, b);
            }
        }
        */
        build_mesh();
        apply_checker_shade();
        //texture_map();
    }   

};


class Cylinder : public Primitive {
public:
    float radius, zmin, zmax, thetamax;

    Cylinder(float radius, float zmin, float zmax, float tmax) {
        this->radius = radius;
        this->zmax = zmax;
        this->zmin = zmin;

        //calculate necessary parameters (in radians)
        this->thetamax = tmax * PI / 180.0f;
    }

    void get_bounding_cube() {
        b_cube_vertices[0] = Eigen::Vector4f(radius, radius, zmin, 1.0f);
        b_cube_vertices[1] = Eigen::Vector4f(-1*radius, radius, zmin, 1.0f);
        b_cube_vertices[2] = Eigen::Vector4f(-1 * radius, -1*radius, zmin, 1.0f);
        b_cube_vertices[3] = Eigen::Vector4f(radius, -1*radius, zmin, 1.0f);

        b_cube_vertices[4] = Eigen::Vector4f(radius, radius, zmax, 1.0f);
        b_cube_vertices[5] = Eigen::Vector4f(-1 * radius, radius, zmax, 1.0f);
        b_cube_vertices[6] = Eigen::Vector4f(-1 * radius, -1 * radius, zmax, 1.0f);
        b_cube_vertices[7] = Eigen::Vector4f(radius, -1 * radius, zmax, 1.0f);
    }

    virtual void build(int width, int height) {
        get_bounding_cube();
        dice_factor = get_dice_factor(width, height);
        points.resize(dice_factor * dice_factor);
        vertex_colors.resize(dice_factor * dice_factor);

        int i = 0;
        for (int u = 0; u < dice_factor; ++u) {
            for (int v = 0; v < dice_factor; ++v) {
                Eigen::Vector4f position;
                
                float u_n = (float)u / ((float)dice_factor - 1);
                float v_n = (float)v / ((float)dice_factor - 1);

                float theta = u_n * thetamax;

                position.x() = radius * cos(theta);
                position.y() = radius * sin(theta);
                position.z() = zmin + v_n * (zmax - zmin);
                position.w() = 1.0f;

                points[i] = position;
                ++i;
            }
        }

        for (int u = 0; u < dice_factor-1; ++u)
        {
            int aStart = u * dice_factor;
            int bStart = (u + 1) * dice_factor;
            for (int v = 0; v < dice_factor-1; ++v)
            {
                int const a = aStart + v;
                int const a1 = aStart + (v + 1);
                int const b = bStart + v;
                int const b1 = bStart + (v + 1);
                add_quad(a, a1, b1, b);
            }
        }

        //set_primitive_color();
        apply_checker_shade();
    }
};

class Torus : public Primitive {
public:
    float major_r;
    float minor_r;

    float phi_min, phi_max, theta_max;

    Torus(float major_r, float minor_r, float pmin, float pmax, float tmax) {
        this->major_r = major_r;
        this->minor_r = minor_r;

        this->phi_min = pmin * PI / 180.0f;
        this->phi_max = pmax * PI / 180.0f;
        this->theta_max = tmax * PI / 180.0f;
    }

    
    void get_bounding_cube() {
        float big_r = major_r + minor_r;
        b_cube_vertices[0] = Eigen::Vector4f(big_r, big_r, -minor_r, 1.0f);
        b_cube_vertices[1] = Eigen::Vector4f(-1*big_r, big_r, -minor_r, 1.0f);
        b_cube_vertices[2] = Eigen::Vector4f(-1*big_r, -1*big_r, -minor_r, 1.0f);
        b_cube_vertices[3] = Eigen::Vector4f(big_r, -1*big_r, -minor_r, 1.0f);

        b_cube_vertices[4] = Eigen::Vector4f(big_r, big_r, minor_r, 1.0f);
        b_cube_vertices[5] = Eigen::Vector4f(-1 * big_r, big_r, minor_r, 1.0f);
        b_cube_vertices[6] = Eigen::Vector4f(-1 * big_r, -1 * big_r, minor_r, 1.0f);
        b_cube_vertices[7] = Eigen::Vector4f(big_r, -1 * big_r, minor_r, 1.0f);
    }
    

    virtual void build(int width, int height) {
        get_bounding_cube();
        dice_factor = get_dice_factor(width, height);
        points.resize(dice_factor * dice_factor);
        vertex_colors.resize(dice_factor * dice_factor);

        int i = 0;
        for (int u = 0; u < dice_factor; ++u) {
            for (int v = 0; v < dice_factor; ++v) {
                Eigen::Vector4f position;

                float u_n = (float)u / ((float)dice_factor - 1);
                float v_n = (float)v / ((float)dice_factor - 1);

                float phi = phi_min + v_n * (phi_max - phi_min);
                float theta = u_n * theta_max;
                float r = minor_r * std::cos(phi);


                position.x() = (major_r + r) * std::cos(theta);
                position.y() = (major_r + r) * std::sin(theta);
                position.z() = minor_r * std::sin(phi);
                position.w() = 1.0f;

                points[i] = position;
                ++i;
            }
        }

        for (int u = 0; u < dice_factor - 1; ++u)
        {
            int aStart = u * dice_factor;
            int bStart = (u + 1) * dice_factor;
            for (int v = 0; v < dice_factor - 1; ++v)
            {
                int const a = aStart + v;
                int const a1 = aStart + (v + 1);
                int const b = bStart + v;
                int const b1 = bStart + (v + 1);
                add_quad(a, a1, b1, b);
            }
        }

        set_primitive_color();

    }


};
