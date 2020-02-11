#pragma once
#include "Ri.h"
#include<Eigen/Eigen>

#define PI 3.141592653589793238462643383279502884197
#define MICROPOLYGONS_PER_PIXEL 2

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
    std::vector<int>u_vals;
    std::vector<int>v_vals;
    

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

    virtual void build(int width, int height){}

    void apply_checker_shade() {
        float CHECK_SIZE_X = 10.0f;
        float CHECK_SIZE_Y = 10.0f;

        for (int i = 0; i < points.size(); ++i) {
            auto u = (float)u_vals[i] / (float)(dice_factor - 1);
            auto v = (float)v_vals[i] / (float)(dice_factor - 1);

            if ((int)(floor(u * CHECK_SIZE_X) + floor(v * CHECK_SIZE_Y)) % 2 == 0) {
                vertex_colors[i] = Color(255.0f, 255.0f, 255.0f, 1.0f);
            }
            else {
                vertex_colors[i] = Color(0.0f, 0.0f, 0.0f, 1.0f);
            }
  
        }
    }
};


class Sphere : public Primitive {
public:
    float radius;

    Sphere(float r, Color color) {
        radius = r;
        primitive_color = color;
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

        points.resize(dicefactor * dicefactor + 2);
        u_vals.resize(dicefactor * dicefactor + 2);
        v_vals.resize(dicefactor * dicefactor + 2);
        vertex_colors.resize(dicefactor * dicefactor + 2);

        int pid = 0;

        points[pid] = Eigen::Vector4f(0.0f, r, 0.0f, 1.0f);
        u_vals[pid] = 0;
        v_vals[pid] = 0;
        ++pid;
        

        for (int j = 0; j < parallels - 1; ++j)
        {
            double const polar = PI * double(j + 1) / double(parallels);
            double const sp = std::sin(polar);
            double const cp = std::cos(polar);
            for (int i = 0; i < meridians; ++i)
            {
                double const azimuth = 2.0 * PI * double(i) / double(meridians);
                double const sa = std::sin(azimuth);
                double const ca = std::cos(azimuth);
                float const x = r * sp * ca;
                float const y = r * cp;
                float const z = r * sp * sa;
                Eigen::Vector4f p;
                p[0] = x;
                p[1] = y;
                p[2] = z;
                p[3] = 1.0f;
                points[pid] = p;
                u_vals[pid] = j;
                v_vals[pid] = i;
                ++pid;
                
            }
            
        }

        points[pid] = Eigen::Vector4f(0.0f, -1 * r, 0.0f, 1.0f);
        u_vals[pid] = u_vals[pid - 1];
        v_vals[pid] = v_vals[pid - 1];

        for (int i = 0; i < meridians; ++i)
        {
            int const a = i + 1;
            int const b = (i + 1) % meridians + 1;
            add_triangle(0, b, a);
        }

        for (int j = 0; j < parallels - 2; ++j)
        {
            int aStart = j * meridians + 1;
            int bStart = (j + 1) * meridians + 1;
            for (int i = 0; i < meridians; ++i)
            {
                int const a = aStart + i;
                int const a1 = aStart + (i + 1) % meridians;
                int const b = bStart + i;
                int const b1 = bStart + (i + 1) % meridians;
                add_quad(a, a1, b1, b);
            }
        }

        for (int i = 0; i < meridians; ++i)
        {
            int const a = i + meridians * (parallels - 2) + 1;
            int const b = (i + 1) % meridians + meridians * (parallels - 2) + 1;
            add_triangle(points.size() - 1, a, b);
        }

        set_primitive_color();
    }   

};

//class Torus : public Primitive {
//public:
//    float major_r;
//    float minor_r;
//    int minor_segments;
//    int major_segments;
//
//    Torus(float major_r, float minor_r, int major_segments, int minor_segments) {
//        this->major_r = major_r;
//        this->minor_r = minor_r;
//        this->major_segments = major_segments;
//        this->minor_segments = minor_segments;
//    }
//};
//std::vector<Eigen::Vector4f> generate_torus_mesh(double majorRadius, double minorRadius, double minorSegments = 10, double majorSegments = 10)
//{
//    std::vector<Eigen::Vector4f> points;
//    // minor radius is the thickness of the torus / 2
//    auto PI2 = PI * 2;
//    auto arc = PI * 2;
//
//    //Eigen::Vector3f center = { 0, 0, 0 };
//
//    for (double j = 0; j < minorSegments; ++j) {
//        for (double i = 0; i < majorSegments; ++i) {
//            double const u = i / majorSegments * arc;
//            double const v = j / minorSegments * PI2;
//
//            //center[0] = majorRadius * std::cos(u);
//            //center[1] = majorRadius * std::sin(u);
//
//            float const x = (majorRadius + minorRadius * std::cos(v)) * std::cos(u);
//            float const y = (majorRadius + minorRadius * std::cos(v)) * std::sin(u);
//            float const z = minorRadius * std::sin(v);
//
//            Eigen::Vector4f vertex;
//            vertex[0] = x;
//            vertex[1] = y;
//            vertex[2] = z;
//            vertex[3] = 1;
//
//            points.push_back(vertex);
//        }
//    }
//    return points;
//}