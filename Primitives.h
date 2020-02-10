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
    std::vector<int> vert_ids;
    std::vector<Color> color;
    std::vector<Eigen::Vector4f> screen_coordinates;

    int ids[3];
};

class Mesh {
public:
    float width, heigth;
    float dice_factor;
    std::vector<Eigen::Vector4f> points;
    //std::vector<Eigen::Vector4f> normals;
    //std::vector<Color> vertex_colors;
    std::vector<Triangle> triangles;
};

class Primitive {
public:
    std::vector<Eigen::Vector4f> vertices;
    std::vector<Triangle> triangles;
    Eigen::Matrix4f model_to_world_matrix;
    std::vector<Eigen::Vector4f> transformed_points;

    Eigen::Vector4f b_cube_vertices[8];

    int get_dice_factor(Eigen::Matrix4f mvp, int width, int height) {

        float bb_left_x = std::numeric_limits<float>::infinity();
        float bb_right_x = -1 * std::numeric_limits<float>::infinity();
        float bb_top_y = -1 * std::numeric_limits<float>::infinity(); 
        float bb_bottom_y = std::numeric_limits<float>::infinity();

        for (int i = 0; i < 8; ++i) {
            auto p = b_cube_vertices[i];
            p = mvp * p;
            p.x() = p.x() / p.w();
            p.y() = p.y() / p.w();
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
};

class Sphere : public Primitive {
public:
    float radius;
    Color primitive_color;

    Sphere(float r, Color color) {
        radius = r;
        primitive_color = color;
        build_sphere();
    }

    void add_triangle(int a, int b, int c) {
        Triangle t;
        t.vert_ids = { a,b,c };
        t.color = { primitive_color, primitive_color, primitive_color };
        triangles.push_back( t );
    }

    void add_quad(int a, int b, int c, int d) {
        Triangle p;
        p.vert_ids = { a,b,c };
        p.color = { primitive_color, primitive_color, primitive_color };
        triangles.push_back( p );

        Triangle q;
        q.vert_ids = { a,c,d };
        q.color = { primitive_color, primitive_color, primitive_color };
        triangles.push_back( q );
    }

    
    void build_sphere() {

        int parallels = 20;
        int meridians = 20;
        float const r = radius;

        vertices.push_back(Eigen::Vector4f(0.0f, r, 0.0f, 1.0f));
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
                vertices.push_back(p);
            }
        }
        vertices.push_back(Eigen::Vector4f(0.0f, -1 * r, 0.0f, 1.0f));
        

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
            add_triangle(vertices.size() - 1, a, b);
        }
        
    }
    
};


//std::vector<Eigen::Vector4f> sphere_mesh(double radius)
//{
//    std::vector<Eigen::Vector4f> points;
//    int parallels = 25;
//    int meridians = 25;
//    double r = radius;
//    //points.push_back(Eigen::Vector4f(0.0f, r, 0.0f));
//    for (int j = 0; j < parallels - 1; ++j)
//    {
//        double const polar = PI * double(j + 1) / double(parallels);
//        double const sp = std::sin(polar);
//        double const cp = std::cos(polar);
//        for (int i = 0; i < meridians; ++i)
//        {
//            double const azimuth = 2.0 * PI * double(i) / double(meridians);
//            double const sa = std::sin(azimuth);
//            double const ca = std::cos(azimuth);
//            float const x = r * sp * ca;
//            float const y = r * cp;
//            float const z = r * sp * sa;
//            Eigen::Vector4f p;
//            p[0] = x;
//            p[1] = y;
//            p[2] = z;
//            p[3] = 1.0;
//            points.push_back(p);
//        }
//    }
//    //points.push_back(Eigen::Vector4f(0.0f, -1 * r, 0.0f));
//    return points;
//}


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