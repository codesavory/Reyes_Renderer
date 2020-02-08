#pragma once
#include "Ri.h"
#include<Eigen/Eigen>

#define PI 3.141592653589793238462643383279502884197

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
};

class Triangle {
public:
    std::vector<int> vert_ids;
    Color color;
};

class Primitive {
public:
    std::vector<Eigen::Vector4f> vertices;
    std::vector<Triangle> triangles;
    Eigen::Matrix4f model_to_world_matrix;
    std::vector<Eigen::Vector4f> transformed_points;

    // primitive color
    //Color color;
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
        t.color = primitive_color;
        triangles.push_back( t );
    }

    void add_quad(int a, int b, int c, int d) {
        Triangle p;
        p.vert_ids = { a,b,c };
        p.color = primitive_color;
        triangles.push_back( p );

        Triangle q;
        q.vert_ids = { a,c,d };
        q.color = primitive_color;
        triangles.push_back( q );
    }

    
    void build_sphere() {

        int parallels = 15;
        int meridians = 15;
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