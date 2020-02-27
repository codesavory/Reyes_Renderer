#pragma once
#include<Eigen/Eigen>
#include <numeric>
#include "Types.h"
#include "Shaders.h"

#define PI 3.141592653589793238462643383279502884197
#define MICROPOLYGONS_PER_PIXEL 1


class Primitive {
public:
    Eigen::Vector4f b_cube_vertices[8];
    Eigen::Matrix4f m;
    Eigen::Matrix4f v;
    Eigen::Matrix4f p;
    Eigen::Matrix4f mvp;
    int dice_factor;
    Color primitive_color;

    std::vector<Eigen::Vector4f> world_points;
    std::vector<Eigen::Vector4f> points;
    std::vector<Eigen::Vector4f> normals;
    std::vector<Color> vertex_colors;
    std::vector<TriangleVerts> triangle_verts;

    void (*geometric_shader)(GeometricShaderPayload& p) = nullptr;
    void (*surface_shader)(FragmentShaderPayload& p) = nullptr;

    void setup_vectors(int l) {
        world_points.resize(l);
        points.resize(l);
        normals.resize(l);
        vertex_colors.resize(l);
    }

    inline UVTuple get_uv(int id) {
        int u = id / dice_factor;
        int v = id - (u * dice_factor);;

        int grid_width = dice_factor;
        int grid_height = dice_factor;

        float u_n = (float)u / ((float)grid_width - 1);
        float v_n = (float)v / ((float)grid_height - 1);

        UVTuple p;
        p.u = u_n;
        p.v = v_n;
        return p;
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


    virtual void get_bounding_cube() = 0;
    virtual void get_3D_coordinates(float u_n, float v_n, Eigen::Vector4f& pos, Eigen::Vector4f& norm) = 0;


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


    void generate_mesh_points(int grid_width, int grid_height) {

        int i = 0;
        for (int u = 0; u < grid_width; ++u) {
            for (int v = 0; v < grid_height; ++v) {
                
                float u_n = (float)u / ((float)grid_width - 1);
                float v_n = (float)v / ((float)grid_height - 1);

                Eigen::Vector4f position;
                Eigen::Vector4f normal;

                get_3D_coordinates(u_n, v_n, position, normal);
                //normal = position.normalized();
                /*position.w() = 0.0f;
                 = position.normalized();
                normal.w() = 1.0f;
                position.w() = 1.0f;*/

                points[i] = position;
                normals[i] = normal;
                ++i;
            }
        }

    }

    void generate_polygons() {
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

    void build(int width, int height) {

        get_bounding_cube();
        dice_factor = get_dice_factor(width, height);
        //dice_factor = 100;
        setup_vectors(dice_factor * dice_factor);
        generate_mesh_points(dice_factor, dice_factor);
        generate_polygons();
        set_primitive_color();
        //texture_map();
        //shade();

    }

    void set_primitive_color() {

        vertex_colors.resize(points.size());
        for (int i = 0; i < points.size(); ++i) {
            vertex_colors[i] = primitive_color;
        }

    }

    void sh() {
        if (geometric_shader == nullptr)
            return;

        GeometricShaderPayload p;
        p.dice_factor = dice_factor;
        p.normals = normals;
        p.points = points;
        p.triangle_verts = triangle_verts;

        geometric_shader(p);

        normals = p.normals;
        points = p.points;
        triangle_verts = p.triangle_verts;
    }
    
    
};


class Sphere : public Primitive {
public:
    float radius;
    float phimin, phimax, thetamax;

    Sphere(float r, float zmin, float zmax, float tmax) {
        this->radius = r;

        this->phimin = (zmin > -radius) ? asin(zmin / radius) : -1 * PI / 2;
        this->phimax = (zmax < radius) ? asin(zmax / radius) : PI / 2;
        this->thetamax = tmax * PI / 180.0f;
    }

    virtual void get_bounding_cube() {
        // top plane
        b_cube_vertices[0] = Eigen::Vector4f(-1 * radius, -1 * radius, radius, 1.0f);
        b_cube_vertices[1] = Eigen::Vector4f(radius, -1 * radius, radius, 1.0f);

        b_cube_vertices[2] = Eigen::Vector4f(radius, radius, radius, 1.0f);
        b_cube_vertices[3] = Eigen::Vector4f(-1 * radius, radius, radius, 1.0f);

        // bottom plane
        b_cube_vertices[4] = Eigen::Vector4f(-1 * radius, -1 * radius, -1 * radius, 1.0f);
        b_cube_vertices[5] = Eigen::Vector4f(radius, -1 * radius, -1 * radius, 1.0f);

        b_cube_vertices[6] = Eigen::Vector4f(radius, radius, -1 * radius, 1.0f);
        b_cube_vertices[7] = Eigen::Vector4f(-1 * radius, radius, -1 * radius, 1.0f);
    }

    virtual void get_3D_coordinates(float u_n, float v_n, Eigen::Vector4f& position, Eigen::Vector4f& normal) {

        float phi = phimin + v_n * (phimax - phimin);
        float theta = u_n * thetamax;

        position.x() = radius * std::cos(theta) * std::cos(phi);
        position.y() = radius * std::sin(theta) * std::cos(phi);
        position.z() = radius * std::sin(phi);

        position.w() = 0.0f;
        normal = position.normalized();
        position.w() = 1.0f;
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

    virtual void get_bounding_cube() {
        b_cube_vertices[0] = Eigen::Vector4f(radius, radius, zmin, 1.0f);
        b_cube_vertices[1] = Eigen::Vector4f(-1*radius, radius, zmin, 1.0f);
        b_cube_vertices[2] = Eigen::Vector4f(-1 * radius, -1*radius, zmin, 1.0f);
        b_cube_vertices[3] = Eigen::Vector4f(radius, -1*radius, zmin, 1.0f);

        b_cube_vertices[4] = Eigen::Vector4f(radius, radius, zmax, 1.0f);
        b_cube_vertices[5] = Eigen::Vector4f(-1 * radius, radius, zmax, 1.0f);
        b_cube_vertices[6] = Eigen::Vector4f(-1 * radius, -1 * radius, zmax, 1.0f);
        b_cube_vertices[7] = Eigen::Vector4f(radius, -1 * radius, zmax, 1.0f);
    }

    virtual void get_3D_coordinates(float u_n, float v_n, Eigen::Vector4f& position, Eigen::Vector4f& normal) {
        
        float theta = u_n * thetamax;
        position.x() = radius * cos(theta);
        position.y() = radius * sin(theta);
        position.z() = zmin + v_n * (zmax - zmin);
        position.w() = 1.0f;

        normal.x() = position.x();
        normal.y() = position.y();
        normal.z() = 0.0f;
        normal.w() = 0.0f;
        normal.normalize();
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

    
    virtual void get_bounding_cube() {
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

    
    virtual void get_3D_coordinates(float u_n, float v_n, Eigen::Vector4f& position, Eigen::Vector4f& normal) {

        float phi = phi_min + v_n * (phi_max - phi_min);
        float theta = u_n * theta_max;
        float r = minor_r * std::cos(phi);

        position.x() = (major_r + r) * std::cos(theta);
        position.y() = (major_r + r) * std::sin(theta);
        position.z() = minor_r * std::sin(phi);
        position.w() = 1.0f;

        Eigen::Vector4f t(major_r * std::cos(theta), major_r * std::sin(theta), 0.0f, 0.0f);
        normal = (position - t).normalized();

    }

};


class Cone : public Primitive {
public:
    float height, radius;
    float thetamax;

    Cone(float height, float radius, float tmax) {
        this->radius = radius;
        this->height = height;
        this->thetamax = tmax * PI / 180.0f;
    }

    virtual void get_bounding_cube() {
        b_cube_vertices[0] = Eigen::Vector4f(radius, radius, 0.0f, 1.0f);
        b_cube_vertices[1] = Eigen::Vector4f(-1 * radius, radius, 0.0f, 1.0f);
        b_cube_vertices[2] = Eigen::Vector4f(-1 * radius, -1 * radius, 0.0f, 1.0f);
        b_cube_vertices[3] = Eigen::Vector4f(radius, -1 * radius, 0.0f, 1.0f);

        b_cube_vertices[4] = Eigen::Vector4f(radius, radius, height, 1.0f);
        b_cube_vertices[5] = Eigen::Vector4f(-1 * radius, radius, height, 1.0f);
        b_cube_vertices[6] = Eigen::Vector4f(-1 * radius, -1 * radius, height, 1.0f);
        b_cube_vertices[7] = Eigen::Vector4f(radius, -1 * radius, height, 1.0f);
    }

    virtual void get_3D_coordinates(float u_n, float v_n, Eigen::Vector4f& position, Eigen::Vector4f& normal) {

        float theta = u_n * thetamax;

        position.x() = radius * (1 - v_n) * std::cos(theta);
        position.y() = radius * (1 - v_n) * std::sin(theta);
        position.z() = v_n * height;
        position.w() = 1.0f;

        //vector from center to position
        Eigen::Vector4f V(position.x(), position.y(), 0.0f, 1.0f);
        V.normalize();
        normal.x() = V.x() * height / radius;
        normal.y() = V.y() * height / radius;
        normal.z() = radius / height;
        normal.w() = 0.0f;
        normal.normalize();

    }
};


class Patch : public Primitive {
public:
    std::vector<Eigen::Vector3f> control_points;

    
    Patch(std::vector<Eigen::Vector3f> cp, float scale) {
        this->control_points = cp;
        for (int i = 0; i < control_points.size(); ++i)
            control_points[i] *= scale;
    }

    virtual void get_bounding_cube() {

        Eigen::Vector3f l;
        l << std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity();

        for (int i = 0; i < 16; ++i) {
            auto p = control_points[i];

            if (p.x() < l.x())
                l.x() = p.x();

            if (p.y() < l.y())
                l.y() = p.y();

            if (p.z() < l.z())
                l.z() = p.z();
        }

        Eigen::Vector3f h;
        h << -1 * std::numeric_limits<float>::infinity(),
            -1 * std::numeric_limits<float>::infinity(),
            -1 * std::numeric_limits<float>::infinity();

        for (int i = 0; i < 16; ++i) {
            auto p = control_points[i];

            if (p.x() > h.x())
                h.x() = p.x();

            if (p.y() > h.y())
                h.y() = p.y();

            if (p.z() > h.z())
                h.z() = p.z();
        }

        b_cube_vertices[0] = Eigen::Vector4f(l.x(), l.y(), l.z(), 1.0f);
        b_cube_vertices[1] = Eigen::Vector4f(l.x(), h.y(), l.z(), 1.0f);
        b_cube_vertices[2] = Eigen::Vector4f(h.x(), l.y(), l.z(), 1.0f);
        b_cube_vertices[3] = Eigen::Vector4f(h.x(), h.y(), l.z(), 1.0f);

        b_cube_vertices[4] = Eigen::Vector4f(l.x(), l.y(), h.z(), 1.0f);
        b_cube_vertices[5] = Eigen::Vector4f(l.x(), h.y(), h.z(), 1.0f);
        b_cube_vertices[6] = Eigen::Vector4f(h.x(), l.y(), h.z(), 1.0f);
        b_cube_vertices[7] = Eigen::Vector4f(h.x(), h.y(), h.z(), 1.0f);
    }

    inline Eigen::Vector3f eval_curve(Eigen::Vector3f cp[4], float t) {

        Eigen::Vector3f P01 = (1 - t) * cp[0] + t * cp[1];
        Eigen::Vector3f P12 = (1 - t) * cp[1] + t * cp[2];
        Eigen::Vector3f P23 = (1 - t) * cp[2] + t * cp[3];

        Eigen::Vector3f a = (1 - t) * P01 + t * P12;
        Eigen::Vector3f b = (1 - t) * P12 + t * P23;

        return ((1 - t) * a) + (t * b);

    }

    Eigen::Vector3f eval_surface(float u, float v) {

        Eigen::Vector3f ucurve_cp[4];
        for (int i = 0; i < 4; ++i) {
            // select 4 points for a curve
            Eigen::Vector3f curve[4];
            int k = i * 4;
            curve[0] = control_points[k];
            curve[1] = control_points[k + 1];
            curve[2] = control_points[k + 2];
            curve[3] = control_points[k + 3];
            ucurve_cp[i] = eval_curve(curve, u);
        }

        auto z = eval_curve(ucurve_cp, v);
        return z;
    }

    Eigen::Vector3f dU(float u, float v) {

        Eigen::Vector3f P[4];
        Eigen::Vector3f v_curve[4];
        for (int i = 0; i < 4; ++i) {
            P[0] = control_points[i];
            P[1] = control_points[4 + i];
            P[2] = control_points[8 + i];
            P[3] = control_points[12 + i];
            v_curve[i] = eval_curve(P, v);
        }

        return -3 * (1 - u) * (1 - u) * v_curve[0] +
            (3 * (1 - u) * (1 - u) - 6 * u * (1 - u)) * v_curve[1] +
            (6 * u * (1 - u) - 3 * u * u) * v_curve[2] +
            3 * u * u * v_curve[3];
    }

    Eigen::Vector3f dV(float u, float v)
    {
        Eigen::Vector3f u_curve[4];
        for (int i = 0; i < 4; ++i) {

            Eigen::Vector3f curve[4];
            int k = i * 4;
            curve[0] = control_points[k];
            curve[1] = control_points[k + 1];
            curve[2] = control_points[k + 2];
            curve[3] = control_points[k + 3];
            u_curve[i] = eval_curve(curve, u);

        }

        return -3 * (1 - v) * (1 - v) * u_curve[0] +
            (3 * (1 - v) * (1 - v) - 6 * v * (1 - v)) * u_curve[1] +
            (6 * v * (1 - v) - 3 * v * v) * u_curve[2] +
            3 * v * v * u_curve[3];
    }

    virtual void get_3D_coordinates(float u_n, float v_n, Eigen::Vector4f& position, Eigen::Vector4f& normal) {
        Eigen::Vector3f p = eval_surface(u_n, v_n);
        position.x() = p.x();
        position.y() = p.y();
        position.z() = p.z();
        position.w() = 1.0f;

        Eigen::Vector3f dU_vec = dU(u_n, v_n);
        Eigen::Vector3f dV_vec = dV(u_n, v_n);
        Eigen::Vector3f n = dU_vec.cross(dV_vec);
        normal.x() = n.x();
        normal.y() = n.y();
        normal.z() = n.z();
        normal.w() = 0.0f;

        
        /*normal.x() = p.x();
        normal.y() = p.y();
        normal.z() = p.z();
        normal.w() = 0.0f;
        normal.normalize();*/
    }
};
