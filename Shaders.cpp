#include<Eigen/Eigen>
#include "Types.h"
#include "Shaders.h"
#include <math.h>
#include <set>


void checkboard(VertexShaderPayload& pi) {
    float CHECK_SIZE_X = 10.0f;
    float CHECK_SIZE_Y = 10.0f;

    float u = pi.u;
    float v = pi.v;
    if ((int)(floor(u * CHECK_SIZE_X) + floor(v * CHECK_SIZE_Y)) % 2 == 0) {
        pi.c.set(255.0f, 255.0f, 255.0f);
    }
    else {
        pi.c.set(0.0f, 0.0f, 0.0f);
    }

}


inline UVTuple get_uv(int id, int dice_factor) {
    int u = id / dice_factor;
    int v = id - (u * dice_factor);

    int grid_width = dice_factor;
    int grid_height = dice_factor;

    float u_n = (float)u / ((float)grid_width - 1);
    float v_n = (float)v / ((float)grid_height - 1);

    UVTuple p;
    p.u = u_n;
    p.v = v_n;
    return p;
}

void checker_explode(GeometricShaderPayload& p) {
    float CHECK_SIZE_X = 60.0f;
    float CHECK_SIZE_Y = 60.0f;

    int dice_factor = p.dice_factor;
    auto& points = p.points;
    auto& normals = p.normals;
    auto& triangles = p.triangle_verts;
    std::vector<TriangleVerts> n_triangles;

    for (int i = 0; i < triangles.size(); ++i) {
        auto verts = triangles[i].ids;
        bool all_in = true;
        std::set<bool> type;

        for (int j = 0; j < 3; ++j) {
            int v_id = verts[j];
            UVTuple tup = get_uv(v_id, dice_factor);
            float u = tup.u;
            float v = tup.v;
            bool t = (int)(floor(u * CHECK_SIZE_X) + floor(v * CHECK_SIZE_Y)) % 2 == 0;
            float p = (int)floor(u * CHECK_SIZE_X) % 3;
            float q = (int)floor(v * CHECK_SIZE_Y) % 3;
            bool x = (p == 1) && (q == 1);
            type.emplace(x && t);
        }

        if (type.size() == 1) {
            bool f = *type.begin();
            if (f)
                n_triangles.push_back(triangles[i]);

        }
    }

    p.triangle_verts = n_triangles;
}

void earth(FragmentShaderPayload& p) {
    float u = p.u;
    float v = p.v;
    std::shared_ptr<Texture> t = p.texture;

    Color c = t->get_at(u, v);
    p.c = c;
}

void blinn_phong(FragmentShaderPayload& p) {

    Eigen::Vector3f ka(0.01, 0.01, 0.01);
    Eigen::Vector3f kd(p.c.r(), p.c.g(), p.c.b());
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f ambient = amb_light_intensity.cwiseProduct(ka);


    Eigen::Vector3f light_pos(0.0, 0.0, -45.0);
    Eigen::Vector3f light_intensity(600.0f, 600.0f, 600.0f);

    Eigen::Vector3f vert_pos(p.pos.x(), p.pos.y(), p.pos.z());
    Eigen::Vector3f normal(p.normal.x(), p.normal.y(), p.normal.z());
    Eigen::Vector3f light_dir = light_pos - vert_pos;
    float distance = light_dir.norm();
    distance = distance * distance;
    //light_dir.normalize();
    float lambertian = std::max(normal.dot(light_dir.normalized()), 0.0f);
    Eigen::Vector3f diffuse = kd.cwiseProduct((light_intensity / distance) * lambertian);


    Eigen::Vector3f view_dir = vert_pos * -1;
    view_dir.normalize();
    Eigen::Vector3f half_vec = (light_dir + view_dir) / ((light_dir + view_dir).norm());
    //half_vec.normalize();
    float _p = 150;
    float spec_lambertian = pow(std::max(normal.dot(half_vec), 0.0f), _p);
    Eigen::Vector3f specular = ks.cwiseProduct((light_intensity / distance) * spec_lambertian);


    Eigen::Vector3f result_color(0.0f, 0.0f, 0.0f);
    result_color += ambient + diffuse + specular;

    p.c.set(result_color[0], result_color[1], result_color[2]);
}

void blinn_phong_modded(FragmentShaderPayload& p) {

    Eigen::Vector3f ka(0.01, 0.01, 0.01);
    Eigen::Vector3f kd(p.c.r(), p.c.g(), p.c.b());
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    Eigen::Vector3f amb_light_intensity{ 15, 15, 15 };
    Eigen::Vector3f ambient = amb_light_intensity.cwiseProduct(kd / 45);


    Eigen::Vector3f light_pos(-10.0, 0.0, 0.0);
    Eigen::Vector3f light_intensity(600.0f, 600.0f, 600.0f);

    Eigen::Vector3f vert_pos(p.pos.x(), p.pos.y(), p.pos.z());
    Eigen::Vector3f normal(p.normal.x(), p.normal.y(), p.normal.z());
    Eigen::Vector3f light_dir = light_pos - vert_pos;
    float distance = light_dir.norm();
    distance = distance * distance;
    //light_dir.normalize();
    float lambertian = std::max(normal.dot(light_dir.normalized()), 0.0f);
    Eigen::Vector3f diffuse = kd.cwiseProduct((light_intensity / distance) * lambertian);


    Eigen::Vector3f view_dir = vert_pos * -1;
    view_dir.normalize();
    Eigen::Vector3f half_vec = (light_dir + view_dir) / ((light_dir + view_dir).norm());
    //half_vec.normalize();
    float _p = 150;
    float spec_lambertian = pow(std::max(normal.dot(half_vec), 0.0f), _p);
    Eigen::Vector3f specular = ks.cwiseProduct((light_intensity / distance) * spec_lambertian);


    Eigen::Vector3f result_color(0.0f, 0.0f, 0.0f);
    result_color += ambient + diffuse + specular;

    p.c.set(result_color[0], result_color[1], result_color[2]);
}
