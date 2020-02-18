#pragma once
#include<Eigen/Eigen>
#include "Types.h"
#include <math.h>
//extern Eigen::Vector4f __sh_P;
//extern Color __sh_C;
//extern float __sh_u;
//extern float __sh_v;

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

void earth(FragmentShaderPayload& p) {
    float u = p.u;
    float v = p.v;
    std::shared_ptr<Texture> t= p.texture;

    Color c = t->get_at(u, v);
    p.c = c;
}

void light(FragmentShaderPayload& p) {

    Eigen::Vector3f ka(0.01, 0.01, 0.01);
    Eigen::Vector3f kd(p.c.r(), p.c.g(), p.c.b());
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f ambient = amb_light_intensity.cwiseProduct(ka);


    Eigen::Vector3f light_pos(0.0, 10.0, 0.0);
    light_pos.x() *= -1;
    light_pos.y() *= -1;
    Eigen::Vector3f light_intensity(500.0f, 500.0f, 500.0f);

    Eigen::Vector3f vert_pos(p.pos.x(), p.pos.y(), p.pos.z());
    Eigen::Vector3f normal(p.normal.x(), p.normal.y(), p.normal.z());
    Eigen::Vector3f light_dir = light_pos - vert_pos;
    float distance = light_dir.norm();
    distance = distance * distance;
    light_dir.normalize();
    float lambertian = std::max(normal.dot(light_dir), 0.0f);
    Eigen::Vector3f diffuse = kd.cwiseProduct( (light_intensity / distance) * lambertian);


    Eigen::Vector3f view_dir = vert_pos * -1;
    view_dir.normalize();
    Eigen::Vector3f half_vec = (light_dir + view_dir) / ((light_dir + view_dir).norm());
    half_vec.normalize();
    float _p = 150;
    float spec_lambertian = pow(std::max(normal.dot(half_vec), 0.0f), _p);
    Eigen::Vector3f specular = ks.cwiseProduct((light_intensity / distance) * spec_lambertian);


    Eigen::Vector3f result_color(0.0f, 0.0f, 0.0f);
    result_color += ambient + diffuse + specular;

    p.c.set(result_color[0], result_color[1], result_color[2]);
}
