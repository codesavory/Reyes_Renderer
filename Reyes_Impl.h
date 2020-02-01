#include <iostream>
#include <atlimage.h>
#include "Transformations_Impl.h"
#include "Mesh_Datastructure_Impl.h"
#include <Eigen/Eigen>
#include <cmath>

// ******* The following lines have to be added for stb_image_write to
// compile properly
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

const double PI = 3.141592653589793238462643383279502884197;
#define EPSILON 0.000001

float check_float_epsilon(float input_float)
{
    if (input_float < EPSILON)
        return 0.;
    return input_float;
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}




int ppimage(std::vector<Eigen::Vector4f> ndc_points) {
    const int width = 300;
    const int height = 300;
#define CHANNEL_NUM 3

    //grey background
    uint8_t* framebuffer = new uint8_t[width * height * CHANNEL_NUM];

    // Set background color
    // For the following framebuffer, (0,0) pixel is at the top left
    int index = 0;
    for (int i = 0; i < width; ++i)
    {
        for (int j = 0; j < height; ++j)
        {
            int r = 255;
            int g = 255;
            int b = 0;

            framebuffer[index++] = r;
            framebuffer[index++] = g;
            framebuffer[index++] = b;
        }
    }

    for (int i = 0; i < ndc_points.size(); ++i) {
        Eigen::Vector4f ndc_point = ndc_points[i];

        //Any point outside of NDC space chuck it.
        if (ndc_point.x() < -1 || ndc_point.x() > 1 || ndc_point.y() < -1 || ndc_point.y() > 1) continue;
        //Convert ndc coords to image coords
        //(-1,-1) ndc coordinates will be converted to (0,0) in image space
        int x = min(width - 1, (int)((ndc_point.x() + 1) * 0.5 * width));
        int y = min(height - 1, (int)((1 - (ndc_point.y() + 1) * 0.5) * height));

        // Since x and y are coordinates where origin is at the bottom left and the framebuffer
        // has the origin at top left, we have to transform the obtained pixel coordinates. 
        // This is essentially a 90 degree rotation which is the same as flipping the coordinates
        index = (y * width * 3) + x * 3;
        int r = 0;
        int g = 0;
        int b = 0;
        framebuffer[index++] = r;
        framebuffer[index++] = g;
        framebuffer[index++] = b;
    }

    stbi_write_jpg("D:\\CG_Source\\CS285\\HW-1\\Reyes_Architecture1\\stbjpg3.jpg", width, height, 3, framebuffer, 100);
    delete[] framebuffer;
    return 0;
}


void generate_sphere_mesh(double radius, Eigen::Vector3f center, mesh_datastructure& mesh)
{
    int parallels = 25;
    int meridians = 25;
    double r = radius;
    //mesh.vertices.emplace_back(0.0f, 1.0f, 0.0f);
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
            float const x = r * sp * ca + center.x();
            float const y = r * cp + center.y();
            float const z = r * sp * sa + center.z();
            Eigen::Vector3f p;
            p[0] = x;
            p[1] = y;
            p[2] = z;
            mesh.pushvert(p);
            //mesh.vertices.emplace_back(x, y, z);
        }
    }
}

void generate_torus_mesh(double majorRadius, double minorRadius, double minorSegments, double majorSegments, mesh_datastructure& mesh)
{
    // minor radius is the thickness of the torus / 2
    auto PI2 = PI * 2;
    auto arc = PI2;

    Eigen::Vector3f center = { 0, 0, 0 };

    for (double j = 0; j < minorSegments; ++j) {
        for (double i = 0; i < majorSegments; ++i) {
            double const u = i / majorSegments * arc;
            double const v = j / minorSegments * PI2;

            //center[0] = majorRadius * std::cos(u);
            //center[1] = majorRadius * std::sin(u);

            float const x = (majorRadius + minorRadius * std::cos(v)) * std::cos(u);
            float const y = (majorRadius + minorRadius * std::cos(v)) * std::sin(u);
            float const z = minorRadius * std::sin(v);

            Eigen::Vector3f vertex;
            vertex[0] = x;
            vertex[1] = y;
            vertex[2] = z;

            mesh.pushvert(vertex);
        }
    }
}

int render_orthographic_projection()
{
    //create the model
    mesh_datastructure point;//update the vertex coordinates
    mesh_datastructure triangle(3);
    mesh_datastructure sphere(0);

    //transform
    Eigen::Vector3f eye_pos = { 0, 0, -20 };
    float angle = 45;
    Eigen::Matrix4f mvp = get_ortho_projection_matrix(50, 50, 0.1, 50) * get_view_matrix(eye_pos) * get_model_matrix(angle, 'x');
    //eye_fov,aspect_ratio,zNear,zFar

    //point*MVP
    point.setVertex(0, { 0,0,0 });
    std::cout << "Point Coodinates:" << point.v[0] << std::endl;
    Eigen::Vector4f camera_coordinates_point = mvp * to_vec4(point.v[0], 1.0f);
    std::cout << "Camera Coodinates:" << camera_coordinates_point;

    //triangle*MVP
    triangle.setVertex(0, { 5,5,0 });
    triangle.setVertex(1, { 5,10,0 });
    triangle.setVertex(2, { 7.5,7.5,0 });
    std::cout << "Point Coodinates:" << endl;
    triangle.printVertex();
    Eigen::Vector4f camera_coordinates_triangle[3];
    for (int vertex = 0; vertex < triangle.v.size(); vertex++)
        camera_coordinates_triangle[vertex] = mvp * to_vec4(triangle.v[vertex], 1.0f);
    std::cout << "Camera Coodinates:" << camera_coordinates_triangle;



    //generate_sphere_mesh(2, { 0,0,0 }, sphere);

    generate_torus_mesh(30, 7, 64, 64, sphere);

    std::vector<Eigen::Vector4f> verts;
    for (int vertex = 0; vertex < sphere.v.size(); vertex++) {
        auto p = to_vec4(sphere.v[vertex], 1.0f);
        verts.push_back(p);
    }


    //Eigen::Vector4f ndc_points[200];
    for (int i = 0; i < verts.size(); ++i) {
        Eigen::Vector4f p = verts[i];

        //Do prespective transformation to get ndc coordinates
        Eigen::Vector4f camera_coordinates = mvp * p;
        verts[i] = camera_coordinates;
    }
    ppimage(verts);
    return 0;
}



int render_perspective_projection()
{
    //create the model
    mesh_datastructure point;//update the vertex coordinates
    mesh_datastructure triangle(3);
    mesh_datastructure sphere(0);

    //transform
    Eigen::Vector3f eye_pos = { 0, 0, -10 };
    float angle = 45;
    Eigen::Matrix4f mvp = get_perspective_projection_matrix(45, 1.0, 0.1, 50) * get_view_matrix(eye_pos) * get_model_matrix(angle, 'x');
    //eye_fov,aspect_ratio,zNear,zFar

//point*MVP
    point.setVertex(0, { 0,0,0 });
    std::cout << "Point Coodinates:" << point.v[0] << std::endl;
    Eigen::Vector4f camera_coordinates_point = mvp * to_vec4(point.v[0], 1.0f);
    std::cout << "Camera Coodinates:" << camera_coordinates_point;

    //triangle*MVP
    triangle.setVertex(0, { 5,5,0 });
    triangle.setVertex(1, { 5,10,0 });
    triangle.setVertex(2, { 7.5,7.5,0 });
    std::cout << "Point Coodinates:" << endl;
    triangle.printVertex();
    Eigen::Vector4f camera_coordinates_triangle[3];
    for (int vertex = 0; vertex < triangle.v.size(); vertex++)
        camera_coordinates_triangle[vertex] = mvp * to_vec4(triangle.v[vertex], 1.0f);
    std::cout << "Camera Coodinates:" << camera_coordinates_triangle;



    /*
    //sphere
    Eigen::Vector3f center(0, 0, 0);
    double r = 5;
    int index_of_points = 0;
    for (double phi = 0.; phi < 2 * PI; phi += PI / 10.) // Azimuth [0, 2PI]
    {
        for (double theta = 0.; theta < PI; theta += PI / 10.) // Elevation [0, PI]
        {
            Eigen::Vector3f point;
            point[0] = check_float_epsilon(r * cos(phi) * sin(theta) + center.x());
            point[1] = check_float_epsilon(r * sin(phi) * sin(theta) + center.y());
            point[2] = check_float_epsilon(r * cos(theta) + center.z());
            //spherePoints.push_back(point);
            cout << "Point" << index_of_points << ":" << point << endl;
            sphere.pushvert(point);
            //sphere.setVertex(index_of_points++, point);//calculates a vertex for each phase discretized
        }
    }
    */

    generate_sphere_mesh(2, { 0,0,0 }, sphere);
    //generate_torus_mesh(2, 1, 32, 32, sphere);


    std::vector<Eigen::Vector4f> v;
    std::cout << "Point Coodinates:" << endl;
    sphere.printVertex();
    //Eigen::Vector4f camera_coordinates_sphere[200];
    for (int vertex = 0; vertex < sphere.v.size(); vertex++) {
        auto p = to_vec4(sphere.v[vertex], 1.0f);
        v.push_back(p);
    }
    //std::cout << "Camera Coodinates:" << camera_coordinates_sphere;

    std::vector<Eigen::Vector4f> ndc_points;
    //Eigen::Vector4f ndc_points[200];
    for (int i = 0; i < v.size(); ++i) {
        Eigen::Vector4f p = v[i];

        //Do prespective transformation to get clip space
        Eigen::Vector4f camera_coordinates = mvp * p;
        //Divide x,y and z components with w component to get ndc coords. 
        camera_coordinates /= camera_coordinates.w();
        //std::cout << "NDC Coodinates:\n" << camera_coordinates << std::endl;
        ndc_points.push_back(camera_coordinates);
        //std::cout << "-----------------------------------\n\n\n";
    }
    ppimage(ndc_points);
    return 0;
};