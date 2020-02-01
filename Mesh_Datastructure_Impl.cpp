#include <iostream>
#include "Mesh_Datastructure_Impl.h"

mesh_datastructure::mesh_datastructure()
{
    //initialize a single point to zero
    v.push_back({ 0,0,0 });
}

mesh_datastructure::mesh_datastructure(int n)
{
    //resize the vector class and initialize to zeros
    v.resize(n, { 0,0,0 });
}

void mesh_datastructure::setVertex(int ind, Eigen::Vector3f ver)
{
    v[ind] = ver;
}

void mesh_datastructure::pushvert(Eigen::Vector3f ver)
{
    v.push_back(ver);
}
void mesh_datastructure::printVertex()
{
    for (int vertex_index = 0; vertex_index < 1; vertex_index++)
    {
        std::cout << v[vertex_index];
    }
}