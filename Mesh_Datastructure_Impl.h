#pragma once
#include <Eigen/Eigen>
#include <vector>

class mesh_datastructure
{
public:
	std::vector<Eigen::Vector3f> v;//malloc or vector for dynamic

	mesh_datastructure();//initialize to zero

	mesh_datastructure(int n);//resize and initialize variables

	void setVertex(int ind, Eigen::Vector3f ver);
	void printVertex();
	void pushvert(Eigen::Vector3f ver);

	/*each points values*/
	//Eigen::Vector3f color[3];
	//Eigen::Vector2f tex_coordinates[3];
	//Eigen::Vector3f normal[3];
};