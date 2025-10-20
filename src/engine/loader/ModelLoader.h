#pragma once
#include <fstream>
#include <string.h>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include "mesh/mesh.h"
#include <iostream>
#include <memory_resource>

class ModelLoader
{
private:

	// beginn variable
	std::unique_ptr<engine::mesh::Model> _modelTmpPtr; // data will be deserialize into this


	// tmp variable to store read model data for later
	// to construct the model
	engine::mesh::AABB _modelAABB { 
		{FLT_MAX,FLT_MAX,FLT_MAX},
		{ -FLT_MAX, -FLT_MAX, -FLT_MAX }};

	std::vector<engine::math::Vec3f> _vertexPositions;
	std::vector<engine::math::Vec3f> _vertexNormals;
	std::vector<unsigned int> _vertexIndicies;
	std::unordered_set<unsigned int> _uniqueIndex;
	std::vector<unsigned int> _normalsIndicies;
	std::unordered_map<unsigned int, unsigned int> _vertextNormalMap;
	std::unordered_map<engineID_t, engine::mesh::Primitive*> _idToPrimitiveMap;
	std::vector<engine::mesh::Vertex*> _verticies;


	std::pmr::memory_resource* _mr;

	// end variable

	// beginn function declaration
	void readFile(const char* modelPath);
	void deserializeVertexNormalsDataLine(std::ifstream& file);
	void deserializeVertexPosDataLine(std::ifstream& file);
	void deserializeElementIndexDataLine(std::ifstream& file);
	void splitString(const char* str, std::vector<std::string>* v, const char* delimiter = " ");
	void constructVertices();
	bool [[nodiscard]] normalizationCheck() const ;
	void constructPrimitives();
	void constructModel();
	void cubifyAABB();
	bool edgeManifoldCheck(const engineID_t& v0, const engineID_t& v1, const engineID_t& v2);
	// end functioin declaration
public:
	ModelLoader();
	~ModelLoader();
	engine::mesh::Model* load(const char* modelPath, std::pmr::memory_resource* mr);
	

};