#include "ModelLoader.h"


ModelLoader::ModelLoader()
{
	
}

ModelLoader::~ModelLoader()
{
}

engine::mesh::Model* ModelLoader::load(const char* modelPath, std::pmr::memory_resource* mr)
{	
	_mr = mr;
	// creat a new model tobe returned
	_modelTmpPtr = std::make_unique<engine::mesh::Model>(mr);

	readFile(modelPath);
	constructVertices();
	constructPrimitives();
	cubifyAABB();
	constructModel();
	
	// returning the new model
	return _modelTmpPtr.release();
}

void ModelLoader::readFile(const char* modelPath) 
{
	std::ifstream file(modelPath);
	char tmpChar_current = 0, tmpChar_next = 0;

	if (file.is_open())
	{
		while (file)
		{
			tmpChar_current = file.get();

			if (file.peek() != EOF)
				tmpChar_next = file.get();

			if (tmpChar_next == ' ' || (tmpChar_current != 0 && tmpChar_next != 0))
			{
				/* validate data line identifier and
				retriving data on that line till end of line */
				switch (tmpChar_current)
				{
				case 'v':
					switch (tmpChar_next)
					{
					case 'n':
						deserializeVertexNormalsDataLine(file);
						break;
					case ' ':
						deserializeVertexPosDataLine(file);
						break;
					}
					break;

				case 'f':
					deserializeElementIndexDataLine(file);
					break;
				default:
					file.ignore(1000, '\n'); // ignor unwanted line
					continue;
				}
			}
			else
			{
				tmpChar_current = tmpChar_next;
				tmpChar_next = 0;
			}
		}
	}
	else
	{
		char buf[1028];
		strcpy_s(buf,"ERROR::MODELLOADER::IO::FILE::FILE_NOT_FOUND!::");
		strcat_s(buf, modelPath);
		strcat_s(buf, "\n");
		throw std::runtime_error(buf);
	}
}

void ModelLoader::deserializeVertexNormalsDataLine(std::ifstream& file)
{
	std::string tmpStr;
	char tmpChar;
	while (file)
	{
		tmpChar = file.get();
		/* read line char by char till new line */
		if (tmpChar != '\n')
		{
			tmpStr += tmpChar;
		}

		/* if reaches the end of line the parse line */
		if (tmpChar == '\n')
		{
			std::vector<std::string> v;
			splitString((char*)tmpStr.c_str(), &v);
			engine::math::Vec3f v3 = engine::math::Vec3f{ (float)atof(v[0].c_str()) , (float)atof(v[1].c_str()) , (float)atof(v[2].c_str()) };
			_vertexNormals.push_back(v3);

			break;
		}

	}

}

void ModelLoader::deserializeVertexPosDataLine(std::ifstream& file)
{
	std::string tmpStr;
	char tmpChar;
	while (file)
	{
		tmpChar = file.get();
		/* read line char by char till new line */
		if (tmpChar != '\n')
		{
			tmpStr += tmpChar;
		}

		/* if reaches the end of line the parse line */
		if (tmpChar == '\n')
		{
			std::vector<std::string> v;
			splitString((char*)tmpStr.c_str(), &v);

			engine::math::Vec3f v3 = engine::math::Vec3f{ (float)atof(v[0].c_str()) , (float)atof(v[1].c_str()) , (float)atof(v[2].c_str()) };

			// get model dimention for bounding box
			// X axis
			// positive X
			if (v3.x > _modelAABB.max.x)
			{
				_modelAABB.max.x = v3.x;
			}
			// negative X
			if (v3.x < _modelAABB.min.x)
			{
				_modelAABB.min.x = v3.x;
			}
			// end X axis

			//Y axis
			// positive Y
			if (v3.y > _modelAABB.max.y)
			{
				_modelAABB.max.y = v3.y;
			}
			// negative Y
			if (v3.y < _modelAABB.min.y)
			{
				_modelAABB.min.y = v3.y;
			}
			// end Y axis


			// Z axis
			// postive Z
			if (v3.z > _modelAABB.max.z)
			{
				_modelAABB.max.z = v3.z;
			}
			// negative Z
			if (v3.z < _modelAABB.min.z)
			{
				_modelAABB.min.z = v3.z;
			}
			// end Z axis

			_vertexPositions.push_back(v3);
			break;
		}

	}
}

void ModelLoader::deserializeElementIndexDataLine(std::ifstream& file)
{
	std::string tmpStr;
	char tmpChar;
	while (file)
	{
		tmpChar = file.get();
		/* read line char by char till new line */
		if (tmpChar != '\n')
		{
			tmpStr += tmpChar;
		}

		/* if reaches the end of line the parse line */
		if (tmpChar == '\n')
		{
			std::vector<std::string> v;
			splitString(tmpStr.c_str(), &v);
			
			for (std::string str : v)
			{
				std::vector<std::string> v1;
				splitString(str.c_str(), &v1, "/");
				/* -1 index due .obj starts counting at index 1 */
				/* index 0 is the position */
				/* index 1 is the nomarl */
				int val = atoi(v1[0].c_str()) - 1;
				_vertexIndicies.push_back(val);
 				_uniqueIndex.insert(val);
				//_mapOfIdxNighbors[]

				if (v1.size() > 1)
				{
					//_normalsIndicies.push_back(atoi(v1[2].c_str()) - 1);
					//f v1 / vt1 / vn1
					//	^    ^     ^
					//  |	 |	   | 
					// face  |	   |
					//       |	   | 
					//	   texture |
					//			 normals
					_normalsIndicies.push_back(atoi((*--v1.end()).c_str()) - 1);
					_vertextNormalMap[atoi((*v1.begin()).c_str()) - 1] = atoi((*--v1.end()).c_str()) - 1;
				}
			}
			break;
		}
	}
}

void ModelLoader::splitString(const char* str, std::vector<std::string>* v, const char* delimiter)
{
	char* ct = nullptr;
	char* token = strtok_s((char*)str, delimiter, &ct);
	v->push_back(token);

	while (token != nullptr) {
		token = strtok_s(nullptr, delimiter, &ct);
		if (token != nullptr)
			v->push_back(token);
	}
}

void ModelLoader::constructVertices()
{
	// starting primitiveID 
	engineID_t vertexID = 1;

	// there could be an issue here when normals are included 
	// because no normalization check is performed here!
	if (_vertextNormalMap.size() != 0)
	{
		for (auto& pair : _vertextNormalMap)
		{
			engine::math::Vec4f vecPos = { _vertexPositions[pair.first]};
			engine::math::Vec4f vecNormals = { _vertexNormals[pair.second]};
			engine::mesh::Vertex* v = new engine::mesh::Vertex;
			v->id = vertexID++;
			v->coords = vecPos;
			v->normal = vecNormals;

			_modelTmpPtr->vertexPool[v->id] = v;
			_modelTmpPtr->vertexToPrimMap[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL][v->id];
			
			_verticies.push_back(v);
			_modelTmpPtr->vertexIndcicesBufferOrder.emplace_back(v->id);
		}
	}
	else
	{	
		// check if the coords are noramlized
		if (normalizationCheck())
		{
			// center of box
			engine::math::Vec3f center = {
				((_modelAABB.max.x + _modelAABB.min.x) / 2), //x
				((_modelAABB.max.y + _modelAABB.min.y) / 2), //y
				((_modelAABB.max.z + _modelAABB.min.z) / 2)  //z
			};

			float s_max = std::max({
				(_modelAABB.max.x - _modelAABB.min.x),	//x
				(_modelAABB.max.y - _modelAABB.min.y),	//y
				(_modelAABB.max.z - _modelAABB.min.z) }	//z
				);

			float scalingFactor = 2 / s_max;

			// scaling each vertex positions down / normalized to -1 <--> 1
			_verticies.reserve(_vertexPositions.size());

			for (engine::math::Vec3f& vertextPos : _vertexPositions)
			{
				engine::mesh::Vertex* v = new engine::mesh::Vertex;
				v->id = vertexID++;
				v->coords = {
					(vertextPos.x - center.x) * scalingFactor,
					(vertextPos.y - center.y) * scalingFactor,
					(vertextPos.z - center.z) * scalingFactor,
					1.0f
				};

				_verticies.emplace_back(v);

				(void)_modelTmpPtr->vertexPool.emplace(v->id, v);

				auto [outerIt, outerIns] = _modelTmpPtr->vertexToPrimMap.emplace(
					engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL,
					std::pmr::unordered_map<engineID_t, std::pmr::unordered_set<engineID_t>>{ _mr }
				);
				auto& innerMap = outerIt->second;

				auto [innerIt, innerIns] = innerMap.emplace(
					v->id,
					std::pmr::unordered_set<engineID_t>{ _mr }
				);
			}


			/*
				also scaling down the max dimention box to fit the model
			*/
			_modelAABB.max.x = ((_modelAABB.max.x - center.x) * scalingFactor);
			_modelAABB.max.y = ((_modelAABB.max.y - center.y) * scalingFactor);
			_modelAABB.max.z = ((_modelAABB.max.z - center.z) * scalingFactor);

			_modelAABB.min.x = ((_modelAABB.min.x - center.x) * scalingFactor);
			_modelAABB.min.y = ((_modelAABB.min.y - center.y) * scalingFactor);
			_modelAABB.min.z = ((_modelAABB.min.z - center.z) * scalingFactor);
		}
		else
		{
			// else already normalized
			_verticies.reserve(_vertexPositions.size());
			for (engine::math::Vec3f& vertextPos : _vertexPositions)
			{
				engine::mesh::Vertex* v = new engine::mesh::Vertex;
				v->id = vertexID++;
				v->coords = engine::math::Vec4f(vertextPos,1.0f);


				_modelTmpPtr->vertexPool[v->id] = v;
				(void)_modelTmpPtr->vertexPool.emplace(v->id, v);

				auto [outerIt, outerIns] = _modelTmpPtr->vertexToPrimMap.try_emplace(
					engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL,
					std::pmr::unordered_map<engineID_t, std::pmr::unordered_set<engineID_t>>{ _mr }
				);
				auto& innerMap = outerIt->second;

				auto [innerIt, innerIns] = innerMap.try_emplace(
					v->id,
					std::pmr::unordered_set<engineID_t>{ _mr }
				);

				_verticies.emplace_back(v);

			}
		}
	}

	_modelTmpPtr->nextAvailableVertexID = vertexID;
}

bool [[nodiscard]] ModelLoader::normalizationCheck() const
{
	float dX = _modelAABB.max.x - _modelAABB.min.x;
	float dY = _modelAABB.max.y - _modelAABB.min.y;
	float dZ = _modelAABB.max.z - _modelAABB.min.z;

	bool bX = dX >= 2.0f || dX <= -2.0f ? true : false;
	bool bY = dY >= 2.0f || dY <= -2.0f ? true : false;
	bool bZ = dZ >= 2.0f || dZ <= -2.0f ? true : false;

	return bX || bY || bZ ? true : false;

}

void ModelLoader::constructPrimitives()
{
	// starting primitiveID 
	engineID_t primitiveID = 1;
	engineID_t edgeID = 1;

	// get 3 indecices each loop 
	for (int i = 0; i < _vertexIndicies.size(); i = i + 3)
	{

		bool isManifold = edgeManifoldCheck(_verticies[_vertexIndicies[i]]->id, 
			_verticies[_vertexIndicies[i + 1]]->id,
			_verticies[_vertexIndicies[i + 2]]->id);

		if (isManifold)
		{
			std::runtime_error("ERROR::MESH_LOADER::INVALID_MESH_TOPOLOGY::EDGE_MANIFOLD::FIX_MESH!\n");
		}

		engine::mesh::Primitive* primitive = new engine::mesh::Primitive{};
		primitive->vertices[0] = { _verticies[_vertexIndicies[i]]->id };
		primitive->vertices[1] = { _verticies[_vertexIndicies[i + 1]]->id };
		primitive->vertices[2] = { _verticies[_vertexIndicies[i + 2]]->id };
		primitive->id = primitiveID++;


		engine::mesh::PrimKey primKey = engine::mesh::PrimKey{primitive->vertices[0], primitive->vertices[1], primitive->vertices[2] };

		_modelTmpPtr->primKeyToPrimMap[primKey] = primitive->id;
		_modelTmpPtr->primitivePool[primitive->id] = primitive;
		_modelTmpPtr->primitiveMap[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].insert(primitive->id);

		_modelTmpPtr->vertexToPrimMap[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].at(primitive->vertices[0]).insert(primitive->id);
		_modelTmpPtr->vertexToPrimMap[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].at(primitive->vertices[1]).insert(primitive->id);
		_modelTmpPtr->vertexToPrimMap[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].at(primitive->vertices[2]).insert(primitive->id);

		auto v0 = _modelTmpPtr->vertexPool.at(primitive->vertices[0]);
		auto v1 = _modelTmpPtr->vertexPool.at(primitive->vertices[1]);
		auto v2 = _modelTmpPtr->vertexPool.at(primitive->vertices[2]);



		engine::mesh::EdgeKey edgekey_1 = engine::mesh::EdgeKey{ v0,v1 };
		engine::mesh::EdgeKey edgekey_2 = engine::mesh::EdgeKey{ v1,v2 };
		engine::mesh::EdgeKey edgekey_3 = engine::mesh::EdgeKey{ v2,v0 };

		

		// temporary saving edgeID for later use
		engineID_t edgeID_1 = 0, edgeID_2 = 0, edgeID_3 = 0;

		// check if the created edges already exist
		if (_modelTmpPtr->edgeKeyToEdgeMap.find(edgekey_1) == _modelTmpPtr->edgeKeyToEdgeMap.end())
		{
			// true when edge does not exist. So create a new edge
			engine::mesh::Edge* newEdge = new engine::mesh::Edge{ v0,v1 };
			newEdge->id = edgeID++;
			edgeID_1 = newEdge->id;
			_modelTmpPtr->edgeKeyToEdgeMap[edgekey_1] = newEdge->id;
			_modelTmpPtr->edgePool.emplace(newEdge->id, newEdge);

			_modelTmpPtr->edgeMap[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].insert(newEdge->id);
			newEdge->prims[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].add(primitive->id);
		}
		else
		{
			edgeID_1 = _modelTmpPtr->edgeKeyToEdgeMap.at(edgekey_1);
			auto e_ptr = _modelTmpPtr->edgePool.at(edgeID_1);
			e_ptr->prims[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].add(primitive->id);
		}

		primitive->edges[0] = edgeID_1;


		if (_modelTmpPtr->edgeKeyToEdgeMap.find(edgekey_2) == _modelTmpPtr->edgeKeyToEdgeMap.end())
		{
			// true when edge does not exist
			engine::mesh::Edge* newEdge = new engine::mesh::Edge{ v1,v2 };
			newEdge->id = edgeID++;
			edgeID_2 = newEdge->id;
			_modelTmpPtr->edgeKeyToEdgeMap[edgekey_2] = newEdge->id;
			_modelTmpPtr->edgePool.emplace(newEdge->id, newEdge);

			_modelTmpPtr->edgeMap[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].insert(newEdge->id);

			newEdge->prims[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].add(primitive->id);
		}
		else
		{
			edgeID_2 = _modelTmpPtr->edgeKeyToEdgeMap.at(edgekey_2);
			auto e_ptr = _modelTmpPtr->edgePool.at(edgeID_2);
			e_ptr->prims[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].add(primitive->id);
		}

		primitive->edges[1] = edgeID_2;


		if (_modelTmpPtr->edgeKeyToEdgeMap.find(edgekey_3) == _modelTmpPtr->edgeKeyToEdgeMap.end())
		{
			// true when edge does not exist
			engine::mesh::Edge* newEdge = new engine::mesh::Edge{ v2,v0 };
			newEdge->id = edgeID++;
			edgeID_3 = newEdge->id;
			_modelTmpPtr->edgeKeyToEdgeMap[edgekey_3] = newEdge->id;
			_modelTmpPtr->edgePool.emplace(newEdge->id, newEdge);

			_modelTmpPtr->edgeMap[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].insert(newEdge->id);
			newEdge->prims[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].add(primitive->id);
		}
		else
		{
			edgeID_3 = _modelTmpPtr->edgeKeyToEdgeMap.at(edgekey_3);
			auto e_ptr = _modelTmpPtr->edgePool.at(edgeID_3);
			e_ptr->prims[engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL].add(primitive->id);
		}

		primitive->edges[2] = edgeID_3;

		///  centroid
		primitive->centroid = {
			((v0->coords.x + v1->coords.x + v2->coords.x) / 3),
			((v0->coords.y + v1->coords.y + v2->coords.y) / 3),
			((v0->coords.z + v1->coords.z + v2->coords.z) / 3)
		};
	}

	_modelTmpPtr->nextAvailablePrimitiveID = primitiveID;
	_modelTmpPtr->nextAvailableEdgeID = edgeID;
}

void ModelLoader::constructModel()
{
	_modelTmpPtr->aabb = std::move(_modelAABB);
}

void ModelLoader::cubifyAABB()
{
	engine::math::Vec3f min = _modelAABB.min;
	engine::math::Vec3f max = _modelAABB.max;

	float deltaX = std::abs(max.x - min.x);
	float deltaY = std::abs(max.y - min.y);
	float deltaZ = std::abs(max.z - min.z);

	float maxDim = std::max({ deltaX, deltaY, deltaZ });
	maxDim *= 1.1f; // scale buffer

	// Find the original center of the model AABB
	engine::math::Vec3f originalCenter = (min + max) * 0.5f;

	// Build new cube AABB around original center
	float halfSize = maxDim * 0.5f;
	engine::math::Vec3f cubeMin = originalCenter - engine::math::Vec3f(halfSize, halfSize, halfSize);
	engine::math::Vec3f cubeMax = originalCenter + engine::math::Vec3f(halfSize, halfSize, halfSize);

	// Update the AABB to the cube version
	_modelAABB.min = std::move(cubeMin);
	_modelAABB.max = std::move(cubeMax);
}

bool ModelLoader::edgeManifoldCheck(const engineID_t& v0, const engineID_t& v1, const engineID_t& v2)
{
	engine::mesh::EdgeKey edgekey_1 = engine::mesh::EdgeKey{ v0,v1 };
	engine::mesh::EdgeKey edgekey_2 = engine::mesh::EdgeKey{ v1,v2 };
	engine::mesh::EdgeKey edgekey_3 = engine::mesh::EdgeKey{ v2,v0 };

	if (_modelTmpPtr->edgeKeyToEdgeMap.find(edgekey_1) != _modelTmpPtr->edgeKeyToEdgeMap.end())
	{
		const engineID_t e = _modelTmpPtr->edgeKeyToEdgeMap.at(edgekey_1);
		auto e_ptr = _modelTmpPtr->edgePool.at(e);
		if(e_ptr->prims.at(engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL).size() >= 2 )
			return true;

	}


	if (_modelTmpPtr->edgeKeyToEdgeMap.find(edgekey_2) != _modelTmpPtr->edgeKeyToEdgeMap.end())
	{
		const engineID_t e = _modelTmpPtr->edgeKeyToEdgeMap.at(edgekey_2);
		auto e_ptr = _modelTmpPtr->edgePool.at(e);
		if (e_ptr->prims.at(engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL).size() >= 2)
			return true;

	}


	if (_modelTmpPtr->edgeKeyToEdgeMap.find(edgekey_3) != _modelTmpPtr->edgeKeyToEdgeMap.end())
	{
		const engineID_t e = _modelTmpPtr->edgeKeyToEdgeMap.at(edgekey_3);
		auto e_ptr = _modelTmpPtr->edgePool.at(e);
		if (e_ptr->prims.at(engine::cluster::HierarchyLevel::MODEL_LOADING_LEVEL).size() >= 2)
			return true;
	}

	return false;
}



