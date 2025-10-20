#pragma once
#include <engine/clustering/cluster.h>

class BufferDataGenarator
{
private:

	// begin var declarations
	engine::cluster::ClusterPools& _pool;
	// end var declarations


	// begin function declarations

	// end function declarations
public:
	BufferDataGenarator(engine::cluster::ClusterPools& pool);
	~BufferDataGenarator();
	
	void createBufferData(engine::mesh::LinearizedModelData* linearizedData);
};