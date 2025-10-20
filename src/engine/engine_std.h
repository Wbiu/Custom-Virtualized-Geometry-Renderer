#pragma once
#include <engine/math/math.h>
#include <memory_resource>

typedef uint32_t engineID_t;

namespace engine::std_
{
	struct EngineResourcePool {
		std::pmr::unsynchronized_pool_resource pool
		{ 
			std::pmr::new_delete_resource() 
		};
	};

}
