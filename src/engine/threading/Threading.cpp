#include "Threading.h"

Threading::Threading()
{

}

Threading::~Threading()
{

}

void Threading::load(std::function<void(const char*)>  toExecute, const char* path)
{
	_toExecute = std::move(toExecute);
	strcpy_s(_modelPath,path);
}

void Threading::run()
{
	if (_jthread.joinable()) return; // already running;
	_jthread = std::jthread([this,fn = std::move(_toExecute)]()
		{
			fn(_modelPath);
		}); // returns immediately; worker runs in background
}