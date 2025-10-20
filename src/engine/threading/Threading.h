#pragma once
#include <thread>
#include <functional>
class Threading
{
private:

	// vars
	std::function<void(const char*)> _toExecute;
	std::jthread _jthread;
	char _modelPath[256];
	// end vars

	// functions


	// end functions

public:

	Threading();
	~Threading();

	void load(std::function<void(const char*)>  toExecute, const char* path);
	void run();

};
