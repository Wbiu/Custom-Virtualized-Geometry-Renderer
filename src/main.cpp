#include <core/App.h>
#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")
#define DISABLE_CONSOLE // Comment out this line for console logging and printing!
#ifdef DISABLE_CONSOLE
#pragma comment(linker, "/SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
#endif

int main()
{
	timeBeginPeriod(1);
	App* app = new App(1400, 1050);
	app->startup();
	delete(app);
	timeEndPeriod(1);
	return 0;
}


