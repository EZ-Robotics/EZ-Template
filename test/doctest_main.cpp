// Generates doctest's main() and enables its DOCTEST_CONFIG_IMPLEMENT block.
// That block is what pulls in <windows.h> on a Windows host build, so this
// file includes nothing else: no library header is ever compiled in the same
// translation unit as <windows.h>, which is what previously collided
// EZ-Units' `pascal` unit literal with minwindef.h's `#define pascal __stdcall`.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
