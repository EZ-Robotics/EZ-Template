#ifndef PATHFINDER_LIB_H_DEF
#define PATHFINDER_LIB_H_DEF

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    #define CAPI __declspec(dllexport)
#else
    #define CAPI
#endif

#endif