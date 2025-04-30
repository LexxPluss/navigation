#ifndef PORTABLE_UTILS_H
#define PORTABLE_UTILS_H

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) || defined(_WIN64)
// Some system (e.g., Windows) doesn't come with drand48(), srand48().
// Use rand, and srand for such system.
static double drand48(void)
{
    return ((double)rand())/RAND_MAX;
}

static void srand48(long int seedval)
{
    srand(seedval);
}
#endif  // _WIN32 || _WIN64

#ifdef __cplusplus
}
#endif

#endif