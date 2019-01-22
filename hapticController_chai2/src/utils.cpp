#include <stdio.h>
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include "utils.h"

void diep(const char *s)
{
    perror(s);
	exit(1);
}

void sleepMs(double a_interval)
{
#if defined(_WIN32) | defined (_WIN64)
    Sleep(a_interval);
#endif

    struct timespec t;
    t.tv_sec  = a_interval/1000000.0;
    t.tv_nsec = fmod(a_interval, 1000000.0);
    nanosleep (&t, NULL);

#if defined(_MACOSX)
    struct timespec t;
    t.tv_sec  = a_interval/1000000.0;
    t.tv_nsec = fmod(a_interval, 1000000);
    nanosleep (&t, NULL);
#endif
}
