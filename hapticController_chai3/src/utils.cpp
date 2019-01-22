#include <stdio.h>
#include <stdlib.h>
#include "utils.h"

void diep(const char *s)
{
    perror(s);
	exit(1);
}
