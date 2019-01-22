#ifndef _NETWORK_H_
#define _NETWORK_H_

#include <stdlib.h>

#define MAX_PACKET_LENGTH 512
#define HAPTIC_PACKET_PREFIX "#h"

void diep(const char *s);

void networkStart();
void networkClose();
void networkUpdate();

bool networkIsUp();

int networkReceive(uint8_t * buffer, int bufferLength);

#endif
