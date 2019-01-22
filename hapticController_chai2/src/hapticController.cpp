//===========================================================================
/*
    This file is part of the CHAI 3D visualization and haptics libraries.
    Copyright (C) 2003-2010 by CHAI 3D. All rights reserved.

    This library is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License("GPL") version 2
    as published by the Free Software Foundation.

    For using the CHAI 3D libraries with software that can not be combined
    with the GNU GPL, and for taking advantage of the additional benefits
    of our support services, please contact CHAI 3D about acquiring a
    Professional Edition License.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   2.1.0 $Rev: 322 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//---------------------------------------------------------------------------
#include "chai3d.h"
#include "dhdc.h"
#include "drdc.h"
#include "utils.h"
#include "environment.h"
#include "haptic.h"
#include "network.h"
#include "stateMachine.h"
//---------------------------------------------------------------------------

extern ChaiData chai; // declared in environment.cpp

// declared in utils.h
void waitForAllThreadsDown() {
    printf("waiting for threads down\n");
    while(hapticsIsUp() || networkIsUp() || stateMachineIsUp()) {
        cSleepMs(100);
    }
}

void shutdown() {
    chai.simulationFinished = true;
    waitForAllThreadsDown();
}

void ctrl_c(int sig)
{
    printf("\nHapticController: Received SIGINT, shutting down\n");
    chai.simulationFinished = true;
}

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------
    printf ("\n");
    char titleStr[] = "Haptic Control Environment";
    char linechars[50] = "";
    unsigned W = strlen(titleStr) + 2;
    for(unsigned i = 0; i < W; i++)
    {
        //linechars[i] = '\x71';
        linechars[i] = 0x71;
    }
    linechars[W] = '\0';
    //printf ("\e[34;01m\e(0\x6c%s\x6b\e(B\n", linechars);
    //printf ("\e(0\x78\e(B \e[37;01m%s \e[34;01m\e(0\x78\e(B\n", titleStr);
    //printf ("\e[34;01m\e(0\x6d%s\x6a\e(B\e[0m\n", linechars);

    printf("%s\n\n", titleStr);

    // Register <C-c> handler
    signal(SIGINT, ctrl_c);
    atexit(shutdown);

    chai.simulationFinished = false;

    // start haptic thread
    hapticsStart();

    // start the graphics GL thread
    graphicsInit(); // in environment.cpp

    // start the network send state thread
    networkStart();

    // start the network receive commands state machine
    stateMachineStart();

    // simulation in now running
    chai.simulationRunning = true;

    struct sched_param sp;
    memset(&sp, 0, sizeof(struct sched_param));
    sp.sched_priority = sched_get_priority_min(SCHED_IDLE);
    pthread_t thisThread = pthread_self();
    pthread_setschedparam(thisThread, SCHED_IDLE, &sp);

    graphicsLoop();
    //while(!chai.simulationFinished) { cSleepMs(1000);}

    waitForAllThreadsDown();

    // exit
    return (0);
}

//---------------------------------------------------------------------------

