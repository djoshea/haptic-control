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
#include <signal.h>
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

void shutdown() {
    endSimulation();
    networkClose();
    stateMachineStop();
}

void finish_main(int sig)
{
    printf("Shutting down...\n");
    shutdown();
    exit(-1);
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
    signal(SIGINT, finish_main);
    atexit(shutdown);

    hapticInitialize();
    initScene();
    initHapticTool();
    initHapticWorkspace();
    initGLDisplay(argc, argv);

    runSimulation();

    // exit
    shutdown();
    return (0);
}


//---------------------------------------------------------------------------

