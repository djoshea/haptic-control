#ifndef _ENVIRONMENT_H_INCLUDED_
#define _ENVIRONMENT_H_INCLUDED_

#include <vector>
#include "chai3d.h"
#include "scenegraph/CShapeLine.h"
#include "cMoveToPoint.h"
#include "cBoundingPlane.h"
#include "cPlanarObstacle.h"

//-----------------------------------------------------------------------
// FUNCTION SIGNATURES
//-----------------------------------------------------------------------
//void initScene();
//void initGLDisplay(int argc, char* argv[]);

bool graphicsIsUp();
void graphicsInit();
void graphicsLoop();
bool waitForGraphicsReady(); // returns true if graphics up, false if simulationFinished, otherwise hangs

// main graphics callback
void updateGraphics(void);

// labels
unsigned int addMessageLabel(void);
void updateMessage(unsigned int, string);
void updateMessage(unsigned int, const char*);
void setMessageColor(unsigned int, cColorf);

#define MAX_DEVICE_NAME_LENGTH 100

// for storing all chai related things shared among files / methods
typedef std::vector<cLabel*> cLabelVector;
struct ChaiData {
    char deviceName[MAX_DEVICE_NAME_LENGTH];

    // a haptic device handler
    cHapticDeviceHandler* handler;

    cGenericHapticDevice* hapticDevice;

    // haptic workspace and device info
    double workspaceScaleFactor;
    double scaledStiffnessMax;
    double scaledLinearDampingMax;
    double scaledForceMax;

    // status of the main simulation haptics loop
    bool simulationRunning;
    bool simulationFinished;

    // simulation clock
    cPrecisionClock simClock;
    cPrecisionClock simClockStateMachine;
    cPrecisionClock simClockNetwork;
    cPrecisionClock simClockHapticMonotonic;

    // info about current haptic's capabilities
    cHapticDeviceInfo deviceInfo;

    // width and height of the current window display
    int displayW;
    int displayH;

    // a virtual tool representing the haptic device in the scene
    cGeneric3dofPointer* tool;

    // global maximum force applied by haptic in the XY plane
    double maxHapticForceXY;

    // size of force proxy sphere
    double proxyRadius;

    // container for all text labels in scene
    cGenericObject* rootLabels;
    cLabelVector debugLabels;

    // a world that contains all objects of the virtual environment
    cWorld* world;

    // a camera that renders the world in a window display
    cCamera* camera;
    double cameraDistance;
    double cameraRotX;
    double cameraRotY;

    bool mouseClicked;
    int mouseClickOriginX;
    int mouseClickOriginY;

    // a light source to illuminate the objects in the virtual scene
    cLight *light;

};

#endif
