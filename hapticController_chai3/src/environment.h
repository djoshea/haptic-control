#ifndef _ENVIRONMENT_H_INCLUDED_
#define _ENVIRONMENT_H_INCLUDED_

#include <vector>
#include "chai3d.h"
#include "cMoveToPoint.h"
#include "cBoundingPlane.h"
#include "cPlanarObstacle.h"

//-----------------------------------------------------------------------
// FUNCTION SIGNATURES
//-----------------------------------------------------------------------
void initScene();
void initHapticTool();
void initGLDisplay(int argc, char* argv[]);
void runSimulation();

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the mouse is moved with a button pressed
void mouseButtons(int, int, int, int);
void mouseDrag(int, int);

void updateCamera();

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

void endSimulation();

// main graphics callback
void updateGraphics(void);

// main haptics loop
void updateHaptics(void);

// labels
int addMessageLabel(void);
void updateMessage(int, std::string);
void updateMessage(int, const char*);
void setMessageColor(int, cColorf);

#define MAX_DEVICE_NAME_LENGTH 100

// for storing all chai related things shared among files / methods
typedef std::vector<cLabel*> cLabelVector;
struct ChaiData {
    char deviceName[MAX_DEVICE_NAME_LENGTH];

    // a haptic device handler
    cHapticDeviceHandler* handler;

    cGenericHapticDevicePtr hapticDevice;

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
    cPrecisionClock simClockNetwork;
    cPrecisionClock simClockStateMachine;
    cPrecisionClock simClockHapticStateUpdate;

    // info about current haptic's capabilities
    cHapticDeviceInfo deviceInfo;

    // width and height of the current window display
    int displayW;
    int displayH;

    // a virtual tool representing the haptic device in the scene
    cToolCursor* tool;

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
    cDirectionalLight *light;

};

#endif
