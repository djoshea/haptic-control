#include <vector>
#include <cstdlib>
#include <algorithm>
#include <sched.h>

#include "chai3d.h"
#include "scenegraph/CShapeLine.h"
#include "graphics/CColor.h"
#include "dhdc.h"
#include "drdc.h"
#include "environment.h"
#include "utils.h"
#include "haptic.h"
#include "network.h"
#include "stateMachine.h"
#include "string.h"
#include "cBoundingPlane.h"
#include "cPlanarObstacle.h"
#include "cMoveToPoint.h"
#include "hapticController.h"

#define GLUT_WHEEL_UP 3
#define GLUT_WHEEL_DOWN 4

//---------------------------------------------------------------------------
// DECLARED Variables
//---------------------------------------------------------------------------
ChaiData chai;
extern HapticState haptic;
extern HapticWorkspace workspace;

cPrecisionClock simClockGraphics;
double cameraTheta;
//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W         = 600;
const int WINDOW_SIZE_H         = 600;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;
const int OPTION_RETRACT = 3;
const int OPTION_RESUME = 4;
const int OPTION_TEST_PLANE = 5;

int MESSAGE_HEADER = -1;
int MESSAGE_BOUNDINGBOX = -1;
int MESSAGE_TOUCHINGSCREEN = -1;
int MESSAGE_HAPTICPOSITION = -1;
int MESSAGE_HAPTICVELOCITY = -1;
int MESSAGE_HITOBSTACLE = -1;
int MESSAGE_WORKSPACEEDGE= -1;
int MESSAGE_DEBUG = -1;

cThread* graphicsThread;
bool graphicsUp = false;
bool graphicsReady = false;

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a keyboard key is pressed
void keySelect(unsigned char key, int x, int y);

// callback when the mouse is moved with a button pressed
void mouseButtons(int, int, int, int);
void mouseDrag(int, int);

void updateCamera(double);

void initGLDisplay(int argc, char* argv[]);
void initScene();

// callback when the right mouse button is pressed to select a menu item
void menuSelect(int value);

bool graphicsIsUp() {
    return graphicsUp;
}

void graphicsInit() {
    printf("Graphics: starting\n");
    graphicsUp = true;
    char *myargv [1];
    int myargc=1;
    myargv[0]= strdup("hapticControllerPlane");
    initGLDisplay(myargc, myargv);
    initScene();
    graphicsReady = true;
}

void graphicsLoop() {
    printf("Graphics: entering glutMainLoop\n");
    glutMainLoop(); // start the graphics rendering, will call updateGraphics periodically

    printf("Graphics: stopping\n");
    graphicsUp = false;
}

void redrawTimerFn(int id) {
    glutPostRedisplay();
}

bool waitForGraphicsReady() {
    while(true) {
        if(chai.simulationFinished)
            return false;
        else if(graphicsReady && graphicsReady)
            return true;
        else
            cSleepMs(10);
    }
}

//---------------------------------------------------------------------------
void initScene() {
    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a new world.
    chai.world = new cWorld();

    // set the background color of the environment
    // the color is defined by its (R,G,B) components.
    chai.world->setBackgroundColor(0.1, 0.1, 0.1);

    // create a camera and insert it into the virtual world
    chai.camera = new cCamera(chai.world);
    chai.world->addChild(chai.camera);


    // position and orient the camera
    chai.cameraRotX = 0;
    chai.cameraRotY = 0;
    chai.cameraDistance = 400;
    //updateCamera();

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    chai.camera->setClippingPlanes(-100.00, 100.0);

    // create a light source and attach it to the camera
    chai.light = new cLight(chai.world);
    chai.camera->addChild(chai.light);                   // attach light to camera
    chai.light->setEnabled(true);                   // enable light source
    chai.light->setDirectionalLight(true);
    chai.light->setPos(cVector3d( 0, 0, 500));  // position the light source
    chai.light->setDir(cVector3d(0, 0, -1.0));  // define the direction of the light beam

    /////////////////////////////////////////////////////////////////////////
    // DESCRIPTIVE LABELS
    /////////////////////////////////////////////////////////////////////////

    cGenericObject rootLabels;

    chai.rootLabels = new cGenericObject();
    chai.camera->m_front_2Dscene.addChild(chai.rootLabels);
    // use addMessageLabel to add and updateMessage to update

    MESSAGE_HEADER = addMessageLabel();
    char strControl[200];
    snprintf(strControl, 200, "Haptic Control: %s", chai.deviceName);
    strControl[199] = '\0';
    updateMessage(MESSAGE_HEADER, strControl);
    setMessageColor(MESSAGE_HEADER, cColorf(1.0, 1.0, 1.0));
    MESSAGE_HAPTICPOSITION = addMessageLabel();
    MESSAGE_HAPTICVELOCITY = addMessageLabel();
    MESSAGE_HITOBSTACLE = addMessageLabel();
    setMessageColor(MESSAGE_HITOBSTACLE, cColorf(1.0, .3, 0.3));

    MESSAGE_WORKSPACEEDGE= addMessageLabel();
    setMessageColor(MESSAGE_WORKSPACEEDGE, cColorf(1.0, .3, 0.3));

    MESSAGE_DEBUG = addMessageLabel();
    setMessageColor(MESSAGE_DEBUG, cColorf(0.8, 0.8, 0.8));
}

void initGLDisplay(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve the resolution of the computer display and estimate the position
    // of the GLUT window so that it is located at the center of the screen
    int screenW = glutGet(GLUT_SCREEN_WIDTH);
    int screenH = glutGet(GLUT_SCREEN_HEIGHT);
    int windowPosX = (screenW - WINDOW_SIZE_W) / 2;
    int windowPosY = (screenH - WINDOW_SIZE_H) / 2;
    chai.displayW = WINDOW_SIZE_W;
    chai.displayH = WINDOW_SIZE_H;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(WINDOW_SIZE_W, WINDOW_SIZE_H);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow(argv[0]);
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutMouseFunc(mouseButtons);
    glutMotionFunc(mouseDrag);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI 3D");

    // create a mouse menu (right button)
    glutCreateMenu(menuSelect);
    glutAddMenuEntry("Full screen", OPTION_FULLSCREEN);
    glutAddMenuEntry("Window display", OPTION_WINDOWDISPLAY);
    glutAddMenuEntry("Retract handle", OPTION_RETRACT);
    glutAddMenuEntry("Resume handle", OPTION_RESUME);
    glutAddMenuEntry("Test on Plane", OPTION_TEST_PLANE);
    glutAttachMenu(GLUT_RIGHT_BUTTON);

}

void resizeWindow(int w, int h)
{
    // update the size of the viewport
    chai.displayW = w;
    chai.displayH = h;
    glViewport(0, 0, chai.displayW, chai.displayH);

    chai.rootLabels->setPos(0,chai.displayH, 0);
}

void mouseButtons(int button, int state, int x, int y) {
    if(button == GLUT_LEFT_BUTTON) {
        if(state == GLUT_UP) {
            chai.mouseClicked = false;
       }

        if(state == GLUT_DOWN) {
            if(!chai.mouseClicked) {
                chai.mouseClickOriginX = x;
                chai.mouseClickOriginY = y;
                chai.mouseClicked = true;
            }
        }
    }

    if(button == GLUT_WHEEL_UP && state == GLUT_DOWN) {
        // mouse scroll wheel up zooms in
        chai.cameraDistance = cClamp(chai.cameraDistance - 20.0, 20.0, 800.0);
    }

    if(button == GLUT_WHEEL_DOWN && state == GLUT_DOWN) {
        // mouse scroll wheel down zooms out
        chai.cameraDistance = cClamp(chai.cameraDistance + 20.0, 20.0, 800.0);
    }
}

void mouseDrag(int x, int y)
{
    chai.cameraRotX += (double)(x - chai.mouseClickOriginX) /
            chai.displayW * 2*CHAI_PI;
    chai.cameraRotY += (double)(y - chai.mouseClickOriginY) /
            chai.displayH * 2*CHAI_PI;

    chai.mouseClickOriginX = x;
    chai.mouseClickOriginY = y;
}

void updateCamera(double elapsedSec) {

    if(!chai.mouseClicked) {
        // decay camera back to no rotation when mouse is released
        double mult = pow(0.98, elapsedSec * 1000);
        chai.cameraRotX *= mult;
        chai.cameraRotY *= mult;
    }
    cVector3d eye;

    double tX, tY;
    tX = -chai.cameraRotX;
    tY = -chai.cameraRotY;

    // start at (0, 0, dist)
    cVector3d start = cVector3d(0, 0, chai.cameraDistance);

    cMatrix3d rotX = cRotMatrix(cVector3d(0, 1, 0), tX);
    cVector3d newXAxis = cMul(rotX, cVector3d(1,0,0));

    cMatrix3d rotY = cRotMatrix(newXAxis, tY);

    cVector3d pos = cMul(rotY, cMul(rotX, start));
    cVector3d lookAt = cVector3d(0,0,0);
    cVector3d up = cCross(pos - lookAt, newXAxis);

    /*printf("pos = (%.0f, %.0f, %.0f)\n", pos.x, pos.y, pos.z);
    printf("lookAt = (%.0f, %.0f, %.0f)\n", lookAt.x, lookAt.y, lookAt.z);
    printf("up = (%.0f, %.0f, %.0f)\n", up.x, up.y, up.z);
*/
    chai.camera->set(pos, lookAt, up);
}

//---------------------------------------------------------------------------

double uniform(double min, double max) {
    return (max-min)*((double)rand() / (double)RAND_MAX) + min;
}

void keySelect(unsigned char key, int x, int y)
{
    // escape key
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        shutdown();
    }
    /*
    if (key == 'm') {
        cVector3d newPoint;
        newPoint.set(uniform(-20, 20), uniform(-20,20), 0);
        workspace.moveToPoint->moveToPoint(newPoint);
    }

    if (key == 'a')
        chai.moveToPoint->abort();
       */
}

//---------------------------------------------------------------------------

void menuSelect(int value)
{
    switch (value)
    {
        // enable full screen display
        case OPTION_FULLSCREEN:
            glutFullScreen();
            break;

        // reshape window to original size
        case OPTION_WINDOWDISPLAY:
            glutReshapeWindow(WINDOW_SIZE_W, WINDOW_SIZE_H);
            break;

        case OPTION_RETRACT:
            hapticRetractHandle();
            break;

        case OPTION_RESUME:
            hapticResume();
            break;

        case OPTION_TEST_PLANE:
        	hapticReleaseForTesting();
        	break;
    }
}

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// ON SCREEN MESSAGES FOR DEBUGGING
//---------------------------------------------------------------------------

unsigned int addMessageLabel() {
    cLabel * label = new cLabel();
    label->m_font->setFontFace("courier12");

    chai.debugLabels.push_back(label);
    chai.rootLabels->addChild(label);
    int index = chai.debugLabels.size() - 1;

    label->setPos(10, -20 - index*20, 0);
    updateMessage(index, "");
    setMessageColor(index, cColorf(0.6, 0.6, 0.6));

    return index;
}

void setMessageColor(unsigned index, cColorf color) {
    if(index >= chai.debugLabels.size())
    {
        printf("Error: invalid message number");
        return;
    }

    chai.debugLabels[index]->m_fontColor = color;
}

void updateMessage(unsigned int index, const char* message) {
    if(index >= chai.debugLabels.size())
    {
        printf("Error: invalid message number");
        return;
    }

    if(chai.debugLabels[index]->m_string.compare(message) != 0)
        chai.debugLabels[index]->m_string.assign(message);
}

void updateMessage(unsigned int index, string message) {
    updateMessage(index, message.c_str());
}

void updateGraphics(void)
{
    double elapsedSec;
    elapsedSec = simClockGraphics.stop();
    simClockGraphics.start(true);

    updateCamera(elapsedSec);

    // set the position message
    char strMessage[200];
    cVector3d pos = haptic.posRig;
    snprintf(strMessage, 200, "Haptic position : %+.0f, %+.0f, %+.0f mm", pos.x, pos.y, pos.z);
    updateMessage(MESSAGE_HAPTICPOSITION, strMessage);

    cVector3d vel = haptic.velRig;
    double speed = vel.length();
    if(speed < 2) // denoise at low speeds
        speed = 0;
    snprintf(strMessage, 200, "Haptic speed : %6.1f mm/s", speed);
    updateMessage(MESSAGE_HAPTICVELOCITY, strMessage);
    if(workspace.slowDownDragField != NULL && workspace.slowDownDragField->getFieldActive() &&
            speed > workspace.slowDownDragField->getActivateSpeed())
        setMessageColor(MESSAGE_HAPTICVELOCITY, cColorf(1.0, .5, .5));
    else
       setMessageColor(MESSAGE_HAPTICVELOCITY, cColorf(1.0, 1.0, 1.0));

    if(haptic.hitObstacle)
        snprintf(strMessage, 200, "Hit obstacle!");
    else
        snprintf(strMessage, 200, "");
    updateMessage(MESSAGE_HITOBSTACLE, strMessage);

    if(!haptic.onScreenPlane)
        snprintf(strMessage, 200, "Pulled off screen plane!");
    else if(haptic.atWorkspaceEdge)
        snprintf(strMessage, 200, "Edge of Workspace!");
    else
        snprintf(strMessage, 200, "");
    updateMessage(MESSAGE_WORKSPACEEDGE, strMessage);

    // render world
    chai.camera->renderView(chai.displayW, chai.displayH);

    //printf("time render: %f\n", simClockGraphics.getCurrentTimeSeconds());

    // Swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

    // inform the GLUT window to call updateGraphics again (next frame)
    if (chai.simulationFinished) {
        int win = glutGetWindow();
        if(win != 0) glutDestroyWindow(win);
    } else {
        // schedule redraw at 60 Hz
        glutTimerFunc(1000.0/20.0, redrawTimerFn, 0);
    }
}

