#include <vector>
#include <cstdlib>
#include <algorithm>

#include "chai3d.h"
#include "world/CShapeLine.h"
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

#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif

#define GLUT_WHEEL_UP 3
#define GLUT_WHEEL_DOWN 4

//---------------------------------------------------------------------------
// DECLARED Variables
//---------------------------------------------------------------------------
ChaiData chai;
extern HapticState haptic;
extern HapticWorkspace workspace;

//---------------------------------------------------------------------------
// DECLARED CONSTANTS
//---------------------------------------------------------------------------

// initial size (width/height) in pixels of the display window
const int WINDOW_SIZE_W         = 800;
const int WINDOW_SIZE_H         = 800;

// mouse menu options (right button)
const int OPTION_FULLSCREEN     = 1;
const int OPTION_WINDOWDISPLAY  = 2;
const int OPTION_RETRACT = 3;
const int OPTION_RESUME = 4;

int MESSAGE_HEADER = -1;
int MESSAGE_BOUNDINGBOX = -1;
int MESSAGE_TOUCHINGSCREEN = -1;
int MESSAGE_HAPTICPOSITION = -1;
int MESSAGE_HAPTICVELOCITY = -1;
int MESSAGE_HITOBSTACLE = -1;
int MESSAGE_WORKSPACEEDGE= -1;
int MESSAGE_DEBUG = -1;

cFont *labelFont;

using namespace std;
using namespace chai3d;

cLabel* labelHapticDeviceModel;


//double lastHapicStateUpdateTime;
//double lastPrintTime;
//const double PRINT_INTERVAL = 2;


void graphicsTimer(int data);

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
    updateCamera();

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    //chai.camera->setClippingPlanes(-10.00, 8000.0);

    // create a light source and attach it to the camera
    chai.light = new cDirectionalLight(chai.world);
    chai.camera->addChild(chai.light);                   // attach light to camera
    chai.light->setEnabled(true);                   // enable light source
    //chai.light->setDirectionalLight(true);
    chai.light->setLocalPos(cVector3d( 0, 0, 500));  // position the light source
    chai.light->setDir(cVector3d(0, 0, -1.0));  // define the direction of the light beam

    /////////////////////////////////////////////////////////////////////////
    // DESCRIPTIVE LABELS
    /////////////////////////////////////////////////////////////////////////

    cGenericObject rootLabels;

    labelFont = NEW_CFONTCALIBRI20();

//    labelHapticDeviceModel = new cLabel(labelFont);
//    chai.camera->m_frontLayer->addChild(labelHapticDeviceModel);
//    std::string str = std::string("hello");
//    labelHapticDeviceModel->setText(str);

    chai.rootLabels = new cGenericObject();
    chai.camera->m_frontLayer->addChild(chai.rootLabels);
//    // use addMessageLabel to add and updateMessage to update



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

void initHapticTool() {
    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    chai.handler = new cHapticDeviceHandler();

    chai.handler->getDevice(chai.hapticDevice, 0);

    chai.hapticDevice->open();

    chai.hapticDevice->calibrate();

    // retrieve information about the current haptic device
    if (chai.hapticDevice)
    {
        chai.deviceInfo = chai.hapticDevice->getSpecifications();
    }

    if (chai.deviceInfo.m_model == C_HAPTIC_DEVICE_FALCON) {
      strcpy(chai.deviceName, "falcon");
      printf("Device falcon\n");
    } else if (chai.deviceInfo.m_model == C_HAPTIC_DEVICE_DELTA_3) {
      strcpy(chai.deviceName, "delta.3");
      printf("Device delta.3\n");
    } else {
      chai.hapticDevice->close();
      diep("Device not recognized. Terminating\n");
    }
    // create a 3D tool and add it to the world
    //chai.tool = new cGeneric3dofPointer(chai.world);
    chai.tool = new cToolCursor(chai.world);

    // connect the haptic device to the tool
    chai.tool->setHapticDevice(chai.hapticDevice);

    // Real-world: X/Y is screen plane, X points right, Y points towards ceiling, Z points normal away from screen
    // Haptic: X is away from baseplate, Z is up when baseplate vertical, Y is right looking at baseplate
    // Transform is:
    //      Permute X --> Z, Z --> Y, Y --> X (via mPermute)
    //      Convert meters to mm (via workspaceScaleFactor)

    // rotate the tool so that Z is out from the screen, X is right, Y is up
    cMatrix3d mPermute = cMatrix3d();
    mPermute.set(0,1,0, 0,0,1, 1,0,0);
    chai.tool->setLocalRot(mPermute);

    // set the device to convert m to mm
    double workspaceScaleFactor, forceScaleFactor;
    //printf("Name is %s\n", chai.deviceName);
    if (strcasecmp(chai.deviceName, "delta.3") == 0) {
        // meters to mm
        workspaceScaleFactor = 1000;
        forceScaleFactor = 1000;
    } else if (strcasecmp(chai.deviceName, "falcon") == 0) {
        // meters to mm, but compensate for the falcon's limited workspace
        workspaceScaleFactor = 3000;
        forceScaleFactor = 3000;
    } else {
        workspaceScaleFactor = 1000;
        forceScaleFactor = 1000;
    }

    printf("Haptic: Setting scale factor to %.0f\n", workspaceScaleFactor);

    chai.tool->setWorkspaceScaleFactor(workspaceScaleFactor);
    chai.workspaceScaleFactor = workspaceScaleFactor;

    chai.world->addChild(chai.tool);

    // initialize tool by connecting to haptic device
    chai.tool->start();
    chai.tool->setForcesON();

    // call configure device now that we've started the chai thread
    hapticConfigureDevice();

    // define a radius for the tool (both graphical and haptic proxy)
    chai.proxyRadius = 0.5;
    chai.tool->setRadius(chai.proxyRadius);

    // hide the device sphere. only show proxy.
    chai.tool->setShowContactPoints(true, false);

    // create a white cursor
    chai.tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    chai.tool->enableDynamicObjects(true);

    chai.tool->m_hapticPoint->setRadiusContact(chai.proxyRadius);
    chai.tool->m_hapticPoint->setRadiusDisplay(30);
    //chai.tool->m_proxyPointForceModel->m_collisionSettings.m_checkBothSidesOfTriangles = false;

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    //chai.tool->m_hapticPoint->m_algorithmFingerProxy->m_useDynamicProxy = true;

    // hide the device sphere. only show proxy.
    /*chai.tool->m_hapticPoint->m_sphereProxy->m_material->setRedSalmon();
    chai.tool->m_hapticPoint->m_sphereProxy->setShowEnabled(false);
    chai.tool->m_hapticPoint->m_sphereGoal->m_material->setGreenSea();
    chai.tool->m_hapticPoint->m_sphereGoal->setShowEnabled(false);*/
    //chai.tool->m_hapticPoint->m_sphereProxy->m_material->m_diffuse.set(1.0, 1.0, 0.0);
    //chai.tool->m_hapticPoint->m_sphereProxy->m_material->m_specular.set(1.0, 1.0, 1.0);
    //chai.tool->m_proxySphere->setRadius(chai.proxyRadius);

    chai.tool->m_hapticPoint->setShow(false, false);

    printf("Haptic: Proxy point force model radius = %f.1 mm\n", chai.proxyRadius);

    chai.tool->setWaitForSmallForce(false);

    // define a maximum stiffness that can be handled by the current
    // haptic device. The value is scaled to take into account the
    // workspace scale factor
    double stiffnessMax = (double)chai.deviceInfo.m_maxLinearStiffness / forceScaleFactor;
    printf("Haptic: Device Stiffness Max: %f\n", stiffnessMax);
    chai.scaledStiffnessMax = stiffnessMax;

    double linearDamping = (double)chai.deviceInfo.m_maxLinearDamping / forceScaleFactor;
    printf("Haptic: Linear damping max: %.2f\n", linearDamping);
    chai.scaledLinearDampingMax = linearDamping;

    double maxForce = (double)chai.deviceInfo.m_maxLinearForce;// / forceScaleFactor;
    chai.scaledForceMax = maxForce;
    printf("Haptic: Force max: %.2f\n", chai.scaledForceMax);

}

void initGLDisplay(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);


	#ifdef GLEW_VERSION
    	// initialize GLEW
    	glewInit();
	#endif

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
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void graphicsStart() {
    // start the main graphics rendering loop
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();
}

void graphicsTimer(int data)
{
    if (chai.simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

void endSimulation() {
    if(chai.simulationRunning) {
        printf("Haptic: stopping\n");
        // stop the simulation
        chai.simulationRunning = false;

        // wait for graphics and haptics loops to terminate
        while (!chai.simulationFinished) { cSleepMs(100); }

        // close haptic device
        chai.tool->stop();
        shutdown();
   }
}

void runSimulation() {
    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // initialize tool by connecting to haptic device
    printf("Haptic: starting\n");
    chai.tool->start();

    // simulation in now running
    chai.simulationRunning = true;
    chai.simulationFinished = false;

    // start the network thread
    networkStart();

    // start the haptic state machine
    stateMachineStart();

    //test if the curl force field works
    //hapticSetCurlForceFieldActive(cVector3d(0, 0.01, 0),cVector3d(-0.01, 0, 0));

    //test if the error Clamp works
    //hapticErrorClampActive(cVector3d(1,0,0), cVector3d(-1,0,0), cVector3d(1,1,0));

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // start the main graphics rendering loop
    graphicsStart();

    // close everything
    endSimulation();
}

void resizeWindow(int w, int h)
{
    // update the size of the viewport
    chai.displayW = w;
    chai.displayH = h;
    glViewport(0, 0, chai.displayW, chai.displayH);

    chai.rootLabels->setLocalPos(0,chai.displayH, 0);
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
            chai.displayW * 2*C_PI;
    chai.cameraRotY += (double)(y - chai.mouseClickOriginY) /
            chai.displayH * 2*C_PI;

    chai.mouseClickOriginX = x;
    chai.mouseClickOriginY = y;
}

void updateCamera() {
    if(!chai.mouseClicked) {
        // decay camera back to no rotation when mouse is released
        chai.cameraRotX *= 0.99;
        chai.cameraRotY *= 0.99;
    }
    cVector3d eye;

    double tX, tY;
    tX = -chai.cameraRotX;
    tY = -chai.cameraRotY;

    // start at (0, 0, dist)
    cVector3d start = cVector3d(0, 0, chai.cameraDistance);

    cMatrix3d rotX = cRotAxisAngleRad(0, 1, 0, tX);
    cVector3d newXAxis = cMul(rotX, cVector3d(1,0,0));

    cMatrix3d rotY = cRotAxisAngleRad(newXAxis.x(), newXAxis.y(), newXAxis.z(), tY);

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
        endSimulation();

        // exit application
        exit(0);
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
    }
}

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// ON SCREEN MESSAGES FOR DEBUGGING
//---------------------------------------------------------------------------

int addMessageLabel() {
    cLabel* label = new cLabel(labelFont);

    chai.debugLabels.push_back(label);
    chai.rootLabels->addChild(label);
//    label->setText("hello");
//    chai.camera->m_frontLayer->addChild(label);
    int index = chai.debugLabels.size() - 1;

    label->setLocalPos(10, -20 - index*20, 0);
    updateMessage(index, "");
    setMessageColor(index, cColorf(0.6, 0.6, 0.6));

    return index;
}

void setMessageColor(int index, cColorf color) {
    if(index < 0 || index >= chai.debugLabels.size())
    {
        printf("Error: invalid message number %d \n", index);
        return;
    }

    //chai.debugLabels[index]->m_fontColor = color;
}

void updateMessage(int index, const char* message) {
//	std::string str = message;
//	updateMessage(index, "Message: " + str);
//	chai.debugLabels[index]->setText(std::string("hello world 2"));
}

//void updateMessage( int index, const char* message) {
//    if(index >= chai.debugLabels.size())
//    {
//        printf("Error: invalid message number");
//        return;
//    }
//
//    if(chai.debugLabels[index]->getText().compare(message) != 0)
//        chai.debugLabels[index]->setText(message);
//}

void updateMessage(int index, std::string message) {
//    if(index < 0 || index >= chai.debugLabels.size())
//	{
//		printf("Error: invalid message number %d\n", index);
//		return;
//	}
//
//	printf("updating %d with %s\n", index, message.c_str());

	//chai.debugLabels[index]->setText("hello world");
    //if(chai.debugLabels[index]->getText().compare(message) != 0)
     //  chai.debugLabels[index]->setText(cStr(5));
}

double cameraTheta;
void updateGraphics(void)
{
    updateCamera();

    for(int iLabel = 0; iLabel <  chai.debugLabels.size(); iLabel++)
    {
    	chai.debugLabels[iLabel]->setLocalPos(10, -20 - iLabel*20, 0);
    }

    // set the position message
    char strMessage[200];
    cVector3d pos = chai.tool->getDeviceGlobalPos();
    snprintf(strMessage, 200, "Haptic position : %+.9f, %+.9f, %+.9f mm", pos.x(), pos.y(), pos.z());
    updateMessage(MESSAGE_HAPTICPOSITION, strMessage);

    cVector3d vel = haptic.velRig;
    double speed = vel.length();
    if(speed < 2) // denoise at low speeds
        speed = 0;
    snprintf(strMessage, 200, "Haptic speed : %6.1f mm/s", speed);
    updateMessage(MESSAGE_HAPTICVELOCITY, strMessage);

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

    // wait until all GL commands are completed
    glFinish();

    // Swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

    // inform the GLUT window to call updateGraphics again (next frame)
    if (chai.simulationRunning)
    {
        glutPostRedisplay();
    }
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    bool forceStarted = false;
    // reset clock
    chai.simClock.reset();
    chai.simClockHapticStateUpdate.reset();
    chai.simClockHapticStateUpdate.start(true);


//    lastHapicStateUpdateTime = chai.simClockHapticStateUpdate.getCurrentTimeSeconds();

    // main haptic simulation loop
    while(chai.simulationRunning)
    {
//    	double stateUpdateDt = chai.simClockHapticStateUpdate.getCurrentTimeSeconds() - lastHapicStateUpdateTime;
//    	lastHapicStateUpdateTime = chai.simClockHapticStateUpdate.getCurrentTimeSeconds();
//
//		if (chai.simClockHapticStateUpdate.getCurrentTimeSeconds() - lastPrintTime > PRINT_INTERVAL) {
//			printf("\n\n state updated delta T: %.6f \n\n", stateUpdateDt);
//			lastPrintTime = chai.simClockHapticStateUpdate.getCurrentTimeSeconds();
//		}
//
//		printf("%.9f  ",chai.simClockHapticStateUpdate.getCurrentTimeSeconds());


        // compute global reference frames for each object
        chai.world->computeGlobalPositions(true);

        // update position and orientation of tool
        chai.tool->updateFromDevice();

        // compute interaction forces
        if(forceStarted)
            chai.tool->computeInteractionForces();
        else
            forceStarted = true;

        chai.tool->applyToDevice();

        hapticUpdateState();
    }

    // exit haptics thread
    chai.tool->stop();
    cVector3d noForce = cVector3d(0,0,0);
    chai.hapticDevice->setForce(noForce);
    chai.simulationFinished = true;
}

//---------------------------------------------------------------------------
