# ForceDimension SDK setup instructions:

The SDK contains the drivers and libraries that provide bindings for Chai to control the delta device. 
These come from Force Dimension's website but you need to register with them at http://forcedimension.com/support/download 
to get the link. I was using version sdk-3.4.5 with chai 2.3 and 3.0-rc1 for the code included here, but I believe the SDK code
can / should be safely upgraded. Essentially you download the sdk to somewhere accessible and run `make` inside.

If you go digging through the docs / code, note that we're using the Haptic SDK (dhd), not the Robotics SDK (drd), which is used
for having the robot move through a specific path using closed loop control. Chai talks to the Haptic SDK, which enables it
to poll the device's 3d position, and command a 3d force vector, at 4 kHz.

# Chai Setup instructions (for Linux):

1) Install Chai3D version 2.3.0 or latest v3 version from Chai's website http://www.chai3d.org/download/releases. The two folders in this repo are the functional versions for Chai3d v2 and v3.

2) Install some dependencies, in particular `freeglut3`. There may be others needed, please let me know and I'll add them here.
> `sudo apt install mesa-common-dev libusb-1.0-0-dev freeglut3 freeglut3-dev`

3) May need to create symlink to libusb so default Chai3D makefile can find it. (this is a hack)
> `sudo ln -s /usr/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/libusb-1.0.so`

4) cd into chai directory and build
> `make`

5) Can check if Omega 3 is connected using:
> `lsusb`

This probably won't print anything useful like device manufacturer, etc, but try unplugging usb cable and rerunning `lsusb` and look for change.

6) You can then go into the demos folder, try building them (if they aren't already) and running them to make sure things are working

## Providing raw access to the device haptics 

The issue is that, by default, the linux kernel does not allow
unprivileged raw access to the USB ports, which the delta.3 requires.

You can test that your device works fine by running the examples with
the 'sudo' command, e.g.:
> `sudo ./gravity`

The best way to fix it permanently (to avoid having to use 'sudo') is to
change the permissions assigned by the kernel on all Force Dimension USB
devices. You can do that by putting the included `11-forcedimension.rules` file in
`/etc/udev/rules.d`, and make sure it's attributes and ownership status
match that of the other rule files. You will need to restart you
computer afterwards (or restart the udev daemon and unplug/replug your
delta.3).

# Haptic controller code setup

Once the SDK and Chai3d are downloaded and successfully building, you can build the hapticController also using `make`. 
You will need to edit the `Makefile` to point the build script at the correct locations/versions for the SDK and for Chai3d.

There are a few settings that are fairly important within `haptic.h`:
* `HAPTIC_DEVICE_ANGLE_DEG`: the angle the haptic is physically tilted away from vertical plane (0 being vertical, 90 being horizontal). We used 45 degrees.
* `HAPTIC_WORKSPACE_ANGLE_DEG`: the angle the workspace plane is tilted away from vertical. We ran with this at 0 so the handle moved in a vertical plane, but you could tilt it a bit towards or away from the subject if more comfortable. 
* `HAPTIC_EFFECTOR_MASS`: the mass of the end-effector in kgs. This enables the device to provide accurate gravity compensation. Typically this is set iteratively by finding the smallest value where the device doesn't drift downwards.
* `WORKSPACE_WIDTH` and `WORKSPACE_HEIGHT`: the size of the workspace plane in mm, typically I used 250 mm for both, though I think the very corners of this rectangle aren't physically accessible. 
* Alternatively, `WORKSPACE_RADIUS` dictates the size of the circular workspace in mm. Version 2 is setup to use `cBoundingCircle` instead of `cBoundingPlane` (rectangle).
* There are also some useful parameters worth tweaking in `initHapticWorkspace()`, mainly those related to the screen plane that determine the speed with which the endpoint approaches and retracts from the screen plane. Too weak and the device approaches very slowly, too strong and its approach can scare the shit out of you. There's a state machine built into this that toggles the approach from being slow when the device is far from the screen plane, and then changes to a stiff planar spring when the device is "on" (near) the screen plane, so that it feels like a planar constraint.
* `PORT_IN`, `PORT_OUT`, `SERVER_IP`, and `LOOP_INTERVAL` in `network.cpp` configure where UDP packets are sent and received.

# Haptic code outline (how to read the code)

I'm following along here with v3, but it's similar in v2. Start in `hapticController.cpp` where main() is defined. First come a number of initialization calls:
* `hapticInitialize()` in `haptic.cpp` which calls a bunch of dhd SDK methods and starts the device. This will cause the device to find the limits of its encoders if it's just been powered on.
* `initScene()` in `environment.cpp` which sets up the Chai world object.
* `initHapticTool` in `environment.cpp` which tells Chai about the device. This is where a few scaling factors kick in that adjust the code for the delta.3 vs. the falcon. There are also some configuration steps that constrain what forces / stiffnesses Chai is willing to command and some parameters about the "proxy" object used in the rendering algorithms for the device.
* `initHapticWorkspace` in `haptic.cpp`. This is where all of the "objects" that exist in the haptic workspace (that can create forces) are created and initially configured. All of them are held in the global `HapticWorkspace workspace` defined here and declared in `haptic.h` to see a list of all the currently defined things. If you add a new thing, you should create a place to store it in `struct HapticWorkspace` and then initialize it in this function.
* `initGLDisplay` in `environment.cpp` will setup the OpenGL GLUT window to display your graphics. You can add right click menu options here to help with debugging. I've setup `glutMouseFunc` and `glutMotionFunc` so that you can drag the 3d display to rotate it to help debugging. You can also use the scroll wheel to zoom in and out.

`main` calls `runSimulation` in `environment.cpp`, which then kicks off a few threads by calling these non-blocking functions:
* `networkStart()` in `network.cpp` kicks off the UDP *send* thread that repeatedly sends the position/velocity out.
* `stateMachineStart()` in `stateMachine.cpp` kicks off the UDP receive/parse/command thread that polls for incoming UDP packets, parses the packets according to a very ad hoc byte packing scheme, and calls the appropriate function defined in `haptic.cpp` to manipulate the environment as requested.
* `updateHaptics()` in `envionment.cpp` which gets started using a high priority thread to do the force rendering. See below.
* `graphicsStart()` in `environment.cpp` which kicks off the GL rendering thread.

## Update haptics loop

Inside `updateHaptics()` in `environment.cpp` is the loop that calls Chai's main functions that do all the computations and control the device. This while loop will/should run at 4 kHz. The call to `chai.tool->computeInteractionForces()` will internally loop over everything that exists inside `chai.world` and ask these objects about the forces they would like to render, which is where your code gets to participate. The loop ends with a call to `hapticUpdateState` in `haptic.cpp`, which is a catch-all for anything that needs to happen at 4 kHz along with the haptic. This is where I polled to see whether the haptic was currently colliding with any obstacles (to relay this back to the task), and where we updated the filtered velocity and acceleration estimate. (The SDK computes a velocity estimate also but I think it's just a boxcar filter, which suffices for velocity but isn't great to differentiate for acceleration).

## State machine loop

The function `stateMachineUpdate()` in `stateMachine.cpp` is the loop that manipulates the environment as instructed by the task over UDP. This calls `parseBinPacket()` to unpack the bytes from the received packet and populates the fields of the `HapticCommand` struct in a `commandId` specific way. There's a bit of unfortunate redundancy here, in that both `parseBinPacket` and `stateMachineUpdate` have switch statements over all the haptic commands defined, but it made it cleaner to debug the two separately. `stateMachineUpdate` then calls the appropriate `haptic...` method, all of which are defined in `haptic.cpp`, to actually manipulate something about the environment, typically by changing the properties of one of the things in the `HapticWorkspace workspace`, or turning one of them on or off via `setEnabled()`.

I'd originally envisioned this as having more logic to it, i.e. having it actually be a state machine instead of a command relay loop. But it became quickly obvious that the logic was task specific and better handled by the task logic. What logic does remain here is inside the `haptic...` methods, which occasionally will turn off competing things when they operate. For instance, `hapticMoveToPoint` calls `hapticAbortPerturbation` and `hapticConstrainAbort` since it wouldn't make sense to have these things running concurrently. So there are some useful side effects here, but very little task state is maintained in this code aside from the states of the `HapticWorkspace` objects themselves. One exception is simply whether the handle is retracted or not (in `stateMachine.h`)

```
struct StateMachineState {
    bool isRetracted;
};

```

## Network loop

This function `networkUpdate` in `network.cpp` simply packs a UDP packet with the positions, velocities, forces, and any other info you'd like to send the task and sends it out repeatedly every `LOOP_INTERVAL` seconds.

## Graphics loop

The function `updateGraphics` in `environment.cpp` updates the Chai3d display window and can be helpful for displaying information and debugging info. You can see the couple of messages (text boxes) I've defined here (and that are created in `initScene()`). Chai v3 I think has also added some other display objects like scopes and dials I think, they might be helpful here for displaying values that change over time.

# Haptic effects!

There are two examples of haptic objects I'd point you too for reference. The first is simpler: `cPerturbationPulse` which simply renders a step force perturbation and draws it on screen as a red arrow. You can see how the object is defined, inheriting from Chai's `cGenericObject` and defining it's own `render` method (to handle the graphics) and that cooperates with `cEffectPerturbationPulse`, which inherits from `cGenericEffect`. In the Chai world, each object exists and can define it's own local coordinate frame, and has 0 or more haptic "effects" which render forces. The built-in effects are stiffness, viscosity, magnetism, etc, but defining your own enables you to create whatever force you want. The actual computation is in 

```cpp
bool cEffectPerturbationPulse::computeForce(const cVector3d& a_toolPos,
                                  const cVector3d& a_toolVel,
                                  const unsigned int& a_toolID,
                                  cVector3d& a_reactionForce)
```

which takes in the position and velocity in object-local coordinates, writes the force it would like to produce into `a_reactionForce`, returning `true` if the force is nonzero. For the perturbation force, the logic is simply ramping the force on and off gradually by scaling its magnitude. The rest is communication between the `cPerturbationPulse` object and its accompanying `cEffectPerturbationPulse` effect.

The second example is `cPlanarObstacle` which implements "2d" polygon obstacles that then get rendered as 3d box shapes whose vertical walls act as obstacles in the 2d plane. Rather than implement its own effect directly, it uses `cEffectSurface` and `cEffectViscosity` to do the actual rendering, and the object inherits from `cMesh` which is defined by Chai and maintains its own list of triangles that these effects will operate on. This allows it to benefit from the rendering algorithms Chai has already implemented for rendering forces on and within a complex triangle mesh, which is tricky to get right and is where using Chai really helps save you time. I've also implemented a `cEffectPlanarViscousTrap`, which basically makes the haptic "stick" to the obstacle after a collision. This I thought would be useful for training but it was not actually that helpful. The bulk of the work here is in `setPoints` and `rebuildMesh`, which take the list of polygon points, put them in clockwise order, and then build all the triangles that define the extruded 3d shape. After calling `clear()`, these triangles get added to the object using `newTriangle()`, both of which are defined by Chai in `cMesh`.

## Adding your own

I'd recomment copying one of the classes I've already defined and editing it to suit your needs, customizing the forces it produces and the way it renders itself visually to help with debugging and online monitoring. Then add an instance of your object inside `HapticWorkspace` and initialize it in `initHapticWorkspace()`. You can control it by calling its methods and setting its properties from new control functions you can define in `haptic.cpp`, which in turn you can call by adding new commands inside `stateMachine.cpp`.

Good luck!
