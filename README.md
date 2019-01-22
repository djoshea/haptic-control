# ForceDimension SDK setup instructions:

The SDK contains the drivers and libraries that provide bindings for Chai to control the delta device. 
These come from Force Dimension's website but you need to register with them at http://forcedimension.com/support/download 
to get the link. I was using version sdk-3.4.5 with chai 2.3 and 3.0-rc1 for the code included here, but I believe the SDK code
can / should be safely upgraded. Essentially you download the sdk to somewhere accessible and run `make` inside.

If you go digging through the docs / code, note that we're using the Haptic SDK (dhd), not the Robotics SDK (drd), which is used
for having the robot move through a specific path using closed loop control. Chai talks to the Haptic SDK, which enables it
to poll the device's 3d position, and command a 3d force vector, at 4 kHz.

# Chai Setup instructions (for Linux):

1) Install Chai3D version 2.3.0 or latest v3 version from Chai's website http://www.chai3d.org/download/releases. The two folders
in this repo are the functional versions for Chai3d v2 and v3.

2) Install some dependencies, in particular `freeglut3`. There may be others needed, please let me know and I'll add them here.
> `sudo apt install mesa-common-dev libusb-1.0-0-dev freeglut3 freeglut3-dev

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

  sudo ./gravity

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
* HAPTIC_DEVICE_ANGLE_DEG: the angle the haptic is physically tilted away from vertical 
