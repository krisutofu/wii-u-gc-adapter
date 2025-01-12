
wii-u-gc-adapter
================

Tool for using the Wii U GameCube Adapter on Linux

**New** (Fork Improvements)
---------------------------

* comprehensive analog input configuration (axes)
  - define custom mapping of scales to analog axes and define custom mapping of axes to analog inputs

* try command line option `--claim`, maybe it helps against libusb ERRORs

* spoofing options (which are likely mostly useless) to spoof XBOX controller meta data (use xboxdrv instead)

* add many new mapping options and better button choices optimized for PC gaming
  - map analog inputs to various axes with custom scales
  - choose a button for Z
  - triggers as binary button
  - remapping D-pad to remaining XBOX buttons
  - remapping control sticks to D-pads (even sensitive duty cycling available)

* bug fixes related to shoulder button codes and added options such as --trigger-buttons, or uinput_create()

* add help text with "--help" option

* support alternative A, B, X, Y, Z gamepad button names (games use one or the other)

* don't die on libusb interrupts

* trigger-shoulder complementary mode (when shoulder button is pressed, reset the trigger input)
* trigger-only mode as default (shoulder buttons (fully depressed triggers) activate the triggers as binary buttons instead)

* improved input range (better uinput_user_dev absinfo values) to improve the behaviour of the analog inputs, notably the L and R shoulder buttons
  - allow for full thumb stick movement (by matching min and max values closer to the actual range)
  - absfuzz is set for the purpose of ignoring input noise (stops the constant input triggering of L/R shoulder triggers, especially useful when remapped as discrete input)
  - absflat is set as a deadzone value for the L and R triggers
  - manual absinfo values via command line options

**Next Feature Ideas**
--------

* is it sufficient to spoof IDs and name to mimic XBOX controllers? What about playstation controllers?
* programming and replay mode for custom button macros
* per-port configuration (copy global configuration variables as pointers into port struct to optionally overridable the globals)
* support additional buttons for D-pad and Z (BTN_MODE (Guide button) which has no ingame relevance, other ones?)
* add relative axes to the names
* support additional axes (ABS_HAT0[XY] (analog D-pad), ABS_THROTTLE, ABS_RUDDER)
  - emit ABS_HAT0 by pressing D-pad (with or without active D-pad buttons)
* configuration file for command line arguments
* button recording and replaying when pressing BTN_SELECT
* duty cycling mode for any buttons when pressing Z and L or R

Prerequisites
-------------
* libudev
* libusb(x) >= 1.0.16

Building
--------
Just run `make`. That's all there is to it!

Usage
-----
Simply run the program. You'll probably have to run it as root in order to
grab the USB device from the kernel and use the uinput interface. Both of
these can be worked around with udev rules, which I'm currently too lazy to
add at the moment. To stop the program just kill it in any way you want.

Seperate virtual controllers are created for each one plugged into the adapter
and hotplugging (both controllers and adapters) is supported.

Quirks
------
* Input ranges on the sticks/analog triggers are defined to try to match the
  physical ranges of the controls. To remove this scaling run the program with
  the `--raw` flag.
* If all your controllers start messing with the mouse cursor, you can fix
  them with this xorg.conf rule. (You can place it in a file in xorg.conf.d)

````
Section "InputClass"
        Identifier "Wii U GameCube Adapter Blacklist"
        MatchProduct "Wii U GameCube Adapter Port "
        Option "Ignore" "on"
EndSection
````
