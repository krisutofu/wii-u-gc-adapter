Fork Improvements
=====

* add help text with "--help" option

* improved input range (better uinput_user_dev absinfo values) to improve the behaviour of the analog inputs, notably the L and R shoulder buttons
  - allow for full thumb stick movement (by matching min and max values closer to the actual range)
  - absfuzz is set for the purpose of ignoring input noise (stops the constant input triggering of L/R shoulder triggers, especially useful when remapped as discrete input)
  - absflat is set as a deadzone value for the L and R triggers
  - note: you can further configure axes in key remapping software

* support alternative A, B, X, Y, Z gamepad button names (games use one or the other)

* don't die on libusb interrupts

* Trigger-shoulder complementary mode (when shoulder button is pressed, reset the trigger input)
* Shoulder-only mode (pressing trigger activates the shoulder buttons, allows xboxdrv users to map the Dpad inputs to Trigger buttons, replacing trigger axes.)

wii-u-gc-adapter
================

Tool for using the Wii U GameCube Adapter on Linux

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
