
Wii U GCN Adapter Plus
======================

Tool for using the Wii U GameCube Adapter on Linux.

It's functionality is mainly to translate between a USB endpoint in software and an input event device in software. The input event device represents the final software controller.

Therefor, it creates an adapter device (`struct libusb_device`) that listens to un/plugged USB devices.
And it creates/destroys input event devices (`struct udev_device`) on the fly.

Each adapter runs a loop that examines connection status and writes and reads the input event device in each cycle. The USB endpoint is read before and written after that, also in each cycle.

When it reads a USB message from the adapter device, it sends an input event to the input event device. When it reads a force feedback input event from the input event device, it writes a rumble message to the USB endpoint. Both is alternating.


**Upcoming** Improvements (soon)
--------------------------------

* Add complete button mapping capability

* Fix libusb refcounting bug where creating and destroying adapters would not count device references.

* Big refactoring to make code better maintainable
  - throws away `-r` option and `./old/*` python code

**New** Improvements
---------------------------

* comprehensive analog input configuration (axes)
  - define custom mapping of scales to analog axes and define custom mapping of axes to analog inputs
  - split analog inputs into two output axes
  - suport for additional output axes such as ABS_WHEEL, ABS_GAS and ABS_BRAKE.

* try command line option `--claim`, maybe it helps against libusb ERRORs

* ~~spoofing options (which are likely mostly useless) to spoof XBOX controller meta data (use xboxdrv instead)~~
  - has been removed because the adapter then cannot distinguish between genuine and fake controllers.
    - maybe it would work to send a message to controllers and check the response.
    - much more complicated
  - The previous implementation did not support replugging (orphaned adapter devices on startup) and hotplugging of spoofed controllers.

* add many new mapping options and better button choices optimized for PC gaming
  - choose a button for Z
  - triggers as binary button
  - remapping D-pad to remaining XBOX buttons
  - remapping control sticks to D-pads (even sensitive duty cycling available)

* bug fixes related to shoulder button codes and added options such as --trigger-buttons, or uinput_create()

* add help text with "--help" option

* support alternative A, B, X, Y, Z gamepad button names (games use one or the other)

* don't die on libusb interrupts

* trigger-shoulder complementary mode (when shoulder button is pressed, reset the trigger input)
* trigger-only mode as default (shoulder buttons (fully depressed triggers) activates the triggers as buttons instead)

* improved input range (better uinput_user_dev absinfo values) to improve the behaviour of the analog inputs, notably the L and R shoulder buttons
  - allow for full thumb stick movement (by matching min and max values closer to the actual range)
  - absfuzz is set for the purpose of ignoring input noise (stops the constant input triggering of L/R shoulder triggers, especially useful when remapped as discrete input)
  - absflat can be set as a deadzone for analog axes
  - manual absinfo values via command line options

**Next Feature Ideas**
--------

* fixing the bug that thumbstick to dpad mapping happens globally (not per controller)
* full button mapping capability
* separate configuration per GCN-Adapter port
  - copy global configuration variables as pointers into port struct to optionally override the globals

* adding working udev rules
* support modern GCN controller (with extra buttons)
* allow mapping multiple axes or buttons to the same input event
  - how to handle the interference??
* automated tests with dummy input
* programming and replay mode for custom button macros
* add relative axes to the names
* support additional axes (ABS_HAT[1,2,3][X,Y])
  - emit ABS_HAT0 by pressing D-pad (with or without active D-pad buttons)
* button recording and replaying when pressing BTN_SELECT
* simulate pressure sensitivity with a duty cycling mode for any buttons, activated when pressing Z and L or R
* configuration file for command line arguments
  - read from path
  - write command line options to path
  - add and print default configuration file path (which is always included)

Spoofing XBox or Playstation Controllers
----------------------------------------

* Does it work to spoof IDs and name to mimic XBOX controllers? What about playstation controllers?
  - No. Please feed the input event devices of this driver into xboxdrv using the `--evdev` option plus `--mimic-xpad`.
    - It's only necessary if your controller is not already detected by the software and you need to emulate XBOX behaviour (e.g. in Yooka Laylee)
    - use the `-D` option when you need multiple controllers (multiplayer games)
  - if your software recognizes this controller, you should not use xboxdrv, otherwise your software recognizes two controllers at the same time.

Configuration
-------------

You can write your command line options into a configuration file

```sh
cat <<-END
	--remap-dpad
	--trigger-buttons
	END > "$XDG_CONFIG_HOME/wii-u-adapter.opts"
```

and use them together with other options when running the command

```sh
wii-u-gc-adapter … $(cat "$XDG_CONFIG_HOME/wii-u-adapter.opts") …
```

Or simply save your command as a shell script

```sh
echo wii-u-gc-adapter --remap-dpad --trigger-buttons > "$XDG_CONFIG_HOME/wii-u-adapter.sh"
chmod a+x "$XDG_CONFIG_HOME/wii-u-adapter.sh"
```

which you can run by calling the script's name

```sh
"$XDG_CONFIG_HOME/wii-u-adapter.sh"
```

You can create individual configurations for different games this way.

Prerequisites
-------------
* libudev
* libusb(x) >= 1.0.16

New:

* glib >= 2.0  (simply because I really don't have to make my own hash table)
* GCC compiler (possibly, but you could test another one by swapping it in the Makefile)

Please update the Makefile and the `.vscode/c_cpp_properties.json`, when you add new dependencies.

Building
--------
Just run `make`. That's all there is to it!

Usage
-----
Simply run the program `wii-u-gc-adapter`. You maybe have to run it as root in order to
grab the USB device from the kernel and use the uinput interface. Both of
these can be worked around with udev rules, which I'm currently too lazy to
add at the moment. [That way also provides less control over options and configuration.]
To stop the program just kill it in any way you want.

Seperate virtual controllers are created for each one plugged into the adapter
and hotplugging (both controllers and adapters) is supported.

Quirks
------
* If all your controllers start messing with the mouse cursor, you can fix
  them with this xorg.conf rule. (You can place it in a file in xorg.conf.d)

````
Section "InputClass"
        Identifier "Wii U GameCube Adapter Blacklist"
        MatchProduct "Wii U GameCube Adapter Port "
        Option "Ignore" "on"
EndSection
````
