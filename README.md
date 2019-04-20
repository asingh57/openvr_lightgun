# openvr_lightgun


This tool allows all openVR supported VR controllers to be emulated as mouse input on a display. Alternatively, they can be used as lighguns for arcade games. You'll need an Arduino Leonardo. The program communicates with the arduino to emulate a mouse/keyboard combo.

Watch a demonstrative video by clicking on the video below: (Apologies for vertical video)
[![Sample here](https://img.youtube.com/vi/aBct3N1zhEM/maxresdefault.jpg)](https://youtu.be/aBct3N1zhEM)

Compile the arduino_program using the arduino ide onto the Leonardo. Note the COM port of your arduino.
Disable mouse acceleration on windows by following the instructions [here](https://www.gamingscan.com/how-to-disable-mouse-acceleration-in-windows/)

Setup your Oculus Rift/HTC Vive/any other VR device to the PC, Install SteamVR, finally compile the OpenVR_LightGun cpp with the windows compiler, run the compiled executable
Finally, follow the instructions on screen to calibrate and voila!

[Here's how to compile a cpp in Windows](https://docs.microsoft.com/en-us/cpp/build/walkthrough-compiling-a-native-cpp-program-on-the-command-line?view=vs-2019)

Future goals:
Scrap the arduino completely and learn how to write an emulated driver for Windows...
