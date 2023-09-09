# Repository for the robot Firmware code of the UnBall team

This repository is related to the firmware where it can receive communications (via wifi or radio) and it can control the robots.

## Attention 

If you in linux you seem to not have permission to pass the code into the board, try:
```bash
sudo chmod a+rw /dev/ttyACM0
#or 
sudo chmod a+rw /dev/ttyUSB0
```
the /ttyDeviceN is related to how your OS reads the board and the board drivers
N -> is related exclusively to the number of boards/usb devices into your computer :) 

## Installation
The firmware code can be found at: [UnBall - Firmware](https://github.com/unball/Firmware.git).

This firmware is uploaded using the platformIO extention for VScode editor.
To install the VScode editor download the .deb file from [VScode](https://code.visualstudio.com/) and in the terminal use the following command: 
```bash
dpkg -i path/to/debFile/debFileName.deb 
```

Once the VScode editor is installed to install de platformIO extention follow the instrutions at [PlatformIO - installation](https://platformio.org/install/ide?install=vscode).

## Uploading
**IF USING A EXTERNAL POWER SUPLY, USE THE ADAPTED NO POWER USB CABLE**

Use the platformIO extention options to build and upload the code.

## Debugging 

You can use the debug from PlataformIO with VSCode or change the line in ```main.cpp```:

```
#define WEMOS_DEBUG false
```

to 

```
#define WEMOS_DEBUG true
```

Note that to upload the firmware, you have to put WEMOS_DEBUG to false. 

## Note!!

### If you are using Linux: 

At ```platformio.ini``` the default upload port to the project is 

```
upload_port = /dev/ttyUSB0
```

But if you disconnect and connect the usb again, the port can be ```/dev/ttyUSB1```. And an error could occur. 

Besides that, in your OS the serial port could be ```/deb/ttyACM0``` or ```/deb/ttyACM1``` (if the situation above happens).

### If you are using Windows:

We recommend to comment the code line above because Windows handles serial ports in another way. 

### Caution 

If the plataform.io isn't showing all tty ports on ubuntu it probobably is related to BRLTTY modules, then we gonna have to remove it. Don't worry this module is related this module is for acessibility related to braile display onto ubuntu.

To start it you gotta list all the devices, just to check if it's working

```bash
ls /dev/ttyUSB* 
#or 
ls /dev/ttyACM*
```

if the port isn't showing there, the system hasn't laoded the board, then it probably is the module, to remove it:

```bash
sudo apt-get remove --auto-remove brltty
```

Unplug and replug your board, type again in Terminal
```bash
ls /dev/ttyUSB* 
#or 
ls /dev/ttyACM*
```

it probably fixed it, for you but if it hasn't fixed try the link below.

[Link to debug it or if you wanna see the original solution](https://forum.arduino.cc/t/ubuntu-arduino-ide-not-showing-any-ports/1043925/19)
