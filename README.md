# Repository for the robot Firmware code of the UnBall team
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