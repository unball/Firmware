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