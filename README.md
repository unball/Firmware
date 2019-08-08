# Repository for the robot Firmware code of the UnBall team
## Installation
The firmware code can be found at: [UnBall - Firmware](https://github.com/unball/Firmware.git).

This firmware is uploaded using the platformIO extention for VScode editor.
To install the VScode editor download the .deb file from [VScode](https://code.visualstudio.com/) and in the terminal use the following command: 
```bash
dpkg -i path/to/debFile/debFileName.deb 
```

Once the VScode editor is installed to install de platformIO extention follow the instrutions at [PlatformIO - installation](https://platformio.org/install/ide?install=vscode).

### Teensy loader
instructions on how to install the teensy loader can be found in the [pjrc - official web page](https://www.pjrc.com/teensy/loader_linux.html). When trying to upload any code to teensy with the platformIO the teensy loader will be automatically called.

## Uploading
**IF USING A EXTERNAL POWER SUPLY, USE THE ADAPTED NO POWER USB CABLE**

Using the platformIO extention options build and upload the code (pressing the teensy config button may be needed).