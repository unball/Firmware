#ifndef PINS_H
#define PINS_H


namespace Pins {
	//**Encoder pins**
	int channelA = 2;
	int channelB = 20;

	//**Motor pins**
	int PWMA = 3;
	int AIN1 = 4;
	int AIN2 = 5;

	int PWMB = 21;
	int BIN1 = 22;
	int BIN2 = 23;

	int STBY = 6;

	//**Radio pins**
	int CE = 9;
	int CS = 10;

	//**Led pins**
	int red = 2;
	int green = 1;
	int blue = 0;
}

#endif
