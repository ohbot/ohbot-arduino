/*****************************
* V1.8  Servo Controller for Ohbot Robot using Ohbrain board
*
* This sketch needs the following libraries which are both available under GNU public licence: Adafruit Neopixel, VarSpeedServo.
*
* To install the Adafruit Neopixel library select Sketch, Include Library, Manage Libraries.
*
* In the box that says Filter your search type Adafruit Neopixel
*
* Click on the box that appears and click the Install button.
*
* To install the VarSpeedServo library go to this link and download the zip file:
*
* https://github.com/netlabtoolkit/VarSpeedServo/releases/download/v1.1.3/VarSpeedServo.zip
*
* Select Sketch, Include Library, Add .ZIP Library.
*
* Browse to where you downloaded VarSpeedServo.zip � select it and click Open.
*
* This sketch also needs the wire library which is included in the Arduino environment
*
* *****************************
* This Servo Controller for Ohbot Robot is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This software is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.

* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*
* Overview
* The Servo Controller for Ohbot Robot receives commands on the serial port and converts these to instructions
* for moving servo motors and controlling LEDs.  It also allows the analogue inputs on the Arduino to be polled
*
*
*****************************
* V1.5  Added eye light LED support, correction to input response.
* V1.6 Draft add I2C support for inputs.
* V1.7 Added demo program support
* V1.8 Remapped pinlist for 32U4 based board
* Commands:
* m, d, a, x, n, t, r, l, i, g, e, h, c, v
* // {29D81537-AF38-4BE3-9BEE-4B6952124806}
static const GUID <<name>> =
{ 0x29d81537, 0xaf38, 0x4be3, { 0x9b, 0xee, 0x4b, 0x69, 0x52, 0x12, 0x48, 0x6 } };


* mss,ttt[,vvv]      MOVE
*  ss = servo number 0 to  11
*  ttt = position in degrees, if >180 then uS pulse width
*  vvv = speed 1-250, 1 slowest, 0 = full speed.
* defaults to centre (90 deg) and full speed with no parameters
*
* dss       DETACH
* detaches servo ss
* dx        DETACH ALL
*
* ass       ATTACH
* attaches servo ss
* ax        ATTACH ALL specified servos
*
* xss,uuuu      SET MAX PULSE
* set max value uuu  in uS pulse width for servo ss rep 180 deg
* nss,uuu     SET MIN PULSE
* set min value uuu in uS pulse width for servo ss rep 0 deg
* t         STORE
* store min and max values for all servos in EEPROM
* r         RESET
* sets all servos to midpoint.
*
*
* lnn,rrr,ggg,bbb
* set led nn - 0 or 1 if  2 eyes used to r, g b values
* blank is obviously lnn,0,0,0
*
* ipp       READ Analog
* read Analog port pp.
* responds with vpp,vvv where v is value of analog port.
*
* hpp,e,ttt
* start monitoring port pp
* e = event to change and ttt parameter
* e = 1 change greater than ttt 0 or not included any change
* e = 2 value rises above threshold ttt
* e = 3 value falls above threshold ttt
* for events 2 and 3, it will trigger once, and be reset when value falls back below/above threshold.
* kpp
* stop monitoring port pp

**special commands for setup
* e5798     erase eeprom
* Commands for I2C on D2,D3
* ca,uuuu
* set port address of I2C device to be addressed.
* cw,bb,bb,bb...
* write bytes bb...
* cr,nn
* read nn bytes from device
* returns
* v04,vvv,vvv,vvv....
* v
* returns version number
* Ports:
* Digital 4-13 servos or pin 13 for leds
* Analog A0-A5 inputs
* I2C - 2 default ports used.
* D2 I2C SDA Also servo 90
* D3 I2C SCL Also servo 91
* D0 Serial1 Rx Also servo 92
* D1 Serial1 Tx Also servo 93
* For reference with Ohbot
* Servo 0 HeadNod
* Servo 1 Head turn
* Servo 2 Eye turn
* Servo 3 Lid Blink
* Servo 4 Top Lip
* Servo 5 Bottom Lip
* Servo 6 Eye tilt
*/



#include <VarSpeedServo.h>
#include <EEPROM.h>
#include <avr/power.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

typedef struct _GUID {
	unsigned long  Data1;
	unsigned short Data2;
	unsigned short Data3;
	unsigned char  Data4[8];
} GUID;
// Which pin on the Arduino is connected to the NeoPixels?
#define LEDPIN            13

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      2
#define NumDPSteps    11
#define NumPSteps   19
#define NoOfServos    12
#define NoOfAnalogIns 6

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);
char disbuf[128];
VarSpeedServo servo0;
VarSpeedServo servo1;
VarSpeedServo servo2;
VarSpeedServo servo3;
VarSpeedServo servo4;
VarSpeedServo servo5;
VarSpeedServo servo6;
VarSpeedServo servo7;
VarSpeedServo servo8;
VarSpeedServo servo9;
VarSpeedServo servo90;
VarSpeedServo servo91;


VarSpeedServo servar[NoOfServos] =
{
	servo0,servo1,servo2,servo3,servo4, servo5, servo6, servo7, servo8, servo9,servo90, servo91
};

byte i2caddress = 0;
byte i2cbuffer[24];
byte attachar[NoOfServos] =
{
	1,1,1,1,1,1,1,0,0,0,0,0
};

int pinlist[NoOfServos] =
{
	4,5,6,7,8,9,10,11,12,13,2,3
};
int apinlist[NoOfAnalogIns] =
{
	A0,A1,A2,A3,A4,A5
};

int minvals[NoOfServos] =
{
	544,544,544,544,544,544,544,544,544,544,544,544
};

int maxvals[NoOfServos] =
{
	2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400
};

int lastpos[NoOfServos] =
{
	90,90,90,90,90,90,90,90,90,90,90,90
};

int avalues[NoOfAnalogIns] = //last values read from port
{
	0,0,0,0,0,0
};

int tvalues[NoOfAnalogIns] =
{
	0,0,0,0,0,0
};

int evalues[NoOfAnalogIns] =
{
	0,0,0,0,0,0
};



const unsigned int sentenceSize = 50;
const unsigned int serialnosize = 8;
const unsigned int sversize = 6;
const unsigned int daterefsize = 11;

static const GUID servoid =
{ 0x29d81537, 0xaf38, 0x4be3,{ 0x9b, 0xee, 0x4b, 0x69, 0x52, 0x12, 0x48, 0x6 } };
const uint8_t curver = 19;
const uint8_t defprogram[NumDPSteps] =
{
	0x05,0x00,
	0x05,0x01,
	0x05,0x02,
	0x05,0x03,
	0x05,0x06,
	0x06
};

const uint8_t defprocs[NumPSteps] =
{
	0xd2,
	0xd9, 90,50,100,
	0xd9, 30,50,100,
	0xd9, 150,50,200,
	0xd9, 90,50,100,
	0xd3,0x07
};
char sentence[sentenceSize];
int offsets[30];
int offsetcount;
char field[10];
uint8_t lguid[16];
char cursftwarever[sversize] = "1.8";
//boolean demomode;
boolean recdemo;
uint8_t PrgMem[100];
uint8_t PrgCnt;
uint8_t PrgLen;
uint8_t procoffset;


uint8_t noprocs;


void setup()
{
//	demomode = false;
	pinMode(A0, INPUT_PULLUP);
	PrgCnt = 0;
	PrgLen = 0;
	noprocs = 0;

	uint8_t eval = EEPROM.read(0);
	if ((eval == 0xff) || (eval != curver))
	{
		// no eeprom data
		// use default program
		loaddefp();
		writeEEPROM();
	}
	pixels.begin(); // This initializes the NeoPixel library.
	pixels.setPixelColor(0, pixels.Color(0, 0, 0)); //switch off
	pixels.setPixelColor(1, pixels.Color(0, 0, 0));
	pixels.show();

	Serial.begin(19200);
	while (!Serial)
	{
		delay(10);
	}
	Serial.print("Servo Control SVer:v");
	Serial.println(cursftwarever);
	ShowExtraInf();
	ShowMinMax();
}

// load default demo prog
void loaddefp()
{
	int ix;
	PrgLen = NumDPSteps;
	for (ix = 0; ix < PrgLen; ix++)
	{
		PrgMem[ix] = defprogram[ix];
	}
	procoffset = PrgLen;
	for (ix = 0;ix < NumPSteps;ix++)
	{
		PrgMem[ix + procoffset] = defprocs[ix];
		PrgLen++;
	}

}

void attachServos()
{
	int ix;
	for (ix = 0; ix < NoOfServos; ix++)
	{
		if (attachar[ix] == 1)
		{
			servar[ix].attach(pinlist[ix], minvals[ix], maxvals[ix]);
			servar[ix].write(lastpos[ix]);
		}
	}
}

void detachServos()
{
	int ix;
	for (ix = 0; ix < NoOfServos; ix++)
	{
		if (attachar[ix] == 1)  servar[ix].detach();

	}
}



void ShowMinMax()
{
	int ix;
	for (ix = 0; ix < NoOfServos; ix++)
	{
		Serial.print("Servo: ");
		Serial.print(ix);
		Serial.print(" on pin");
		Serial.print(pinlist[ix]);
		Serial.print(" min uS: ");
		Serial.print(minvals[ix]);
		Serial.print(" max uS: ");
		Serial.println(maxvals[ix]);

	}
}

void ShowExtraInf()
{
	sprintf(disbuf, "Guid = {%08lX-%04hX-%04hX-%02hhX%02hhX-%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX}",
		servoid.Data1, servoid.Data2, servoid.Data3,
		servoid.Data4[0], servoid.Data4[1], servoid.Data4[2], servoid.Data4[3],
		servoid.Data4[4], servoid.Data4[5], servoid.Data4[6], servoid.Data4[7]);
	Serial.println(disbuf);

}
void writeEEPROM()
{
	int iz;
//	int iy;
	int eprompntr = 1;
	EEPROM.write(0, (uint8_t)curver);//10 first version, 11 second version, 12 third 12 servo version
	for (int ix = 0; ix < NoOfServos; ix++)
	{
		EEPROM.write(eprompntr, lowByte(minvals[ix]));
		eprompntr++;
		EEPROM.write(eprompntr, highByte(minvals[ix]));
		eprompntr++;
		EEPROM.write(eprompntr, lowByte(maxvals[ix]));
		eprompntr++;
		EEPROM.write(eprompntr, highByte(maxvals[ix]));
		eprompntr++;
	};
	for (iz = 0; iz < NoOfServos; iz++)
	{
		EEPROM.write(eprompntr, attachar[iz]);
		eprompntr++;
	}
	uint32_t gfirst = servoid.Data1;
	uint8_t aval;
	for (int ix = 0; ix < 4;ix++)
	{
		aval = gfirst & 0xff;
		EEPROM.write(eprompntr, aval);
		eprompntr++;
		gfirst = gfirst >> 8;
	}
	gfirst = servoid.Data2;
	for (int ix = 0; ix < 2;ix++)
	{
		aval = gfirst & 0xff;
		EEPROM.write(eprompntr, aval);
		eprompntr++;
		gfirst = gfirst >> 8;
	}
	gfirst = servoid.Data3;
	for (int ix = 0; ix < 2;ix++)
	{
		aval = gfirst & 0xff;
		EEPROM.write(eprompntr, aval);
		eprompntr++;
		gfirst= gfirst >> 8;
	}
	for (int ix = 0; ix < 8;ix++)
	{
		EEPROM.write(eprompntr, servoid.Data4[ix]);
		eprompntr++;
	}
	EEPROM.write(eprompntr, PrgLen);
	eprompntr++;
	EEPROM.write(eprompntr, procoffset);
	eprompntr++;
	for (int ix = 0; ix < PrgLen; ix++)
	{
		EEPROM.write(eprompntr, PrgMem[ix]);
		eprompntr++;
	}
}


int readEEPROM()
{
	int eprompntr = 1;
//	int resval;
	int cmdno;
	cmdno = (int)EEPROM.read(0);

	for (int ix = 0; ix < NoOfServos; ix++)
	{
		minvals[ix] = EEPROM.read(eprompntr); // lo byte
		eprompntr++;
		minvals[ix] = minvals[ix] | EEPROM.read(eprompntr) << 8;
		eprompntr++;
		maxvals[ix] = EEPROM.read(eprompntr); // lo byte
		eprompntr++;
		maxvals[ix] = maxvals[ix] | EEPROM.read(eprompntr) << 8;
		eprompntr++;
	};
	for (int ix = 0; ix < NoOfServos; ix++)
	{
		attachar[ix] = EEPROM.read(eprompntr);
		eprompntr++;
	}

	for (int ix = 0; ix < 16; ix++)
	{
		lguid[ix] = EEPROM.read(eprompntr);
		eprompntr++;
	}

	PrgLen = EEPROM.read(eprompntr);
	eprompntr++;
	procoffset = EEPROM.read(eprompntr);
	eprompntr++;
	for (int ix = 0; ix < PrgLen;ix++)
	{
		PrgMem[ix] = EEPROM.read(eprompntr);
		eprompntr++;
	}

	return cmdno;
}

void checkinput()
{
	int ix;
	int evno;
	// do monitoring

	for (ix = 0; ix < NoOfAnalogIns; ix++)
	{
		evno = evalues[ix];

		if (evno != 0)
		{
			int resval = analogRead(apinlist[ix]);
			switch (evno)
			{
			case 1:
			{
				if (abs(resval - avalues[ix])> tvalues[ix])
				{
					Serial.print("v");
					Serial.print(ix);
					Serial.print(",");
					Serial.println(resval);
					avalues[ix] = resval;
				}

				break;
			}

			case 2:
			{
				if (resval <= tvalues[ix])
				{
					avalues[ix] = 0;
				}
				else
				{
					if (avalues[ix] == 0)
					{
						Serial.print("v");
						Serial.print(ix);
						Serial.print(",");
						Serial.println(resval);
						avalues[ix] = resval;
					}
				};
				break;
			}
			case 3:
			{
				if (resval >= tvalues[ix])
				{
					avalues[ix] = 0;
				}
				else
				{
					if (avalues[ix] == 0)
					{
						Serial.print("v");
						Serial.print(ix);
						Serial.print(",");
						Serial.println(resval);
						avalues[ix] = resval;
					}
				};
				break;
			}

			}
		}
	}
}
void getfield(int ifield)
{
	// do this without passing params
	field[0] = '\0';
	int istart = offsets[ifield];
	int ifinish = offsets[ifield + 1] - 1;
	int iy = 0;
	for (int ix = istart; ix < ifinish; ix++)
	{
		field[iy] = sentence[ix];
		iy++;
	}
	field[iy] = '\0';
}

void setmidpoint()
{
	for (int ix = 0; ix < NoOfServos; ix++)
	{
		if (attachar[ix] == 1) servar[ix].write(90, 0);
	}
}

int getservonum()
{
	int sn = 0;
	char rpst[3] = "00";
	rpst[0] = field[1];
	rpst[1] = field[2]; // could be /0
	rpst[2] = '\0';
	sn = atoi(rpst);
	if (sn > 89)
	{
		sn = sn - 80;
		if (sn < 12)
			return sn;
		else
			return 0;

	}
	else
	{
		if (sn < NoOfServos)
			return sn;
		else
			return 0;
	}
}
void loop()
{
	boolean domore = true;
	byte inChar;
	int ix;
	unsigned int i;
	int target;
	int tspeed;
	uint8_t redval;
	uint8_t greenval;
	uint8_t blueval;

	offsets[0] = 0;
	offsetcount = 1;
	// wait around for a command then do it.
	i = 0;
	while (domore)
	{
		if (Serial.available())
		{
			// get the new byte:
			inChar = (char)Serial.read();
			if (inChar != '\n' && i < sentenceSize)
			{
				sentence[i] = inChar;
				i++;

				if (inChar == ',')
				{
					offsets[offsetcount] = i;
					offsetcount++;
				};
			}
			// if the incoming character is a newline, set a flag
			// so the main loop can do something about it:
			if (inChar == '\n')
			{
				offsets[offsetcount] = i + 1;
				sentence[i + 1] = '\0';
				domore = false;
			}
		}
		checkinput();
	}
	getfield(0);
	int servonum;
	if (field[0] == 'm')
	{
		servonum = getservonum();
		target = 90; // check about us or a set centre
		tspeed = 0;
		if (offsetcount > 1)
		{
			getfield(1);
			target = atoi(field);
			tspeed = 0; // full speed
			if (offsetcount > 2)
			{
				getfield(2);
				tspeed = atoi(field);
			}
		}
		servar[servonum].write(target, tspeed);
		lastpos[servonum] = target;

	}

	if (field[0] == 'd')
	{
		if (field[1] == 'x')
		{
			detachServos();
		}
		else
		{
			servonum = getservonum();
			servar[servonum].detach();
		}
	}

	if (field[0] == 'a')
	{
		if (field[1] == 'x')
		{
			attachServos();
		}
		else
		{
			servonum = getservonum();
			servar[servonum].attach(pinlist[servonum], minvals[servonum], maxvals[servonum]);
			servar[servonum].write(lastpos[servonum]);
			attachar[servonum] = 1;
		}
	}

	if (field[0] == 'x')
	{

		servonum = getservonum();
		if (offsetcount > 1)
		{
			getfield(1);
			target = atoi(field);
			maxvals[servonum] = target;
			servar[servonum].detach();
			servar[servonum].attach(pinlist[servonum], minvals[servonum], maxvals[servonum]);
		}
	}

	if (field[0] == 'n')
	{

		servonum = getservonum();
		if (offsetcount > 1)
		{
			getfield(1);
			target = atoi(field);
			minvals[servonum] = target;
			servar[servonum].detach();
			servar[servonum].attach(pinlist[servonum], minvals[servonum], maxvals[servonum]);
		}
	}

	if (field[0] == 't')
	{
		writeEEPROM();
		readEEPROM();
		ShowMinMax();

	}
	if (field[0] == 'r')
	{
		setmidpoint();

	}

	if (field[0] == 'i')
	{
		int ainp = getservonum();
		int resval = analogRead(apinlist[ainp]);
		Serial.print("v");
		Serial.print(ainp);
		Serial.print(",");
		Serial.println(resval);
	};

	if (field[0] == 'h')
	{
		int aport = getservonum();
		constrain(aport, 0, 7);
		getfield(1);
		target = atoi(field);
		constrain(target, 0, 3);
		evalues[aport] = target;
		int resval = analogRead(apinlist[aport]);
		if (target == 1)
			avalues[aport] = resval; // store current
		else
			avalues[aport] = 0;
		getfield(2);
		target = atoi(field);
		constrain(target, 0, 1024);
		tvalues[aport] = target;

	}

	if (field[0] == 'k')
	{
		int aport = getservonum();
		constrain(aport, 0, 7);
		evalues[aport] = 0;

	};

	if (field[0] == 'v')
	{
		Serial.print("SVer:v");
		Serial.println(cursftwarever);
	}


	if (field[0] == 'e')
	{
		if ((field[1] == '5') && (field[2] == '7') && (field[3] == '9') && (field[4] == '8'))
		{
			for (ix = 0; ix < 100; ix++)
			{
				EEPROM.write(ix, 0xff);
			}
			Serial.println("EEPROM erased");
		}
	}
	if (field[0] == 'l')
	{
		servonum = getservonum();
		getfield(1);
		redval = atoi(field);
		getfield(2);
		greenval = atoi(field);
		getfield(3);
		blueval = atoi(field);
		pixels.setPixelColor(servonum, pixels.Color(redval, greenval, blueval));
		pixels.show();

	}
	if (field[0] == 'c')
	{
		byte bufno = 0;
		// I2C commands
		if (field[1] == 'a')
		{
			getfield(1);
			i2caddress = atoi(field);
			Wire.begin();
		}
		if (field[1] == 'w')
		{
			bufno = 0;
			if (offsetcount > 23) offsetcount = 23;
			for (ix = 1; ix < offsetcount;ix++)
			{
				getfield(ix);
				i2cbuffer[ix - 1] = atoi(field);
				bufno++;
			}
			Wire.beginTransmission(i2caddress); //open communication
			for (ix = 0;ix < bufno; ix++)
			{
				Wire.write(i2cbuffer[ix]);
			}
			Wire.endTransmission();
		}
		if (field[1] == 'r')
		{
			getfield(1);
			bufno = atoi(field);
			Wire.requestFrom(i2caddress, bufno);
			int avbytes = Wire.available();
			if (bufno <= avbytes)
			{
				for (ix = 0; ix < avbytes;ix++)
				{
					i2cbuffer[ix] = Wire.read();
				}
				Serial.print("v04");
				for (ix = 0; ix < avbytes; ix++)
				{
					Serial.print(",");
					Serial.print(i2cbuffer[ix]);
				}
				Serial.println();
			}
		}
	}
}


