/******************************************************************************
GeoCache Hunt Project (GeoCache.cpp)

This is skeleton code provided as a project development guideline only.  You
are not required to follow this coding structure.  You are free to implement
your project however you wish.

List Team Members Here:

1. Gabe
2. Oleh
3. TJ
4.

NOTES:

You only have 32k of program space and 2k of data space.  You must
use your program and data space wisely and sparingly.  You must also be
very conscious to properly configure the digital pin usage of the boards,
else weird things will happen.

The Arduino GCC sprintf() does not support printing floats or doubles.  You should
consider using sprintf(), dtostrf(), strtok() and strtod() for message string
parsing and converting between floats and strings.

The GPS provides latitude and longitude in degrees minutes format (DDDMM.MMMM).
You will need convert it to Decimal Degrees format (DDD.DDDD).  The switch on the
GPS Shield must be set to the "Soft Serial" position, else you will not receive
any GPS messages.

*******************************************************************************

Following is the GPS Shield "GPRMC" Message Structure.  This message is received
once a second.  You must parse the message to obtain the parameters required for
the GeoCache project.  GPS provides coordinates in Degrees Minutes (DDDMM.MMMM).
The coordinates in the following GPRMC sample message, after converting to Decimal
Degrees format(DDD.DDDDDD) is latitude(23.118757) and longitude(120.274060).  By
the way, this coordinate is GlobalTop Technology in Taiwan, who designed and
manufactured the GPS Chip.

"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C/r/n"

$GPRMC,         // GPRMC Message
064951.000,     // utc time hhmmss.sss
A,              // status A=data valid or V=data not valid
2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm)
N,              // N/S Indicator N=north or S=south
12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm)
E,              // E/W Indicator E=east or W=west
0.03,           // Speed over ground knots
165.48,         // Course over ground (decimal degrees format ddd.dd)
260406,         // date ddmmyy
3.05,           // Magnetic variation (decimal degrees format ddd.dd)
W,              // E=east or W=west
A               // Mode A=Autonomous D=differential E=Estimated
*2C             // checksum
/r/n            // return and newline

Following are approximate results calculated from above GPS GPRMC message
to the GEOLAT0/GEOLON0 tree location:

LAT DDDMM.MMMM 2307.1256 N = DDD.DDDDD = 23.118757
LON DDDMM.MMMM 12016.4438 E = DDD.DDDDD = 120.274060
Distance to GEOLAT0/GEOLON0 = 45335760 feet
Relative bearing to GEOLAT0/GEOLON0 = 217.519670 degrees;

******************************************************************************/

/*
Configuration settings.

These defines makes it easy to enable/disable certain capabilities
during the development and debugging cycle of this project.  There
may not be sufficient room in the PROGRAM or DATA memory to enable
all these libraries at the same time.  You are only permitted to
have NEO_ON, GPS_ON and SDC_ON during the actual GeoCache Flag
Hunt.
*/

//-TJay- I changed these just to test my display, 1 = on, 0 = off
#define NEO_ON 1		// NeoPixelShield
#define TRM_ON 0		// SerialTerminal
#define SDC_ON 0		// SecureDigital
#define GPS_ON 0		// Live GPS Message (off = simulated)
#define BrightnessPin A0

// define pin usage
#define NEO_TX	6		// NEO transmit
#define GPS_TX	7		// GPS transmit
#define GPS_RX	8		// GPS receive

// GPS message buffer
#define GPS_RX_BUFSIZ	128
char cstr[GPS_RX_BUFSIZ];

// global variables
uint8_t target = 0;		// target number
float heading = 0.0;	// target heading
float distance = 0.0;	// target distance

#if GPS_ON
#include <SoftwareSerial.h>
SoftwareSerial gps(GPS_RX, GPS_TX);

#endif

#if NEO_ON
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(40, NEO_TX, NEO_GRB + NEO_KHZ800);
#endif

#if SDC_ON
#include <SD.h>
#endif

/*
Following is a Decimal Degrees formatted waypoint for the large tree
in the parking lot just outside the front entrance of FS3B-116.
*/
#define GEOLAT0 28.594532
#define GEOLON0 -81.304437

#if GPS_ON
/*
These are GPS command messages (only a few are used).
*/
#define PMTK_AWAKE "$PMTK010,002*2D"
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_CMD_HOT_START "$PMTK101*32"
#define PMTK_CMD_WARM_START "$PMTK102*31"
#define PMTK_CMD_COLD_START "$PMTK103*30"
#define PMTK_CMD_FULL_COLD_START "$PMTK104*37"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_NMEA_OUTPUT_RMC "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#endif // GPS_ON

/*************************************************
**** GEO FUNCTIONS - BEGIN ***********************
*************************************************/

/**************************************************
Convert Degrees Minutes (DDMM.MMMM) into Decimal Degrees (DDD.DDDD)

float degMin2DecDeg(char *cind, char *ccor)

Input:
cind = string char pointer containing the GPRMC latitude(N/S) or longitude (E/W) indicator
ccor = string char pointer containing the GPRMC latitude or longitude DDDMM.MMMM coordinate

Return:
Decimal degrees coordinate.

**************************************************/
float degMin2DecDeg(char *cind, char *ccor)
{
	float degrees = 0.0;

	// add code here
	String degree;
	String minute;

	for (size_t i = 0; i < 3; i++)
	{
		degree += ccor[i];
	}

	for (size_t i = 3; i < 10; i++)
	{
		minute += ccor[i];
	}

	float min = atof(minute.c_str()) / 60;
	float deg = atof(degree.c_str());

	if (cind == "S" || cind == "W")
		degrees = (deg + min) * -1;
	else
		degrees = (deg + min) * 1;
	

	return(degrees);
}

/**************************************************
Calculate Great Circle Distance between to coordinates using
Haversine formula.

float calcDistance(float flat1, float flon1, float flat2, float flon2)

EARTH_RADIUS_FEET = 3959.00 radius miles * 5280 feet per mile

Input:
flat1, flon1 = first latitude and longitude coordinate in decimal degrees
flat2, flon2 = second latitude and longitude coordinate in decimal degrees

Return:
distance in feet (3959 earth radius in miles * 5280 feet per mile)
**************************************************/
float calcDistance(float flat1, float flon1, float flat2, float flon2)
{
	float distance = 0.0;
	// add code here
	float distance2 = 0.0;
	float diflat = 0.0;
	float diflon = 0.0;

	//Calculations
	diflat = radians(flat2 - flat1);
	flat1 = radians(flat1);
	flat2 = radians(flat2);
	diflon = radians(flon2 - flon1);

	distance = (sin(diflat / 2.0) * sin(diflat / 2.0));
	distance2 = cos(flat1);
	distance2 *= cos(flat2);
	distance2 *= sin(diflon / 2.0);
	distance2 *= sin(diflon / 2.0);
	distance += distance2;

	distance = (2 * atan2(sqrt(distance), sqrt(1.0 - distance)));

	return(distance);
}

/**************************************************
Calculate Great Circle Bearing between two coordinates

float calcBearing(float flat1, float flon1, float flat2, float flon2)

Input:
flat1, flon1 = first latitude and longitude coordinate in decimal degrees
flat2, flon2 = second latitude and longitude coordinate in decimal degrees

Return:
angle in degrees from magnetic north
**************************************************/
float calcBearing(float flat1, float flon1, float flat2, float flon2)
{
	float bearing = 0.0;
	// add code here
	float calc = 0.0;

	float x = 69.1 * (flat2 - flat1);
	float y = 69.1 * (flon2 - flon1) * cos(flat1 / 57.3);

	calc = atan2(y, x);
	bearing = degrees(calc);

	if (bearing <= 1)
		bearing = 360 + bearing;

	return(bearing);
}

/*************************************************
**** GEO FUNCTIONS - END**************************
*************************************************/

#if NEO_ON
/*
Sets target number, heading and distance on NeoPixel Display

NOTE: Target number, bearing and distance parameters used
by this function do not need to be passed in, since these
parameters are in global data space.

*/
void setNeoPixel(void)
{
	UpdateCompass(heading, 100);
	UpdateDistance(distance, 100);
	changeFlag(target, 100);
}

#endif	// NEO_ON

#if GPS_ON
/*
Get valid GPS message. This function returns ONLY once a second.

NOTE: DO NOT CHANGE THIS CODE !!!

void getGPSMessage(void)

Side affects:
Message is placed in global "cstr" string buffer.

Input:
none

Return:
none

*/
void getGPSMessage(void)
{
	uint8_t x = 0, y = 0, isum = 0;

	memset(cstr, 0, sizeof(cstr));

	// get nmea string
	while (true)
	{
		if (gps.peek() != -1)
		{
			cstr[x] = gps.read();

			// if multiple inline messages, then restart
			if ((x != 0) && (cstr[x] == '$'))
			{
				x = 0;
				cstr[x] = '$';
			}

			// if complete message
			if ((cstr[0] == '$') && (cstr[x++] == '\n'))
			{
				// nul terminate string before /r/n
				cstr[x - 2] = 0;

				// if checksum not found
				if (cstr[x - 5] != '*')
				{
					x = 0;
					continue;
				}

				// convert hex checksum to binary
				isum = strtol(&cstr[x - 4], NULL, 16);

				// reverse checksum
				for (y = 1; y < (x - 5); y++) isum ^= cstr[y];

				// if invalid checksum
				if (isum != 0)
				{
					x = 0;
					continue;
				}

				// else valid message
				break;
			}
		}
	}
}

#else
/*
Get simulated GPS message once a second.

This is the same message and coordinates as described at the top of this
file.  You could edit these coordinates to point to the tree out front (GEOLAT0,
GEOLON0) to test your distance and direction calculations.  Just note that the
tree coordinates are in Decimal Degrees format, and the message coordinates are
in Degrees Minutes format.

NOTE: DO NOT CHANGE THIS CODE !!!

void getGPSMessage(void)

Side affects:
Static GPRMC message is placed in global "cstr" null terminated char string buffer.

Input:
none

Return:
none

*/
void getGPSMessage(void)
{
	static unsigned long gpsTime = 0;

	// simulate waiting for message
	while (gpsTime > millis()) delay(100);

	// do this once a second
	gpsTime = millis() + 1000;

	memcpy(cstr, "$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C", sizeof(cstr));

	return;
}

#endif	// GPS_ON

void setup(void)
{
#if TRM_ON
	// init serial interface
	Serial.begin(115200);
#endif	

#if NEO_ON
	// init NeoPixel Shield
	pinMode(BrightnessPin, INPUT);
	strip.begin();
	strip.show(); // Initialize all pixels to 'off'
#endif	

#if SDC_ON
	/*
	Initialize the SecureDigitalCard and open a numbered sequenced file
	name "MyMapNN.txt" for storing your coordinates, where NN is the
	sequential number of the file.  The filename can not be more than 8
	chars in length (excluding the ".txt").
	*/
#endif

#if GPS_ON
	// enable GPS sending GPRMC message
	gps.begin(9600);
	gps.println(PMTK_SET_NMEA_UPDATE_1HZ);
	gps.println(PMTK_API_SET_FIX_CTL_1HZ);
	gps.println(PMTK_SET_NMEA_OUTPUT_RMC);
#endif		

	// init target button here
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// TJay's Code for Displaying data to neopixel
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if NEO_ON
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//UpdateCompass();
//Takes in a degree and wait time. It uses that degree to update which light on the "Compass" to turn red. A.K.A Point us in the right direction;
//Wait time is irrelevant currently, May take out in future;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UpdateCompass(uint32_t degree, uint8_t wait)
{
	//Make  a certain Light red based on degree
	if (degree > 337 || degree < 23)
		strip.setPixelColor(22, strip.Color(255, 0, 0));
	else
		strip.setPixelColor(22, strip.Color(0, 0, 255));

	if (degree > 22 && degree < 68)
		strip.setPixelColor(23, strip.Color(255, 0, 0));
	else
		strip.setPixelColor(23, strip.Color(0, 0, 255));

	if (degree > 67 && degree < 113)
		strip.setPixelColor(31, strip.Color(255, 0, 0));
	else
		strip.setPixelColor(31, strip.Color(0, 0, 255));

	if (degree > 112 && degree < 158)
		strip.setPixelColor(39, strip.Color(255, 0, 0));
	else
		strip.setPixelColor(39, strip.Color(0, 0, 255));

	if (degree > 157 && degree < 203)
		strip.setPixelColor(38, strip.Color(255, 0, 0));
	else
		strip.setPixelColor(38, strip.Color(0, 0, 255));

	if (degree > 202 && degree < 248)
		strip.setPixelColor(37, strip.Color(255, 0, 0));
	else
		strip.setPixelColor(37, strip.Color(0, 0, 255));

	if (degree > 247 && degree < 293)
		strip.setPixelColor(29, strip.Color(255, 0, 0));
	else
		strip.setPixelColor(29, strip.Color(0, 0, 255));

	if (degree > 292 && degree < 338)
		strip.setPixelColor(21, strip.Color(255, 0, 0));
	else
		strip.setPixelColor(21, strip.Color(0, 0, 255));
	strip.show();
	delay(wait * 2);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//UpdateDistance();
//Takes in a distance and wait time. It uses that distance to update the 31 lights that represent how close or far we are from the target;
//If we are more than 80 feet away, a yellow light will be displayed for every 80 feet.
//However if we are less than 80 feet away, a green light will be displayed for every 2.5 feet we are away from the target. 
//Wait time is irrelevant currently, May take out in future
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UpdateDistance(uint32_t distance, uint8_t wait)
{
	//If distance is greater than 80ft, light up a yellow light per 80ft
	if (distance > 80)
	{
		for (uint16_t i = 0; i < 31; i++) {
			{
				if (distance > i * 80)
				{

					if (i < 21)
						strip.setPixelColor(i, strip.Color(255, 255, 0));
					else if (i >= 21 && i + 3 < 29)
						strip.setPixelColor(i + 3, strip.Color(255, 255, 0));
					else if (i + 3 >= 29 && i + 6 < 40)
						strip.setPixelColor(i + 6, strip.Color(255, 255, 0));

				}
				else
				{
					if (i < 21)
						strip.setPixelColor(i, strip.Color(0, 0, 0));
					else if (i >= 21 && i + 3 < 29)
						strip.setPixelColor(i + 3, strip.Color(0, 0, 0));
					else if (i + 3 >= 29 && i + 6 < 40)
						strip.setPixelColor(i + 6, strip.Color(0, 0, 0));
				}

			}
		}
	}
	else // otherwise light up a green light every 2.5 ft
	{
		for (uint16_t i = 0; i < 31; i++) {
			{
				if (distance > i * 2.5f)
				{

					if (i < 21)
						strip.setPixelColor(i, strip.Color(0, 255, 0));
					else if (i >= 21 && i + 3 < 29)
						strip.setPixelColor(i + 3, strip.Color(0, 255, 0));
					else if (i + 3 >= 29 && i + 6 < 40)
						strip.setPixelColor(i + 6, strip.Color(0, 255, 0));

				}
				else
				{
					if (i < 21)
						strip.setPixelColor(i, strip.Color(0, 0, 0));
					else if (i >= 21 && i + 3 < 29)
						strip.setPixelColor(i + 3, strip.Color(0, 0, 0));
					else if (i + 3 >= 29 && i + 6 < 40)
						strip.setPixelColor(i + 6, strip.Color(0, 0, 0));
				}

			}
		}
	}
	strip.show();
	delay(wait * 2);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//UpdateFlag();
//Takes in a flag and wait time. Flag represents a number from 0-3, which referes to 1 of the four flags we are looking for;
//This function updates color of the target flag whch is located in the middle of the compass
//Wait time is irrelevant currently, May take out in future;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void changeFlag(uint16_t flag, uint8_t wait)
{
	if (flag == 0)
	{

		strip.setPixelColor(30, strip.Color(255, 0, 255));
	}
	else 	if (flag == 1)
	{

		strip.setPixelColor(30, strip.Color(0, 255, 255));
	}
	else 	if (flag == 2)
	{

		strip.setPixelColor(30, strip.Color(250, 200, 10));
	}
	else 	if (flag == 3)
	{

		strip.setPixelColor(30, strip.Color(127, 127, 127));
	}

	strip.show();
	delay(wait * 2);
}
#endif
void loop(void)
{
	// if button pressed, set new target

	// returns with message once a second
	getGPSMessage();

	// if GPRMC message (3rd letter = R)
	while (cstr[3] == 'R')
	{
		// parse message parameters

		// calculated destination heading

		// calculated destination distance

#if SDC_ON
		// write current position to SecureDigital then flush
#endif

		break;
	}

#if NEO_ON
	// set NeoPixel target display
	strip.setBrightness(map(analogRead(BrightnessPin), 0, 1023, 0, 255));
	setNeoPixel();

#endif		

#if TRM_ON
	// print debug information to Serial Terminal
	Serial.println(cstr);
#endif		
}