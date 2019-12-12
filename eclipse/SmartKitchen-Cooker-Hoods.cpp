#include <Arduino.h>

#define USE_LCD     	1		//WH 1202
#define USE_DHT     	1		//DHT22 (2302)
#define USE_AMPERS  	1 		//ACS712
#define USE_LOWPOWER   	1
#define USE_BUTTON   	1
#define USE_EPPROM   	1
#define USE_DEBUGSER   	0

#if USE_EPPROM
#include <EEPROM.h>
#endif

#if USE_BUTTON
#include <EasyButton.h>
#endif

#if USE_LOWPOWER
//#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <LowPower.h>
#endif

/**
 * IO PINS
 */

#if USE_LCD
#include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
#define LCD_COLUMNS 12
#define LCD_ROWS 	2
// LCD PINS
#define LCD_RS 	    4
#define LCD_E	 	5
#define LCD_D4	 	6
#define LCD_D5	 	7
#define LCD_D6	 	8
#define LCD_D7	 	9
#endif

#if USE_DHT
#include "./libraries/DHTstable-Lexxai/dht.h"
#define DHT1_pin 		11     // Pin sensor is connected to
#define DHT2_pin 		12     // Pin sensor is connected to
#define DHT_Alarm_value 18 	   // Difference for humidity for alarm
#endif

#define SYSLED	13

#define RELAY_IO 	A1      // Pin relay of power control
#define RELAY_HS 	10      // Pin relay of Him self
#define RELAY_ON   	LOW   	// Relay level for ON state
#define RELAY_OFF  	HIGH  	// Relay level for OFF state

#if USE_AMPERS
#define AMPERSSENSOR_IO         A0   //Pin for Anonalog input from ACS712 module
#define AmpsLevel_LOW           1    // Ampers,Low Level of measured value of current
#endif

#if USE_BUTTON
//Switches Button IO
#define SWITCH_PowerOFF 	2	//EXT INT0
#define SWITCH_Mode	 		3	//EXT INT1
#endif

/**
 * TIME DELAYS DEFINITION
 */

//#define MAX_WORKING_TIME   1000UL*60*60*12 UL   //12 hours
#define MAX_WORKING_TIME      		1000UL*60*60*12 // 12 hours
#define MAX_IORELAY_TIME      		1000UL*60*60*3  // 3 hours

#define AmperesAlarm_Start_Time   		1000UL*30      // 30 sec, How long HIGH Level must be before on relay
#define AmperesAlarm_Stop_Time    		1000UL*60*3    // 3 min, How long Low Level must be before off relay

#define HumidityAlarm_Start_Time  	1000UL*20UL      // 20 sec
#define HumidityAlarm_Stop_Time     1000UL*60UL*2    //2 min

#define Delay_AlarmAmperes_Start_Time    1000UL*60*2    //2 mins

#define DISPLAY_REFRESH_INTERVAL    2000UL

#ifdef EEPROM_h
#define EEPROM_ADDRES_DeviceMODE  0
#define EEPROM_ADDRES_DeviceAlarmDelay  EEPROM_ADDRES_DeviceMODE+1
#endif

#if USE_DHT
dht DHT;
#endif

/*
 *	Modes of device
 */
enum devmodes {
	general = 0, humidityandampers, ampersonly, humidityonly, manual, timedelay ,last
};

enum customLCDChars {
	A = 0, H, AH, DEGREE
};

int deviceMode = devmodes::general;
const char TextDeviceMode[] = { 'G', 'S', 'A', 'H', 'M', 'T' };

#define CHAR_IO_POWERED_ON  217
#define CHAR_IO_POWERED_OFF 218
#define CHAR_DEGRE 			0xEF
#define CHAR_1_2 			0xF2
#define CHAR_1_3 			0xF1
#define CHAR_1_4 			0xF0
#define CHAR_3_4 			0xF3
#define CHAR_0 				0x30
#define CHAR_10				0x7B
#define CHAR_12				0x7C
#define CHAR_15				0x7D
#define CHAR_INF			0xFD

/**
 * TRANSLATION OF LCD TEXT MESSAGES
 * by use tools http://robotosha.ru/electronics/lcd-hd44780.html
 **/
#define TEXT_LANG 'UKR'

#if TEXT_LANG == 'EN'
#define TEXT_MODE_CHANGED  "MODE CHANGED"
#define TEXT_GENERAL       "  General "
#define TEXT_HUMANDAMP     "  Strong  "
#define TEXT_AMPERESONLY    "Amperes only"
#define TEXT_HUMIDITYONLY  "Humid. only"
#define TEXT_MANUAL        "   Manual "
#define TEXT_MANUAL_CTRL   "MANUAL CONTROL"
#define TEXT_HS_POWER_OFF  "HS POWER OFF"
#define TEXT_IO_POWER_OFF  "IO POWER OFF"
#define TEXT_IO_POWER_ON   "IO POWER ON"
#define TEXT_IO_POWER_TSAFE   "SAFE TIMER"
#define TEXT_WELCOME1      "SMART KITCHEN"
#define TEXT_WELCOME2      "lexxai 2019"
#else if TEXT_LANG == 'UKR'
#define TEXT_MODE_CHANGED  "Pe\266\270\274 \267\274i\275e\275\275o"
#define TEXT_GENERAL       "  Oc\275o\263\275\270\271"
#define TEXT_HUMANDAMP     "   Cy\263op\270\271  "
#define TEXT_AMPERESONLY    "A\274\276ep \277i\273\304\272\270"
#define TEXT_HUMIDITYONLY  "Bo\273o\264ic\277\304 \277."
#define TEXT_MANUAL        "   Py\300\275\270\271"
#define TEXT_MANUAL_CTRL   "Py\300\275e \272epy\263."
#define TEXT_HS_POWER_OFF  "B\270\274\270\272a\306 ce\262e"
#define TEXT_IO_POWER_OFF  "B\270\277\307\266. \263\270\274\272\275"
#define TEXT_IO_POWER_ON   "B\270\277\307\266. y\263i\274\272"
#define TEXT_IO_POWER_TSAFE   "Ta\271\274ep \262e\267\276."
#define TEXT_WELCOME1      "* B\245T\261\243KA *"
#define TEXT_WELCOME2      "lexxai 2019"
#endif

const String TextDeviceModeF[] = { TEXT_GENERAL, TEXT_HUMANDAMP,
TEXT_AMPERESONLY,
TEXT_HUMIDITYONLY, TEXT_MANUAL, "TimeADelay " };
/*
 * Humidity and temperature sensor values DHT22 (2302)
 */
float temp1, humidity1;
float temp2, humidity2;
//calibration for sensor difference
const float ctemp12 = 0.2, chumidity12 = -4;
//float delta;
int humidity_delta;

//time of start events
unsigned long StartHumidityAlarm_timer = 0;
unsigned long StopHumidityAlarm_timer = 0;
unsigned long StartIORelay_timer = 0;
unsigned long delayAlarmAmperes_timer = 0;
unsigned long AmpersLowLevel_timer = 0;
unsigned long AmpersHighLevel_timer = 0;
unsigned long RefreshDiplay_timer = 0;

//status of alarms
bool alarmHumidity = false;
bool alarmAmperes = false;
bool statusSensor = false;
unsigned int statusSensorError = 0;
bool useDelayedStartAlarmAmperes = true;
bool isDelayInAlarmAmperes = false;

bool isMAXTimeForRelayIO = false;

/**
 * Detection value of AC voltage from CURRENT MEASURE module
 * for control PowerLOAD state
 */

const int mVperAmp = 66; // use 100 for 20A Module and 66 for 30A Module ACS712

double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;

#ifdef _EasyButton_h
//EasyButton(uint8_t pin, uint32_t debounce_time = 35, uint8_t pullup_enable = true, uint8_t invert = true)
EasyButton button_poweroff(SWITCH_PowerOFF, 100, true, true);
EasyButton button_poweroff2(SWITCH_PowerOFF, 100, true, true);
EasyButton button_mode(SWITCH_Mode, 100, true, true);
#define EasyButton_Interupt 1
#endif

#if USE_LCD
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
#endif

/**
 * Functions
 */

/*
 *  storeDeviceMode
 */
bool storeDeviceMode() {
#ifdef EEPROM_h
	EEPROM.update(EEPROM_ADDRES_DeviceMODE, deviceMode);
	return (EEPROM.read(EEPROM_ADDRES_DeviceMODE) == deviceMode);
#endif
}
/*
 * storeDeviceModeDelayed
 */
void storeDeviceModeDelayed() {
#ifdef EEPROM_h
	EEPROM.update(EEPROM_ADDRES_DeviceAlarmDelay, useDelayedStartAlarmAmperes);
#endif
}
/*
 * enterSleep device
 */
void enterSleep(void) {
#ifdef _AVR_SLEEP_H_
//  /* Setup pin2 as an interrupt and attach handler. */
//  attachInterrupt(0, pin2Interrupt, LOW);
//  delay(100);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	//set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	sleep_mode();
	/* The program will continue from here. */
	/* First thing to do is disable sleep. */
	sleep_disable();
//	Serial.println("Wakeup");
//	Serial.println(millis());
//	delay(20);
#endif
}
/**
 * RELAY ON /OFF for
 * IO - Controled device
 * HS - Himself
 */

void relay_himself_off() {
	digitalWrite(RELAY_HS, RELAY_OFF);
}

void relayoff() {
	isMAXTimeForRelayIO=false;
	StartIORelay_timer = millis();
	digitalWrite(RELAY_IO, RELAY_OFF);
}

bool isRelayIO_ON() {
	return (digitalRead(RELAY_IO) == RELAY_ON);
}

void relayon() {
	isDelayInAlarmAmperes = false;
	if (isMAXTimeForRelayIO){
		return;
	}
	digitalWrite(RELAY_IO, RELAY_ON);
}


void onButton_poweroff_inerrupt() {
#if USE_BUTTON
	button_poweroff.read(INTERRUPT);
	button_poweroff2.read(INTERRUPT);
#endif
}
void onButton_mode_inerrupt() {
#if USE_BUTTON
	button_mode.read(INTERRUPT);
#endif
}

void onButton_poweroff_pressed() {
	if (deviceMode == devmodes::timedelay) {
		useDelayedStartAlarmAmperes = !useDelayedStartAlarmAmperes;
		storeDeviceModeDelayed();
#if USE_LCD
		lcd.clear();
		lcd.print("setTimeADelay");
		lcd.setCursor(5, 1);
		lcd.print(useDelayedStartAlarmAmperes ? "on" : "off");
		delay(2000);
#endif
	} else {
#if USE_LCD
		lcd.clear();
		lcd.print(TEXT_MANUAL_CTRL);
		lcd.setCursor(0, 1);
		lcd.print(TEXT_IO_POWER_OFF);
#endif
		deviceMode = devmodes::manual;
		relayoff();
#if USE_LCD
		delay(2000);
#endif
	}
}

void onButton_poweroff_longpressed() {
#if USE_LCD
	lcd.clear();
	lcd.print(TEXT_MANUAL_CTRL);
	lcd.setCursor(0, 1);
	lcd.print(TEXT_IO_POWER_ON);
#endif
	deviceMode = devmodes::manual;
	isMAXTimeForRelayIO=false;
	StartIORelay_timer = millis();
	relayon();
#if USE_LCD
	delay(2000);
#endif
}
void onButton_poweroff_hs_longpressed() {
#if USE_LCD
	lcd.clear();
	lcd.print(TEXT_MANUAL_CTRL);
	lcd.setCursor(0, 1);
	lcd.print(TEXT_HS_POWER_OFF);
#endif
	relayoff();
	delay(2000);
	relay_himself_off();
	enterSleep();
}

void onButton_mode_pressed() {
	deviceMode++;
	if (deviceMode == devmodes::last) {
		deviceMode = devmodes::general;
	}
	if (deviceMode != devmodes::timedelay) {
		storeDeviceMode();
	}
#if USE_LCD
	lcd.clear();
	lcd.print(TEXT_MODE_CHANGED);
	lcd.setCursor(0, 1);
	lcd.print(TextDeviceModeF[deviceMode]);
	if (deviceMode == devmodes::timedelay) {
		lcd.print(useDelayedStartAlarmAmperes);
	}
	delay(4000);
#endif
}

/*
 * Get Data value from two sensors DHT 2302 and calibrate values
 */

void getDataFromSensor() {
	bool ok = false;
	float delta;
#if USE_DHT
	ok = (DHT.read2302(DHT1_pin) == DHTLIB_OK);
	if (ok) {
		temp1 = DHT.temperature;
		humidity1 = DHT.humidity;
		ok = (DHT.read2302(DHT2_pin) == DHTLIB_OK);
		if (ok) {
			//correction
			temp2 = DHT.temperature + ctemp12;
			humidity2 = DHT.humidity;
			//correction special on zone 68% - 76%
			if (humidity2 > 68. && humidity2 < 76.) {
				delta = ((humidity2 - 68.) / 5.) * 8.;
				humidity2 = humidity2 + chumidity12 + delta;
			} else {
				humidity2 = humidity2 + chumidity12;
			}
		}else{
			statusSensorError=2;
		}
	}else{
		statusSensorError=1;
	}
#endif
	statusSensor = ok;
}

char convertNumber2Char(float mf) {
	char ret;
	int m = int(mf);
	if (m < 1 && mf >= 0.25) {
		if (mf >= (0.75)) {
			ret = char(CHAR_3_4);
		} else if (mf >= (0.5)) {
			ret = char(CHAR_1_2);
		} else if (mf >= (0.33)) {
			ret = char(CHAR_1_3);
		} else {
			ret = char(CHAR_1_4);
		}
	} else if (m >= 10 && m < 12) {
		ret = char(CHAR_10);
	} else if (m >= 12 && m < 15) {
		ret = char(CHAR_12);
	} else if (m == 15) {
		ret = char(CHAR_15);
	} else if (m > 15) {
		ret = char(CHAR_INF);
	} else {
		ret = char(CHAR_0 + (m > 9 ? 9 : m));
	}
	return ret;
}

char getMinutes(unsigned long msec) {
	float mf = ((millis() - msec) / 60000.);
	return convertNumber2Char(mf);
}

/**
 ** calculate difference of humidity values from two sensors DHT 2302
 ** and set alarm status
 */
void calculateHumidityAlarm() {
	if (!statusSensor){
		alarmHumidity = false;
		return;
	}
#if USE_DHT
	humidity_delta = abs(humidity2 - humidity1);
	int humidity_delta_level=DHT_Alarm_value;
	//correct when high humidity environment is
	if (humidity1>90){
		humidity_delta_level=humidity_delta_level/4;
	}else if (humidity1>80){
		humidity_delta_level=humidity_delta_level/2;
	}
	if (humidity_delta >= humidity_delta_level) {
		StopHumidityAlarm_timer = millis();
		if (!alarmHumidity) {
			if ((millis() - StartHumidityAlarm_timer) > HumidityAlarm_Start_Time) {
				alarmHumidity = true;
			}
		}
	} else {
		StartHumidityAlarm_timer = millis();
		if (alarmHumidity) {
			if ((millis() - StopHumidityAlarm_timer) > HumidityAlarm_Stop_Time) {
				alarmHumidity = false;
			}
		}
	}
#endif
}

/**
 *  Calculate AC Voltage value from Analog input from ACS712 module
 *
 */

float getVPP() {
	float result;
#if	USE_AMPERS
	int readValue;             //value read from the sensor
	int maxValue = 0;          // store max value here
	int minValue = 1024;       // store min value here  10 bit ADC

	unsigned long start_time = millis();
	while ((millis() - start_time) < 1000) //sample for 1 Sec
	{
		readValue = analogRead(AMPERSSENSOR_IO);
		// see if you have a new maxValue
		if (readValue > maxValue) {
			/*record the maximum sensor value*/
			maxValue = readValue;
		}
		if (readValue < minValue) {
			/*record the maximum sensor value*/
			minValue = readValue;
		}
	}

	// Subtract min from max
	// 5 Voltage max, 1024 = 2^10 bit ADC
	result = ((maxValue - minValue) * 5.0) / 1024.0;
#endif
	return result;
}

void checkAmpersVoltage() {
#if	USE_AMPERS
	Voltage = getVPP();
	VRMS = (Voltage / 2.0) * 0.707;
	AmpsRMS = (VRMS * 1000) / mVperAmp;

	if (AmpsRMS < AmpsLevel_LOW) {
		AmpersHighLevel_timer = millis();
		if ((millis() - AmpersLowLevel_timer) > AmperesAlarm_Stop_Time) {
			alarmAmperes = false;
		}
	} else {
		AmpersLowLevel_timer = millis();
		if ((millis() - AmpersHighLevel_timer) > AmperesAlarm_Start_Time) {
			alarmAmperes = true;
		}
	}
#endif
}

void printLCDTime(unsigned long &allSeconds) {
#if USE_LCD
	int runHours = allSeconds / 3600;
	int runMinutes = (allSeconds / 60) % 60;
	//int secsRemaining=allSeconds%3600;
	lcd.print(runHours);
	lcd.print(":");
	if (runMinutes <= 9) {
		lcd.print("0");
	}
	lcd.print(runMinutes);
#endif
}

/*
 * Print on LCD status of values and modes
 */

void printStatus() {

	// set the cursor to column 0, line 1
	// (note: line 1 is the second row, since counting begins with 0):
	//lcd.println("Type,\tstatus,\tHumidity (%),\tTemperature (C)\tTime (us)");
#if USE_LCD
	lcd.clear();
	lcd.noBlink();
	lcd.setCursor(0, 0);
	unsigned long ctime = millis();
	unsigned long csec = ctime / 1000;
	if (((csec) % 60) > 57) {
		//Display total spend time
		lcd.print("Total: ");
		printLCDTime(csec);
	} else {
		//display Temperature, Humidity and their delta information
		if (!statusSensor) {
			lcd.print("SENSOR ERR:");
			lcd.print(statusSensorError);
		} else {
			lcd.print(int(temp1));
			lcd.write(customLCDChars::DEGREE);
			lcd.print(int(humidity1));
			lcd.print("% ");
			int temp_delta = abs(temp2 - temp1);
			lcd.print(temp_delta);
			lcd.write(customLCDChars::DEGREE);
			lcd.print(humidity_delta);	//convertNumber2Char
			lcd.print("%");
		}
	}

	//display Amperes information
	lcd.setCursor(0, 1);
	lcd.print(convertNumber2Char(AmpsRMS));
	//lcd.print("A");
	lcd.write(customLCDChars::A);
	//print information about spend time of io relay state every even of ten seconds if relay is on
	int timetoshow=ctime / 10000 % 2;
	if (isRelayIO_ON() && timetoshow) {
		unsigned long rtime = (ctime - StartIORelay_timer);
		unsigned long rsec = rtime / 1000;
		lcd.setCursor(4, 1);
		printLCDTime(rsec);
	} else if(isMAXTimeForRelayIO && timetoshow){
		lcd.print(" SAFE T");
	}else{
		//display timer states for Amperes and Humidity sensors
		lcd.print(getMinutes(AmpersHighLevel_timer));
		lcd.print('/');
		lcd.print(getMinutes(AmpersLowLevel_timer));
		lcd.print(' ');
		lcd.print(getMinutes(StartHumidityAlarm_timer));
		lcd.print('/');
		lcd.print(getMinutes(StopHumidityAlarm_timer));
	}

	//display ALARM status
	lcd.setCursor(6, 0);
	if (alarmHumidity && alarmAmperes) {
		lcd.write(customLCDChars::AH);
	} else if (alarmHumidity) {
		lcd.write(customLCDChars::H);
		//lcd.print("H");
	} else if (alarmAmperes) {
		lcd.write(customLCDChars::A);
		//lcd.print("A");
	}
	//Display status of relay
	lcd.setCursor(LCD_COLUMNS - 2, 1);
	lcd.print(
			isRelayIO_ON() ?
					char(CHAR_IO_POWERED_ON) : char(CHAR_IO_POWERED_OFF));
	lcd.setCursor(LCD_COLUMNS - 1, 1);
	//Display current mode of device
	lcd.print(TextDeviceMode[deviceMode]);

	//Blink if delayed Alarms is.
	lcd.setCursor(LCD_COLUMNS - 2, 1);
	if (isDelayInAlarmAmperes) {
		lcd.blink();
	}else{
		lcd.noBlink();
	}

#endif
}

/**
 * Auto power himself after Max work time of device
 */
void checkMaxMainTime() {
	//Realy IO

	if (isRelayIO_ON()) {
		if ((millis() - StartIORelay_timer) > MAX_IORELAY_TIME) {
#if USE_LCD
			lcd.clear();
			lcd.print(TEXT_IO_POWER_TSAFE);
			lcd.setCursor(0, 1);
			lcd.print(TEXT_IO_POWER_OFF);
#endif
			relayoff();
			isMAXTimeForRelayIO=true;
			delay(2000);
		}
	}
	// Total Working
	if (millis() > MAX_WORKING_TIME) {
#if USE_LCD
		lcd.clear();
		lcd.print(TEXT_IO_POWER_OFF);
#endif
		relayoff();
		isMAXTimeForRelayIO=true;
		delay(2000);
#if USE_LCD
		lcd.setCursor(0, 1);
		lcd.print(TEXT_HS_POWER_OFF);
#endif
		delay(2000);
		relay_himself_off();
#if USE_LOWPOWER
		enterSleep();
#else
		delay(200000);
#endif
	}
}

void getKeyPressed() {
#ifdef _EasyButton_h
#if  EasyButton_Interupt
	button_poweroff.update();
	button_poweroff2.update();
	button_mode.update();
#else
	button_poweroff.read();
	button_poweroff2.read();
	button_mode.read();
#endif
#endif
}

/*
 * Init custom charsets for LCD
 * https://maxpromer.github.io/LCD-Character-Creator/
 */

void initLCDCustomChars() {
#if USE_LCD
	// A
	byte customChar0[8] = { 0x04, 0x0A, 0x0E, 0x0A, 0x00, 0x00, 0x00, 0x00 };
	lcd.createChar(customLCDChars::A, customChar0);
	// H
	byte customChar1[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x0E, 0x0A };
	lcd.createChar(customLCDChars::H, customChar1);
	// A/H
	byte customChar2[] = { 0x04, 0x0A, 0x0E, 0x0A, 0x00, 0x0A, 0x0E, 0x0A };
	lcd.createChar(customLCDChars::AH, customChar2);
	//Temperature small Degree
	byte customChar3[] = { 0x0C, 0x12, 0x12, 0x0C, 0x00, 0x00, 0x00, 0x00 };
	lcd.createChar(customLCDChars::DEGREE, customChar3);
#endif
}

/*
 * Init device setup
 */

void setup() {
	//Power ON for Relay powering himself after boot
	pinMode(RELAY_HS, OUTPUT);
	digitalWrite(RELAY_HS, RELAY_ON);

#ifdef _AVR_WDT_H_
	wdt_enable(WDTO_8S);
#endif

	// Disable the analog comparator by setting the ACD bit
	// (bit 7) of the ACSR register to one.
	//ACSR = B10000000;
	ACSR = 1 << ACD;

	//RELAY_IO is OFF by default on boot
	pinMode(RELAY_IO, OUTPUT);
	relayoff();
	//digitalWrite(RELAY_IO, RELAY_OFF);
	//pinMode(AMPERSSENSOR_IO, INPUT);

#if USE_BUTTON
	//Switches PULLUP
	pinMode(SWITCH_PowerOFF, INPUT_PULLUP);
	pinMode(SWITCH_Mode, INPUT_PULLUP);
#endif

#if USE_DEBUGSER
	Serial.begin(9600);
	Serial.println("Kitchen setup ...");
#endif

#ifdef _EasyButton_h
	button_poweroff.begin();
	button_poweroff.onPressed(onButton_poweroff_pressed);
	button_poweroff.onPressedFor(2000, onButton_poweroff_longpressed);
#if EasyButton_Interupt
	if (button_poweroff.supportsInterrupt()) {
		button_poweroff.enableInterrupt(onButton_poweroff_inerrupt);
		//Serial.println("EasyButton button_poweroff Interrupt example");
	}
#endif
	button_poweroff2.begin();
	button_poweroff2.onPressedFor(6000, onButton_poweroff_hs_longpressed);
	button_mode.begin();
	//button_mode.onSequence(2, 2000, onButton_mode_pressed);

	button_mode.onPressedFor(2000, onButton_mode_pressed);
#if EasyButton_Interupt
	if (button_mode.supportsInterrupt()) {
		button_mode.enableInterrupt(onButton_mode_inerrupt);
		//Serial.println("EasyButton button_mode Interrupt example");
	}
#endif
#endif
	//deviceMode = 1;
	//storeDeviceMode();
#ifdef EEPROM_h
	deviceMode = EEPROM[EEPROM_ADDRES_DeviceMODE];
	useDelayedStartAlarmAmperes = EEPROM[EEPROM_ADDRES_DeviceAlarmDelay];
#endif

#if USE_LCD
	// set up the LCD's number of columns and rows:
	lcd.begin(LCD_COLUMNS, LCD_ROWS);
#endif

	if (deviceMode == devmodes::manual) {
#if USE_LCD
		lcd.print(TEXT_WELCOME1);
		lcd.setCursor(0, 1);
		lcd.print(TEXT_IO_POWER_ON);
		delay(2000);
#endif
		relayon();
	} else {
#if USE_LCD
		lcd.clear();
		lcd.print("SAFE TIMER");
		lcd.setCursor(0, 1);
		lcd.print("MAIN: ");
		unsigned long time = MAX_WORKING_TIME/1000;
		printLCDTime(time);
		delay(700);
		lcd.clear();
		lcd.print("SAFE TIMER");
		lcd.setCursor(0, 1);
		lcd.print("FAN: ");
		time = MAX_IORELAY_TIME/1000;
		printLCDTime(time);
		delay(700);
		if (useDelayedStartAlarmAmperes){
			lcd.clear();
			lcd.print("DELAY TIMER");
			lcd.setCursor(0, 1);
			lcd.print("AMP.:");
			time = Delay_AlarmAmperes_Start_Time/1000;
			printLCDTime(time);
			delay(700);
		}
		lcd.clear();
		lcd.print(TEXT_WELCOME1);
		lcd.setCursor(0, 1);
		lcd.print(TEXT_WELCOME2);
		delay(2000);
#endif
	}
	initLCDCustomChars();
	RefreshDiplay_timer = DISPLAY_REFRESH_INTERVAL * 4;
}
/**
 * For DEBUG
 */

#if USE_DEBUGSER
void charsetPrint() {
#if USE_LCD
	int currentRow = 0;
	int currentCol = 0;
	for (int letter = 0; letter <= 255; letter++) {
		//  lcd.setCursor(currentCol, currentRow-1);
		lcd.setCursor(currentCol, currentRow);
		lcd.print(char(letter));
		Serial.println(letter);
		currentCol++;

		//  if (currentCol > 16){
		if (currentCol > 11) {
			currentCol = 0;
			if (currentRow < 1) {
				currentRow++;
			} else {
				delay(10000);
				lcd.clear();
				currentRow = 0;
				Serial.println("----");
			}
		}
	}
#endif
}
#endif

/**
 *	If Amperes Control used then my be need some time for heating device
 */

bool isAlarmAmperes() {
	if (!useDelayedStartAlarmAmperes) {
		return alarmAmperes;
	}
	if (alarmAmperes == false) {
		delayAlarmAmperes_timer = millis();
		return false;
	} else if ((millis() - delayAlarmAmperes_timer)
			> Delay_AlarmAmperes_Start_Time) {
		return alarmAmperes;
	}
	isDelayInAlarmAmperes = !isRelayIO_ON();
	return false;
}

/**
 * Checking that depended of device mode
 */
void checkModes() {
	isDelayInAlarmAmperes = false;
	switch (deviceMode) {
	case devmodes::timedelay:
	case devmodes::manual:
		break;
	case devmodes::general:
		if (isAlarmAmperes() || alarmHumidity) {
			relayon();
		} else {
			relayoff();
		}
		break;
	case devmodes::humidityandampers:
		if (isAlarmAmperes() && alarmHumidity) {
			relayon();
		} else {
			relayoff();
		}
		break;
	case devmodes::humidityonly:
		if (alarmHumidity) {
			relayon();
		} else {
			relayoff();
		}
		break;
	case devmodes::ampersonly:
		if (isAlarmAmperes()) {
			relayon();
		} else {
			relayoff();
		}
		break;
	}
}

/*
 * MAIN LOOP
 */

void loop() {
#ifdef _AVR_WDT_H_
	wdt_reset();
#endif

//	charsetPrint();
	if (millis() - RefreshDiplay_timer >= DISPLAY_REFRESH_INTERVAL) {
		RefreshDiplay_timer = millis();
		getKeyPressed();
		getDataFromSensor();
		calculateHumidityAlarm();
		checkAmpersVoltage();
		checkModes();
		printStatus();
		checkMaxMainTime();
	}
#if USE_DEBUGSER
	Serial.flush();
#endif
#if USE_LOWPOWER
	LowPower.idle(SLEEP_1S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF,
			USART0_OFF, TWI_OFF);
#endif
}
