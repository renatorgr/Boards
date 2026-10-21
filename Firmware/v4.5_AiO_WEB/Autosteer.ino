/* Autosteer.ino - steering: settings and data from the program (UDP), WAS, switch, motor.
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Based on the code of the AgOpenGPS AiO v4 Teensy firmware (AgOpenGPS project), as in
   Balázs Gunics' fork of the AgOpenGPS boards (github.com/gunicsba/agopengps_boards, firmware V4.1). Its
   original header is kept below.
   Small changes 2026 by Claude (Anthropic): whole UDP packet read (copied to the XIAO), [status] notes,
   the program told apart by its hello, IMU hello for the TM171 only, BNO08x code taken out;
   WAS voltage check, autosteer off at once when switched off, why it went off, sensor value
   for the web page (see zSafety.ino); wheel angle sent as 0 on the bench (WAS check off, WAS
   out of range), IMU hello only when an IMU was found (zImu.ino); the ADS1115 / autosteer state as one
   [boot] line (on the USB serial and the web page).
   AiO_WEB 4.5 (Claude (Anthropic) for renatorgr), from the AiO firmware 27.04.2025: its Keya (motor slip as the kickout, the motor
   position as the wheel angle when there is no WAS, faults turn autosteer off), autosteer cut-off beyond
   the WAS map (+-50 deg; only with the WAS voltage in range, zSafety.ino) or above 25 km/h, the WAS
   map, no steering below 0.2 km/h, PWM at 122 Hz and LOW_HIGH_DEGREES 1.5.
   AiO_WEB 4.5, 20 Oct 2026, ideas of the AgOpenGPS AiO 26 firmware: the minimum and maximum speed set on the
   web page, the current kickout only after the current is high for a while, the soft start
   (AutosteerPID.ino), autosteer off when the Keya heartbeat stops (KeyaCANBUS.ino).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
/*
   UDP Autosteer code for Teensy 4.1
   For AgOpenGPS
   01 Feb 2022
   Like all Arduino code - copied from somewhere else :)
   So don't claim it as your own
*/

/************************* User Settings *************************/

//How many degrees before decreasing Max PWM
// (AiO_WEB 4.5: 1.5 as in the AgOpenGPS AiO v4 firmware 27.04.2025; was 3.0: full PWM until 1.5 deg from the line)
#define LOW_HIGH_DEGREES 1.5

/*  PWM Frequency ->
	 490hz (default) = 0
	 122hz = 1
	 3921hz = 2
*/
// (AiO_WEB 4.5: 122 Hz as in the AiO firmware; was 490 Hz. Low enough to act as dither on a proportional
// hydraulic valve: the spool does not stick, small corrections get through)
#define PWM_Frequency 1

// AiO_WEB: DEFAULTS of the steering settings on the web page (zSettings.ino, Settings > Steering), from
// the AiO 26 firmware's ideas. They can be changed there without reprogramming.
#define STEER_MIN_KMH_X10   2     // below 0.2 km/h (the program's speed) the wheels are not turned (was fixed)
#define STEER_MAX_KMH       25    // above 25 km/h autosteer is cut (was fixed)
#define SOFT_START_X10MS    0     // soft start (x 10 ms): 0 = off, as before; the AiO 26 firmware uses 500 ms
#define CURRENT_KICK_X10MS  50    // current sensor kickout: the current must stay high 0.5 s (AiO 26: 1 s)

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 2400

//   ***********  Motor drive connections  **************888
//Connect ground only for cytron, Connect Ground and +5v for IBT2

//Dir1 for Cytron Dir, Both L and R enable for IBT2
#define DIR1_RL_ENABLE  4

//PWM1 for Cytron PWM, Left PWM for IBT2
#define PWM1_LPWM  2

//Not Connected for Cytron, Right PWM for IBT2
#define PWM2_RPWM  3

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 32
#define WORKSW_PIN 34
#define REMOTE_PIN 37

//Define sensor pin for current or pressure sensor
#define CURRENT_SENSOR_PIN A17
#define PRESSURE_SENSOR_PIN A10

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

#include <Wire.h>
#include <EEPROM.h>
#include "zADS1115.h"
ADS1115_lite adc(ADS1115_DEFAULT_ADDRESS);     // Use this for the 16-bit version ADS1115
#include <IPAddress.h>


#ifdef ARDUINO_TEENSY41
// ethernet
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#endif

#ifdef ARDUINO_TEENSY41
//uint8_t Ethernet::buffer[200]; // udp send and receive buffer
uint8_t autoSteerUdpData[UDP_TX_PACKET_MAX_SIZE];  // Buffer For Receiving UDP Data
#endif

//loop time variables in microseconds
const uint16_t LOOP_TIME = 25;  //40Hz
uint32_t autsteerLastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2; // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

//fromAutoSteerData FD 253 - ActualSteerAngle*100 -5,6, SwitchByte-7, pwmDisplay-8
uint8_t PGN_253[] = { 0x80,0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_253_Size = sizeof(PGN_253) - 1;

//fromAutoSteerData FD 250 - sensor values etc
uint8_t PGN_250[] = { 0x80,0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0,0,0,0, 0xCC };
int8_t PGN_250_Size = sizeof(PGN_250) - 1;
uint8_t aog2Count = 0;
float sensorReading;
float sensorSample;

// AiO_WEB, from the AiO firmware: WAS map (a correction curve for a WAS that is not linear). Input
// degrees -> output degrees; the same values = no change. Beyond the ends (+-50 deg) autosteer is cut.
#define WAS_MAP_N 21
float inputWAS[WAS_MAP_N]  = { -50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0 };  //Input WAS do not adjust
float outputWAS[WAS_MAP_N] = { -50.00, -45.0, -40.0, -35.0, -30.0, -25.0, -20.0, -15.0, -10.0, -5.0, 0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0 };

// The AiO firmware's multiMap (Rob Tillaart, github.com/RobTillaart/MultiMap), for floats
float wasMap(float value)
{
	if (value <= inputWAS[0]) return outputWAS[0];
	if (value >= inputWAS[WAS_MAP_N - 1]) return outputWAS[WAS_MAP_N - 1];
	uint8_t pos = 1;
	while (value > inputWAS[pos]) pos++;
	if (value == inputWAS[pos]) return outputWAS[pos];
	return (value - inputWAS[pos - 1]) * (outputWAS[pos] - outputWAS[pos - 1]) / (inputWAS[pos] - inputWAS[pos - 1]) + outputWAS[pos - 1];
}

elapsedMillis gpsSpeedUpdateTimer = 0;

//EEPROM
int16_t EEread = 0;

//Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;

//speed sent as *10
float gpsSpeed = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0; //the desired angle from AgOpen
int16_t steeringPosition = 0; //from steering sensor
float steerAngleError = 0; //setpoint - actual

//pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//Steer switch button  ******************************************
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0; // Steering Wheel Encoder
bool encEnable = false; //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;

//Variables for settings
struct Storage {
	uint8_t Kp = 40;              // proportional gain
	uint8_t lowPWM = 10;          // band of no action
	int16_t wasOffset = 0;
	uint8_t minPWM = 9;
	uint8_t highPWM = 60;         // max PWM value
	float steerSensorCounts = 30;
	float AckermanFix = 1;        // sent as percent
};  Storage steerSettings;      // 11 bytes (AW: 14 surely?

//Variables for settings - 0 is false
struct Setup {
	uint8_t InvertWAS = 0;
	uint8_t IsRelayActiveHigh = 0;    // if zero, active low (default)
	uint8_t MotorDriveDirection = 0;
	uint8_t SingleInputWAS = 1;
	uint8_t CytronDriver = 1;         // 0 if IBT, but we need to throw Keya into the mix. As we set bits from UDP, best to separate these and override "if keya"
	uint8_t SteerSwitch = 0;          // 1 if switch selected
	uint8_t SteerButton = 0;          // 1 if button selected
	uint8_t ShaftEncoder = 0;
	uint8_t PressureSensor = 0;
	uint8_t CurrentSensor = 0;
	uint8_t PulseCountMax = 5;
	uint8_t IsDanfoss = 0;
	uint8_t IsUseY_Axis = 0;     			//Set to 0 to use X Axis, 1 to use Y avis

}; Setup steerConfig;               // 9 bytes (AW: 14, surely?)

void steerConfigInit()
{
	if (!isKeya) {
		if (steerConfig.CytronDriver)
		{
			pinMode(PWM2_RPWM, OUTPUT);
		}
	}
}

void steerSettingsInit()
{
	// for PWM High to Low interpolator
	highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}

void autosteerSetup()
{
	//PWM rate settings. Set them both the same!!!!
	/*  PWM Frequency ->
		 490hz (default) = 0
		 122hz = 1
		 3921hz = 2
	*/
	if (PWM_Frequency == 0)
	{
		analogWriteFrequency(PWM1_LPWM, 490);
		analogWriteFrequency(PWM2_RPWM, 490);
	}
	else if (PWM_Frequency == 1)
	{
		analogWriteFrequency(PWM1_LPWM, 122);
		analogWriteFrequency(PWM2_RPWM, 122);
	}
	else if (PWM_Frequency == 2)
	{
		analogWriteFrequency(PWM1_LPWM, 3921);
		analogWriteFrequency(PWM2_RPWM, 3921);
	}

	//keep pulled high and drag low to activate, noise free safe
	pinMode(WORKSW_PIN, INPUT_PULLUP);
	pinMode(STEERSW_PIN, INPUT_PULLUP);
	pinMode(REMOTE_PIN, INPUT_PULLUP);
	pinMode(DIR1_RL_ENABLE, OUTPUT);

	// Disable digital inputs for analog input pins
	pinMode(CURRENT_SENSOR_PIN, INPUT_DISABLE);
	pinMode(PRESSURE_SENSOR_PIN, INPUT_DISABLE);

	//set up communication
	Wire1.end();
	Wire1.begin();

	// Check ADC 
	if (!adc.testConnection()) Autosteer_running = false;   // no ADS1115: no WAS reading (the [boot] line below)

	//50Khz I2C
	//TWBR = 144;   //Is this needed?

	EEPROM.get(0, EEread);              // read identifier

	if (EEread != EEP_Ident)            // check on first start and write EEPROM
	{
		EEPROM.put(0, EEP_Ident);
		EEPROM.put(10, steerSettings);
		EEPROM.put(40, steerConfig);
		EEPROM.put(60, networkAddress);
	}
	else
	{
		EEPROM.get(10, steerSettings);     // read the Settings
		EEPROM.get(40, steerConfig);
		EEPROM.get(60, networkAddress);
	}

	steerSettingsInit();
	steerConfigInit();

	if (Autosteer_running)
	{
		StatusLog.line("[boot] Autosteer: ADS1115 (WAS) found, waiting for the program");
		// Autosteer Led goes Red if ADS1115 is found
		digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
		digitalWrite(AUTOSTEER_STANDBY_LED, 1);
	}
	else
	{
		Autosteer_running = false;  //Turn off auto steer if no ethernet (Maybe running T4.0)
		StatusLog.line("[boot] Autosteer OFF: ADS1115 (WAS) not found on I2C (Wire1) - GPS only, module IP x.x.x.120");
		return;
	}

	adc.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); //128 samples per second
	adc.setGain(ADS1115_REG_CONFIG_PGA_6_144V);

}// End of Setup

void autosteerLoop()
{
#ifdef ARDUINO_TEENSY41
	ReceiveUdp();
#endif
	//Serial.println("AutoSteer loop");

	// Loop triggers every 100 msec and sends back gyro heading, and roll, steer angle etc
	currentTime = systick_millis_count;

	if (currentTime - autsteerLastTime >= LOOP_TIME)
	{
		autsteerLastTime = currentTime;

		//reset debounce
		encEnable = true;

		//If connection lost to AgOpenGPS, the watchdog will count up and turn off steering
		if (watchdogTimer++ > 250) watchdogTimer = WATCHDOG_FORCE_VALUE;

		//read all the switches
		workSwitch = digitalRead(WORKSW_PIN);  // read work switch

		if (steerConfig.SteerSwitch == 1)         //steer switch on - off
		{
			steerSwitch = digitalRead(STEERSW_PIN); //read auto steer enable switch open = 0n closed = Off
		}
		else if (steerConfig.SteerButton == 1)    //steer Button momentary
		{
			reading = digitalRead(STEERSW_PIN);
			if (reading == LOW && previous == HIGH)
			{
				if (currentState == 1)
				{
					currentState = 0;
					steerSwitch = 0;
				}
				else
				{
					currentState = 1;
					steerSwitch = 1;
				}
			}
			previous = reading;
		}
		else                                      // No steer switch and no steer button
		{
			// So set the correct value. When guidanceStatus = 1,
			// it should be on because the button is pressed in the GUI
			// But the guidancestatus should have set it off first
			if (guidanceStatusChanged && guidanceStatus == 1 && steerSwitch == 1 && previous == 0)
			{
				steerSwitch = 0;
				previous = 1;
			}

			// This will set steerswitch off and make the above check wait until the guidanceStatus has gone to 0
			if (guidanceStatusChanged && guidanceStatus == 0 && steerSwitch == 0 && previous == 1)
			{
				steerSwitch = 1;
				previous = 0;
			}
		}

		if (steerConfig.ShaftEncoder && pulseCount >= steerConfig.PulseCountMax)
		{
			steerSwitch = 1; // reset values like it turned off
			currentState = 1;
			previous = 0;
			steerKick(SO_ENCODER, pulseCount);           // why it went off (zSafety.ino)
		}

		// Pressure sensor?
		if (steerConfig.PressureSensor)
		{
			sensorSample = (float)analogRead(PRESSURE_SENSOR_PIN);
			sensorSample *= 0.25;
			sensorReading = sensorReading * 0.6 + sensorSample * 0.4;
			if (sensorReading >= steerConfig.PulseCountMax)
			{
				steerSwitch = 1; // reset values like it turned off
				currentState = 1;
				previous = 0;
				steerKick(SO_PRESSURE, (int)sensorReading);
			}
		}

		// Current sensor?
		if (steerConfig.CurrentSensor)
		{
			if (isKeya) {
				// AiO_WEB: as in the AiO firmware, the Keya's slip (it turns slower than told: the driver
				// holds the wheel) is the kickout sensor (KeyaCANBUS.ino)
				sensorReading = keyaSlip;
				if (sensorReading >= steerConfig.PulseCountMax) {
					steerSwitch = 1; // reset values like it turned off
					currentState = 1;
					previous = 0;
					steerKick(SO_KEYA, (int)sensorReading);
				}
			}
			else {
				sensorSample = (float)analogRead(CURRENT_SENSOR_PIN);
				sensorSample = (abs(775 - sensorSample)) * 0.5;
				sensorReading = sensorReading * 0.7 + sensorSample * 0.3;
				sensorReading = min(sensorReading, 255);

				// AiO_WEB, as in the AiO 26 firmware: only a current that stays high for the time set on the web page
				// (Settings > Steering) is a kickout; a short peak (the valve or motor starting) is not
				static uint32_t currentHighMs = 0;   // when it went above the limit (0 = below)
				if (sensorReading >= steerConfig.PulseCountMax)
				{
					if (currentHighMs == 0) currentHighMs = currentTime | 1;
					if (currentTime - currentHighMs >= (uint32_t)cfg.currentKickX10ms * 10)
					{
						steerSwitch = 1; // reset values like it turned off
						currentState = 1;
						previous = 0;
						steerKick(SO_CURRENT, (int)sensorReading);
					}
				}
				else currentHighMs = 0;

			}
		}

		// the value for the web page (zSafety.ino): the kickout sensor, or else the pressure input
		if (steerConfig.PressureSensor || steerConfig.CurrentSensor) sensorViewNote(sensorReading);
		else sensorViewNote(pressureFiltered((float)analogRead(PRESSURE_SENSOR_PIN) * 0.25f));

		remoteSwitch = digitalRead(REMOTE_PIN); //read auto steer enable switch open = 0n closed = Off

		/*
		  #if Relay_Type == 1
			SetRelays();       //turn on off section relays
		  #elif Relay_Type == 2
			SetuTurnRelays();  //turn on off uTurn relays
		  #endif
		*/

		// AiO_WEB: Keya with the WAS on "differential" in AgOpenGPS = no WAS: the motor position is the
		// wheel angle (below), as in the AiO firmware; the ADS1115 is not read and the WAS check is off
		const bool keyaWas = isKeya && !steerConfig.SingleInputWAS;

		//get steering position
		if (keyaWas) { }
		else if (steerConfig.SingleInputWAS)   //Single Input ADS
		{
			adc.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);
			steeringPosition = adc.getConversion();
			adc.triggerConversion();//ADS1115 Single Mode

			steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
			helloSteerPosition = steeringPosition - 6800;
		}
		else    //ADS1115 Differential Mode
		{
			adc.setMux(ADS1115_REG_CONFIG_MUX_DIFF_0_1);
			steeringPosition = adc.getConversion();
			adc.triggerConversion();

			steeringPosition = (steeringPosition >> 1); //bit shift by 2  0 to 13610 is 0 to 5v
			helloSteerPosition = steeringPosition - 6800;
		}

		// WAS voltage check (zSafety.ino). Out of range (wire, plug, sensor) = autosteer
		// blocked and turned off like a kickout, so the program sees the switch off.
		if (!keyaWas && wasCheck(steeringPosition))
		{
			steerSwitch = 1;
			currentState = 1;
			previous = 0;
		}

		// AiO_WEB: Keya heartbeat lost (CAN cable, plug, motor power) = autosteer off and blocked, like the
		// WAS check (KeyaCANBUS.ino)
		if (keyaCheck())
		{
			steerSwitch = 1;
			currentState = 1;
			previous = 0;
		}

		switchByte = 0;                    // (after the WAS check, so it shows its block too)
		switchByte |= (remoteSwitch << 2); //put remote in bit 2
		switchByte |= (steerSwitch << 1);   //put steerswitch status in bit 1 position
		switchByte |= workSwitch;

		// switched off (switch, button, kickout, WAS) = autosteer off now, not at the next
		// steering packet from the program (up to 100 ms later at 10 Hz)
		if (steerSwitch == 1) watchdogTimer = WATCHDOG_FORCE_VALUE;

		//DETERMINE ACTUAL STEERING POSITION

		//convert position to steer angle. 32 counts per degree of steer pot position in my case
		//  ***** make sure that negative steer angle makes a left turn and positive value is a right turn *****
		if (keyaWas)
		{
			// AiO firmware: the Keya position / counts per degree, plus an offset learned slowly from the
			// wheel angle the GPS heading gives (zHandlers.ino) while moving above 1.2 km/h
			static float keya_GPS_offset = 0;
			steeringPosition = keyaSteeringPosition + steerSettings.wasOffset;
			helloSteerPosition = steeringPosition;
			const float keyaSteerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
			if (gpsSpeed > 1.2 && !steerConfig.InvertWAS && wheelAngleGPSOk)
			{
				const float wasDiff = (keyaSteerAngleActual + keya_GPS_offset) - wheelAngleGPS;
				keya_GPS_offset -= (wasDiff * (0.001));
			}
			steerAngleActual = keyaSteerAngleActual + keya_GPS_offset;
		}
		else if (steerConfig.InvertWAS)
		{
			steeringPosition = (steeringPosition - 6805 - steerSettings.wasOffset);   // 1/2 of full scale
			steerAngleActual = (float)(steeringPosition) / -steerSettings.steerSensorCounts;
		}
		else
		{
			steeringPosition = (steeringPosition - 6805 + steerSettings.wasOffset);   // 1/2 of full scale
			steerAngleActual = (float)(steeringPosition) / steerSettings.steerSensorCounts;
		}

		//Ackerman fix
		if (steerAngleActual < 0) steerAngleActual = (steerAngleActual * steerSettings.AckermanFix);

		// bench (WAS check off) with no WAS: the reading is about 0 V, which shows as the wheels at
		// full lock to one side on the program's screen. Send 0 degrees instead (zSafety.ino).
		if (!keyaWas && wasBenchZero())
		{
			steerAngleActual = 0;
			helloSteerPosition = 0;
		}

		// AiO_WEB, as in the AiO firmware: a wheel angle beyond the WAS map (+-50 deg: a wrong calibration, a
		// slipped sensor) or the program's speed above the maximum (web page, default 25 km/h) cuts autosteer.
		// With the WAS voltage out of range the WAS check above decides (zSafety.ino), not this.
		const bool angleOut = (steerAngleActual < inputWAS[0] || steerAngleActual > inputWAS[WAS_MAP_N - 1]) && (keyaWas || wasVoltsOk());
		const bool tooFast  = gpsSpeed > cfg.steerMaxKmh;
		if (angleOut || tooFast)
		{
			if (steerSwitch == 0) steerKick(tooFast ? SO_FAST : SO_ANGLE, tooFast ? (int)gpsSpeed : (int)steerAngleActual);
			steerSwitch = 1; // reset values like it turned off
			currentState = 1;
			previous = 0;
			watchdogTimer = WATCHDOG_FORCE_VALUE;
		}

		// AiO firmware: WAS map (inputWAS -> outputWAS, the same values = no change)
		steerAngleActual = wasMap(steerAngleActual);

		const bool steerOn = watchdogTimer < WATCHDOG_THRESHOLD;
		if (steerOn)
		{
			//Enable H Bridge for IBT2, hyd aux, etc for cytron. Don't care about this for Keya
			if (true || !isKeya) {
				if (steerConfig.CytronDriver)
				{
					if (steerConfig.IsRelayActiveHigh)
					{
						digitalWrite(PWM2_RPWM, 0);
					}
					else
					{
						digitalWrite(PWM2_RPWM, 1);
					}
				}
				else digitalWrite(DIR1_RL_ENABLE, 1);
			}
			steerAngleError = steerAngleActual - steerAngleSetPoint;   //calculate the steering error
			//if (abs(steerAngleError)< steerSettings.lowPWM) steerAngleError = 0;

			// AiO firmware: don't turn the wheels below the minimum speed (the program's speed; web page,
			// default 0.2 km/h)
			if (gpsSpeed < cfg.steerMinKmhX10 * 0.1f) steerAngleError = 0;

			keyaIntendToSteer = true;
			calcSteeringPID();  //do the pid
			motorDrive();       //out to motors the pwm value
			// Autosteer Led goes GREEN if autosteering

			digitalWrite(AUTOSTEER_ACTIVE_LED, 1);
			digitalWrite(AUTOSTEER_STANDBY_LED, 0);
		}
		else
		{
			//we've lost the comm to AgOpenGPS, or just stop request
			//Disable H Bridge for IBT2, hyd aux, etc for cytron
			// Don't care about this for Keya
			if (true || !isKeya) {
				if (steerConfig.CytronDriver)
				{
					if (steerConfig.IsRelayActiveHigh)
					{
						digitalWrite(PWM2_RPWM, 1);
					}
					else
					{
						digitalWrite(PWM2_RPWM, 0);
					}
				}
				else digitalWrite(DIR1_RL_ENABLE, 0); //IBT2
			}

			pwmDrive = 0; //turn off steering motor
			steerRampReset();           // the next engage starts with the soft start (AutosteerPID.ino)
			keyaIntendToSteer = false;  // Keya: motorDrive() sends it the disable command
			motorDrive(); //out to motors the pwm value
			pulseCount = 0;
			// Autosteer Led goes back to RED when autosteering is stopped
			digitalWrite(AUTOSTEER_STANDBY_LED, 1);
			digitalWrite(AUTOSTEER_ACTIVE_LED, 0);
		}
		steerOnNote(steerOn);              // on -> off: why (zSafety.ino)
	} //end of timed loop

	//This runs continuously, outside of the timed loop, keeps checking for new udpData, turn sense
	//delay(1);

	// Speed pulse
	if (gpsSpeedUpdateTimer < 1000)
	{
		if (speedPulseUpdateTimer > 200) // 100 (10hz) seems to cause tone lock ups occasionally
		{
			speedPulseUpdateTimer = 0;

			//130 pp meter, 3.6 kmh = 1 m/sec = 130hz or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
			//gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
			float speedPulse = gpsSpeed * 36.1111;

			//Serial.print(gpsSpeed); Serial.print(" -> "); Serial.println(speedPulse);

			if (gpsSpeed > 0.11) { // 0.10 wasn't high enough
				tone(velocityPWM_Pin, uint16_t(speedPulse));
			}
			else {
				noTone(velocityPWM_Pin);
			}
		}
	}
	else  // if gpsSpeedUpdateTimer hasn't update for 1000 ms, turn off speed pulse
	{
		noTone(velocityPWM_Pin);
	}

	if (encEnable)
	{
		thisEnc = digitalRead(REMOTE_PIN);
		if (thisEnc != lastEnc)
		{
			lastEnc = thisEnc;
			if (lastEnc) EncoderFunc();
		}
	}

} // end of main loop

int currentRoll = 0;
int rollLeft = 0;
int steerLeft = 0;

#ifdef ARDUINO_TEENSY41
// UDP Receive
void ReceiveUdp()
{
	// When ethernet is not running, return directly. parsePacket() will block when we don't
	if (!Ethernet_running)
	{
		return;
	}

	uint16_t len = Eth_udpAutoSteer.parsePacket();
	if (len > 0) statusNoteAogPacket(Eth_udpAutoSteer.remoteIP());   // [status] messages (zStatus.ino)

	// if (len > 0)
	// {
	//  Serial.print("ReceiveUdp: ");
	//  Serial.println(len);
	// }

	// Check for len > 4, because we check byte 0, 1, 3 and 3
	if (len > 4)
	{
		// read the whole packet, so the XIAO ESP32C3 gets all of it (zXiao.ino). The steering code
		// below still uses only the first UDP_TX_PACKET_MAX_SIZE (24) bytes, as before.
		static uint8_t udpPacket[512];
		const int got = Eth_udpAutoSteer.read(udpPacket, sizeof(udpPacket));
		if (got <= 0) return;
		memcpy(autoSteerUdpData, udpPacket, got < (int)UDP_TX_PACKET_MAX_SIZE ? got : (int)UDP_TX_PACKET_MAX_SIZE);
		xiaoForward(udpPacket, (uint16_t)got, len);

		if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) //Data
		{
			if (autoSteerUdpData[3] == 0xFE && Autosteer_running)  //254
			{
				steerNote254();                // last steering data (zSafety.ino)
				gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
				gpsSpeedUpdateTimer = 0;

				prevGuidanceStatus = guidanceStatus;

				guidanceStatus = autoSteerUdpData[7];
				guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

				//Bit 8,9    set point steer angle * 100 is sent
				steerAngleSetPoint = ((float)(autoSteerUdpData[8] | ((int8_t)autoSteerUdpData[9]) << 8)) * 0.01; //high low bytes

				//Serial.print("steerAngleSetPoint: ");
				//Serial.println(steerAngleSetPoint);

				//Serial.println(gpsSpeed);

				if ((bitRead(guidanceStatus, 0) == 0) || (gpsSpeed < 0.1) || (steerSwitch == 1))
				{
					watchdogTimer = WATCHDOG_FORCE_VALUE; //turn off steering motor
				}
				else          //valid conditions to turn on autosteer
				{
					watchdogTimer = 0;  //reset watchdog
				}

				//Bit 10 Tram
				tram = autoSteerUdpData[10];

				//Bit 11
				relay = autoSteerUdpData[11];

				//Bit 12
				relayHi = autoSteerUdpData[12];

				//----------------------------------------------------------------------------
				//Serial Send to agopenGPS

				int16_t sa = (int16_t)(steerAngleActual * 100);

				PGN_253[5] = (uint8_t)sa;
				PGN_253[6] = sa >> 8;

				// heading
				PGN_253[7] = (uint8_t)9999;
				PGN_253[8] = 9999 >> 8;

				// roll
				PGN_253[9] = (uint8_t)8888;
				PGN_253[10] = 8888 >> 8;

				PGN_253[11] = switchByte;
				PGN_253[12] = (uint8_t)pwmDisplay;

				//checksum
				int16_t CK_A = 0;
				for (uint8_t i = 2; i < PGN_253_Size; i++)
					CK_A = (CK_A + PGN_253[i]);

				PGN_253[PGN_253_Size] = CK_A;

				//off to AOG
				SendUdp(PGN_253, sizeof(PGN_253), Eth_ipDestination, portDestination);

				//Steer Data 2 -------------------------------------------------
				if (steerConfig.PressureSensor || steerConfig.CurrentSensor)
				{
					if (aog2Count++ > 2)
					{
						//Send fromAutosteer2
						PGN_250[5] = (byte)sensorReading;

						//add the checksum for AOG2
						CK_A = 0;

						for (uint8_t i = 2; i < PGN_250_Size; i++)
						{
							CK_A = (CK_A + PGN_250[i]);
						}

						PGN_250[PGN_250_Size] = CK_A;

						//off to AOG
						SendUdp(PGN_250, sizeof(PGN_250), Eth_ipDestination, portDestination);
						aog2Count = 0;
					}
				}

				//Serial.println(steerAngleActual);
				//--------------------------------------------------------------------------
			}

			//steer settings
			else if (autoSteerUdpData[3] == 0xFC && Autosteer_running)  //252
			{
				//PID values
				steerSettings.Kp = ((float)autoSteerUdpData[5]);   // read Kp from AgOpenGPS

				steerSettings.highPWM = autoSteerUdpData[6]; // read high pwm

				steerSettings.lowPWM = (float)autoSteerUdpData[7];   // read lowPWM from AgOpenGPS

				steerSettings.minPWM = autoSteerUdpData[8]; //read the minimum amount of PWM for instant on

				float temp = (float)steerSettings.minPWM * 1.2;
				steerSettings.lowPWM = (byte)temp;

				steerSettings.steerSensorCounts = autoSteerUdpData[9]; //sent as setting displayed in AOG

				steerSettings.wasOffset = (autoSteerUdpData[10]);  //read was zero offset Lo

				steerSettings.wasOffset |= (autoSteerUdpData[11] << 8);  //read was zero offset Hi

				steerSettings.AckermanFix = (float)autoSteerUdpData[12] * 0.01;

				//crc
				//autoSteerUdpData[13];

				//store in EEPROM
				EEPROM.put(10, steerSettings);

				// Re-Init steer settings
				steerSettingsInit();
				steerNote252();                // shown on the web page (zSafety.ino)
			}

			else if (autoSteerUdpData[3] == 0xFB)  //251 FB - SteerConfig
			{
				uint8_t sett = autoSteerUdpData[5]; //setting0

				if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
				if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
				if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
				if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
				if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
				if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
				if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
				if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

				steerConfig.PulseCountMax = autoSteerUdpData[6];

				//was speed
				//autoSteerUdpData[7];

				sett = autoSteerUdpData[8]; //setting1 - Danfoss valve etc

				if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
				if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
				if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
				if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

				//crc
				//autoSteerUdpData[13];

				EEPROM.put(40, steerConfig);

				// Re-Init
				steerConfigInit();
				steerNote251();                // shown on the web page (zSafety.ino)

			}//end FB
			else if (autoSteerUdpData[3] == 0xF7)  //247 vehicle geometry (AgOpenWeb), the whole packet
			{
				vehicleGeometry(udpPacket, got);   // zHandlers.ino
			}
			else if (autoSteerUdpData[3] == 200) // Hello from AgIO
			{
				// AgOpenWeb or AgOpenGPS? (automatic output mode, zStatus.ino)
				statusNoteHello(udpPacket, (uint16_t)got, Eth_udpAutoSteer.remoteIP());
				safetyNoteHello();             // restart / WAS check off on the program's screen (zSafety.ino)

				if (Autosteer_running)
				{
					int16_t sa = (int16_t)(steerAngleActual * 100);

					helloFromAutoSteer[5] = (uint8_t)sa;
					helloFromAutoSteer[6] = sa >> 8;

					helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
					helloFromAutoSteer[8] = helloSteerPosition >> 8;
					helloFromAutoSteer[9] = switchByte;

					SendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), Eth_ipDestination, portDestination);
				}
				// the IMU hello only when there is an IMU (TM171 or BNO085, zImu.ino), as in the AiO firmware
				if (imuType != IMU_NONE) SendUdp(helloFromIMU, sizeof(helloFromIMU), Eth_ipDestination, portDestination);
			}

			else if (autoSteerUdpData[3] == 201)
			{
				//make really sure this is the subnet pgn
				if (autoSteerUdpData[4] == 5 && autoSteerUdpData[5] == 201 && autoSteerUdpData[6] == 201)
				{
					networkAddress.ipOne = autoSteerUdpData[7];
					networkAddress.ipTwo = autoSteerUdpData[8];
					networkAddress.ipThree = autoSteerUdpData[9];

					//save in EEPROM and restart
					EEPROM.put(60, networkAddress);
					SCB_AIRCR = 0x05FA0004; //Teensy Reset
				}
			}//end 201

			//whoami
			else if (autoSteerUdpData[3] == 202)
			{
				//make really sure this is the reply pgn
				if (autoSteerUdpData[4] == 3 && autoSteerUdpData[5] == 202 && autoSteerUdpData[6] == 202)
				{
					IPAddress rem_ip = Eth_udpAutoSteer.remoteIP();

					//hello from AgIO
					uint8_t scanReply[] = { 128, 129, Eth_myip[3], 203, 7,
						Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3],
						rem_ip[0],rem_ip[1],rem_ip[2], 23 };

					//checksum
					int16_t CK_A = 0;
					for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
					{
						CK_A = (CK_A + scanReply[i]);
					}
					scanReply[sizeof(scanReply) - 1] = CK_A;

					static uint8_t ipDest[] = { 255,255,255,255 };
					uint16_t portDest = 9999; //AOG port that listens

					//off to AOG
					SendUdp(scanReply, sizeof(scanReply), ipDest, portDest);
				}
			}
		} //end if 80 81 7F
	}
}
#endif

#ifdef ARDUINO_TEENSY41
void SendUdp(uint8_t* data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
	Eth_udpAutoSteer.beginPacket(dip, dport);
	Eth_udpAutoSteer.write(data, datalen);
	Eth_udpAutoSteer.endPacket();
}
#endif

//ISR Steering Wheel Encoder
void EncoderFunc()
{
	if (encEnable)
	{
		pulseCount++;
		encEnable = false;
	}
}
