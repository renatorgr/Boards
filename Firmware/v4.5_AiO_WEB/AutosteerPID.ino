/* AutosteerPID.ino - steering PID and output (Cytron, IBT2, Danfoss, Keya).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   From the AgOpenGPS AiO v4 Teensy firmware (AgOpenGPS project), as in
   Balázs Gunics' fork of the AgOpenGPS boards (github.com/gunicsba/agopengps_boards, firmware V4.1).
   Change by Claude (Anthropic) 2026 for renatorgr (v4.5_AiO_WEB): the Keya gets its speed and
   enable / disable as in the AiO firmware 27.04.2025 (SteerKeya, KeyaCANBUS.ino).
   AiO_WEB 4.5, 20 Oct 2026: the soft start, after the one of the AgOpenGPS AiO 26 firmware
   (AutosteerProcessor.cpp, GPL v3): steerRamp().
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// Soft start (web page, Settings > Steering; off by default). When autosteer engages, the output
// rises from 0 to the PID value over the time set; on a hard step (from about minPWM to about
// highPWM) or a reversal it rises again over half that time. Linear, as the AiO 26 default.
static uint8_t  rampMode = 0;      // 0 = steering off, 1 = soft start, 2 = soft step / reversal, 3 = normal
static uint32_t rampMs   = 0;      // when the ramp began
static int16_t  rampLast = 0;      // the PID value of the last loop (before the ramp)

// Autosteer.ino, steering off: the next engage starts with the soft start
void steerRampReset()
{
    rampMode = 0;
    rampLast = 0;
}

static int16_t steerRamp(int16_t pwm)
{
    const uint32_t startMs = (uint32_t)cfg.softStartX10ms * 10;
    if (startMs == 0) { rampMode = 3; rampLast = pwm; return pwm; }
    const uint32_t now = millis();
    const int16_t hi = steerSettings.highPWM, lo = steerSettings.minPWM;
    if (rampMode == 0) { rampMode = 1; rampMs = now; }
    else if (rampMode == 3)
    {
        const int16_t th = (hi - lo) / 3;
        const bool hardStep = abs(pwm) > hi - th && abs(rampLast) < lo + th;
        const bool reversal = ((rampLast > 0 && pwm < 0) || (rampLast < 0 && pwm > 0)) && abs(pwm) > lo + 20;
        if (hardStep || reversal) { rampMode = 2; rampMs = now; }
    }
    rampLast = pwm;
    if (rampMode == 3) return pwm;
    const uint32_t dur = rampMode == 1 ? startMs : startMs / 2;
    const uint32_t el  = now - rampMs;
    if (el >= dur) { rampMode = 3; return pwm; }
    int16_t out = (int16_t)((float)pwm * (float)el / (float)dur);
    // keep at least minPWM, so the valve / motor moves at all during the ramp
    const int16_t floorPwm = abs(pwm) < lo ? abs(pwm) : lo;
    if (pwm != 0 && abs(out) < floorPwm) out = pwm > 0 ? floorPwm : -floorPwm;
    return out;
}

void calcSteeringPID(void)
{
    //Proportional only
    pValue = steerSettings.Kp * steerAngleError;
    pwmDrive = (int16_t)pValue;

    errorAbs = abs(steerAngleError);
    int16_t newMax = 0;

    if (errorAbs < LOW_HIGH_DEGREES)
    {
        newMax = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
    }
    else newMax = steerSettings.highPWM;

    //add min throttle factor so no delay from motor resistance.
    if (pwmDrive < 0) pwmDrive -= steerSettings.minPWM;
    else if (pwmDrive > 0) pwmDrive += steerSettings.minPWM;

    //Serial.print(newMax); //The actual steering angle in degrees
    //Serial.print(",");

    //limit the pwm drive
    if (pwmDrive > newMax) pwmDrive = newMax;
    if (pwmDrive < -newMax) pwmDrive = -newMax;

    pwmDrive = steerRamp(pwmDrive);    // AiO_WEB: soft start (off by default)

    if (steerConfig.MotorDriveDirection) pwmDrive *= -1;

    if (steerConfig.IsDanfoss)
    {
        // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
        // Danfoss: PWM 50% On = Center Position
        // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
        pwmDrive = (constrain(pwmDrive, -250, 250));

        // Calculations below make sure pwmDrive values are between 65 and 190
        // This means they are always positive, so in motorDrive, no need to check for
        // steerConfig.isDanfoss anymore
        pwmDrive = pwmDrive >> 2; // Divide by 4
        pwmDrive += 128;          // add Center Pos.

        // pwmDrive now lies in the range [65 ... 190], which would be great for an ideal opamp
        // However the TLC081IP is not ideal. Approximating from fig 4, 5 TI datasheet, @Vdd=12v, T=@40Celcius, 0 current
        // Voh=11.08 volts, Vol=0.185v
        // (11.08/12)*255=235.45
        // (0.185/12)*255=3.93
        // output now lies in the range [67 ... 205], the center position is now 136
        //pwmDrive = (map(pwmDrive, 4, 235, 0, 255));
    }
}

//#########################################################################################

void motorDrive(void)
{
	// Used with Cytron MD30C Driver
	// Steering Motor
	// Dir + PWM Signal
	if (isKeya) {
		// AiO_WEB: the AiO firmware's Keya (KeyaCANBUS.ino): speed, and enable / disable
		SteerKeya(pwmDrive, keyaIntendToSteer);
		pwmDisplay = pwmDrive < 0 ? -pwmDrive : pwmDrive;
	}
	else if (steerConfig.CytronDriver)
	{
		// Cytron MD30C Driver Dir + PWM Signal
		if (pwmDrive >= 0)
		{
			bitSet(PORTD, 4);  //set the correct direction
		}
		else
		{
			bitClear(PORTD, 4);
			pwmDrive = -1 * pwmDrive;
		}

		//write out the 0 to 255 value
		analogWrite(PWM1_LPWM, pwmDrive);
		pwmDisplay = pwmDrive;
	}
	else
	{
		// IBT 2 Driver Dir1 connected to BOTH enables
		// PWM Left + PWM Right Signal

		if (pwmDrive > 0)
		{
			analogWrite(PWM2_RPWM, 0);//Turn off before other one on
			analogWrite(PWM1_LPWM, pwmDrive);
		}
		else
		{
			pwmDrive = -1 * pwmDrive;
			analogWrite(PWM1_LPWM, 0);//Turn off before other one on
			analogWrite(PWM2_RPWM, pwmDrive);
		}

		pwmDisplay = pwmDrive;
	}
}
