/* AutosteerPID.ino - steering PID and output (Cytron, IBT2, Danfoss, Keya).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   From the AgOpenGPS AiO v4 Teensy firmware (AgOpenGPS project), as in
   Balázs Gunics' fork of the AgOpenGPS boards (github.com/gunicsba/agopengps_boards, firmware V4.1).
   Change by Claude (Anthropic) 2026 for renatorgr (v4.5_AiO_WEB): the Keya gets its speed and
   enable / disable as in the AiO firmware 27.04.2025 (SteerKeya, KeyaCANBUS.ino).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
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
