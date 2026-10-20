/* KeyaCANBUS.ino - Keya motor on the CAN bus (Teensy CAN3).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   From the AgOpenGPS AiO v4 Teensy firmware (zCANBUS.ino of
   AIO_v4_Firmware_Keya_CANBUS_HPR_Dual_27.04.2025, AgOpenGPS project, GPL v3): the commands and
   their responses, the heartbeat (motor position, speed, current, faults), the motor slip used as the
   kickout sensor, SteerKeya() and keyaCommand().
   Changes by Claude (Anthropic) 2026 for renatorgr (v4.5_AiO_WEB): the names the rest of this
   firmware uses (isKeya, disableKeyaSteer), the motor current and the faults kept for the web page,
   the faults reported once in the [status] messages and as the reason autosteer went off
   (zSafety.ino) instead of being printed on every heartbeat; no prints on the USB serial while steering;
   the fault name "less phase" written "phase loss".
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// KeyaCANBUS.ino
// The Keya is found by its heartbeat (CAN id 0x07000001, every 20 ms), at any time after start-up.
// Its heartbeat gives: bytes 0-1 the motor position (cumulative angle), 2-3 its speed, 4-5 its
// current, 6-7 its fault bits. Autosteer.ino uses:
//   - the motor slip as the kickout sensor (AgOpenGPS "current sensor" kickout): when the motor
//     turns slower than it was told (the driver holds the wheel), keyaSlip grows;
//   - the motor position as the wheel angle when AgOpenGPS has the WAS on "differential" (no WAS on
//     the board): position / counts per degree, corrected slowly by the wheel angle measured from
//     the GPS heading (zHandlers.ino), as in the AiO firmware;
//   - a fault (over / under voltage, over current, stall, hall, ...) turns autosteer off.

#ifndef lowByte
#define lowByte(w) ((uint8_t)((w) & 0xFF))
#define highByte(w) ((uint8_t)((w) >> 8))
#endif

// templates for commands with matching responses, only need first 4 bytes
uint8_t keyaDisableCommand[] = { 0x23, 0x0C, 0x20, 0x01 };
uint8_t keyaDisableResponse[] = { 0x60, 0x0C, 0x20, 0x00 };

uint8_t keyaEnableCommand[] = { 0x23, 0x0D, 0x20, 0x01 };
uint8_t keyaEnableResponse[] = { 0x60, 0x0D, 0x20, 0x00 };

uint8_t keyaSpeedCommand[] = { 0x23, 0x00, 0x20, 0x01 };
uint8_t keyaSpeedResponse[] = { 0x60, 0x00, 0x20, 0x00 };

uint8_t keyaCurrentQuery[] = { 0x40, 0x00, 0x21, 0x01 };
uint8_t keyaCurrentResponse[] = { 0x60, 0x00, 0x21, 0x01 };

uint8_t keyaFaultQuery[] = { 0x40, 0x12, 0x21, 0x01 };
uint8_t keyaFaultResponse[] = { 0x60, 0x12, 0x21, 0x01 };

uint8_t keyaVoltageQuery[] = { 0x40, 0x0D, 0x21, 0x02 };
uint8_t keyaVoltageResponse[] = { 0x60, 0x0D, 0x21, 0x02 };

uint8_t keyaTemperatureQuery[] = { 0x40, 0x0F, 0x21, 0x01 };
uint8_t keyaTemperatureResponse[] = { 0x60, 0x0F, 0x21, 0x01 };

uint8_t keyaVersionQuery[] = { 0x40, 0x01, 0x11, 0x11 };
uint8_t keyaVersionResponse[] = { 0x60, 0x01, 0x11, 0x11 };

uint8_t keyaEncoderResponse[] = { 0x60, 0x04, 0x21, 0x01 };

uint8_t keyaEncoderSpeedResponse[] = { 0x60, 0x03, 0x21, 0x01 };

uint64_t KeyaID = 0x06000001; // 0x01 is default ID

// What the Keya said last: keyaSteeringPosition, keyaSlip, ... (in the main .ino, which comes before
// Autosteer.ino)

void CAN_Setup()
{
	Keya_Bus.begin();
	Keya_Bus.setBaudRate(250000);
}

// The names of the fault bits (byte 7 bits 1-7, byte 6 bits 0-7)
static void keyaFaultText(uint16_t f, char* out, size_t n)
{
	static const char* const B7[8] = {nullptr, "over voltage", "hardware protection", "E2PROM", "under voltage", "N/A", "over current", "mode failure"};
	static const char* const B6[8] = {"phase loss", "motor stall", "reserved", "hall failure", "current sensing", "no RS232 steer command", "no CAN steer command", "motor stalled"};
	out[0] = '\0';
	for (uint8_t b = 1; b < 8; b++)
		if (f & (1u << b)) { size_t k = strlen(out); snprintf(out + k, n - k, "%s%s", k ? ", " : "", B7[b]); }
	for (uint8_t b = 0; b < 8; b++)
		if (f & (1u << (8 + b))) { size_t k = strlen(out); snprintf(out + k, n - k, "%s%s", k ? ", " : "", B6[b]); }
}

void KeyaBus_Receive()
{
    CAN_message_t KeyaBusReceiveData;
    if (Keya_Bus.read(KeyaBusReceiveData))
    {
        // Heatbeat
        if (KeyaBusReceiveData.id == 0x07000001)
        {
            keyaHeartbeatMs = millis();
            keyaHeartbeats++;

            if (!isKeya)
            {
                StatusLog.line("[status] Keya motor found on the CAN bus (heartbeat): steering by the Keya");
                isKeya = true;
            }
            // 0-1 - Cumulative value of angle (360 def / circle)
            // 2-3 - Motor speed, signed int eg -500 or 500
            // 4-5 - Motor current
            // 6-7 - Control_Close (error code)

            keyaSteeringPosition = (int16_t)((int16_t)KeyaBusReceiveData.buf[0] << 8 | (int16_t)KeyaBusReceiveData.buf[1]) * -1;
            keyaCurrentActualSpeed = (int16_t)((int16_t)KeyaBusReceiveData.buf[2] << 8 | (int16_t)KeyaBusReceiveData.buf[3]);
            keyaCurrent = (int16_t)((int16_t)KeyaBusReceiveData.buf[4] << 8 | (int16_t)KeyaBusReceiveData.buf[5]);

            int16_t error = abs(keyaCurrentActualSpeed - keyaCurrentSetSpeed);
            static int16_t counter = 0;

            if (error > abs(keyaCurrentSetSpeed) + 10)
            {
                if (counter++ >= 8)
                {
                    keyaSlip = abs(abs(keyaCurrentSetSpeed) - error);
                }
            }
            else
            {
                keyaSlip = 0;
                counter = 0;
            }

            // check if there's any motor diag/error data: autosteer off
            const uint16_t faults = ((uint16_t)KeyaBusReceiveData.buf[6] << 8) | (KeyaBusReceiveData.buf[7] & 0xFE);
            if (faults != (keyaFaults & 0xFFFE))
            {
                if (faults)
                {
                    char t[160];
                    keyaFaultText(faults, t, sizeof(t));
                    StatusLog.line("[status] Keya motor fault: %s - autosteer off", t);
                }
                else StatusLog.line("[status] Keya motor: faults cleared");
            }
            keyaFaults = faults | (KeyaBusReceiveData.buf[7] & 1);
            if (faults)
            {
                if (steerSwitch == 0) steerKick(SO_KEYA_FAULT, faults);
                steerSwitch = 1;
                currentState = 1;
                previous = 0;
            }
        }
    }
}

void SteerKeya(int steerSpeed, bool intendToSteer)
{
    if (isKeya)
    {
        int16_t actualSpeed;

        if (intendToSteer)
        {
            actualSpeed = steerSpeed * 3.9;
        }
        else
        {
            keyaCommand(keyaDisableCommand);
            actualSpeed = 0;
        }

        keyaCurrentSetSpeed = actualSpeed * 0.1;

        CAN_message_t KeyaBusSendData;
        KeyaBusSendData.id = KeyaID;
        KeyaBusSendData.flags.extended = true;
        KeyaBusSendData.len = 8;
        memcpy(KeyaBusSendData.buf, keyaSpeedCommand, 4);
        if (steerSpeed < 0)
        {
            KeyaBusSendData.buf[4] = highByte(actualSpeed);
            KeyaBusSendData.buf[5] = lowByte(actualSpeed);
            KeyaBusSendData.buf[6] = 0xff;
            KeyaBusSendData.buf[7] = 0xff;
        }
        else
        {
            KeyaBusSendData.buf[4] = highByte(actualSpeed);
            KeyaBusSendData.buf[5] = lowByte(actualSpeed);
            KeyaBusSendData.buf[6] = 0x00;
            KeyaBusSendData.buf[7] = 0x00;
        }
        Keya_Bus.write(KeyaBusSendData);

        if(intendToSteer) keyaCommand(keyaEnableCommand);
    }
}

// only issue one query at a time, wait for respone
void keyaCommand(uint8_t command[])
{
    if (isKeya)
    {
        CAN_message_t KeyaBusSendData;
        KeyaBusSendData.id = KeyaID;
        KeyaBusSendData.flags.extended = true;
        KeyaBusSendData.len = 8;
        memcpy(KeyaBusSendData.buf, command, 4);
        memset(KeyaBusSendData.buf + 4, 0, 4);
        Keya_Bus.write(KeyaBusSendData);
    }
}

// Motor off (autosteer off, firmware update, ...)
void disableKeyaSteer()
{
    SteerKeya(0, false);
}

// Web page: the steering card (zWeb.ino)
void keyaJson(Json& j)
{
    const uint32_t now = millis();
    char t[160];
    keyaFaultText(keyaFaults & 0xFFFE, t, sizeof(t));
    j.obj("keya");
    j.bo("found", isKeya);
    j.i("ago", keyaHeartbeats ? (long)(now - keyaHeartbeatMs) : -1);   // ms
    j.i("pos", keyaSteeringPosition);
    j.i("speed", keyaCurrentActualSpeed);
    j.i("set", keyaCurrentSetSpeed);
    j.i("current", keyaCurrent);
    j.num("slip", keyaSlip, 0);
    j.str("faults", t);
    j.num("gpsWas", wheelAngleGPS, 1);         // wheel angle from the GPS heading (zHandlers.ino)
    j.bo("gpsWasOk", wheelAngleGPSOk);
    j.close();
}
