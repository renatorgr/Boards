// zHandlers.ino
// NMEA sentence handlers for UM982 + TM171 fused operation.
//
// Architecture matches official AIO v4 firmware pattern:
//   - KSXT sentence is forwarded RAW to AOG via UDP - AOG reads heading from it directly
//   - PANDA is sent with TM171 roll/pitch + TM171 heading (fallback when no KSXT)
//   - When KSXT is fresh, PANDA is suppressed - AOG fuses KSXT heading with PANDA roll
//
// Roll/Pitch format:  integer * 10  (e.g. -90.5 deg = -905) - matches official firmware
// Heading format:     float string, plain degrees 0-360     - matches official firmware

const char* asciiHex = "0123456789ABCDEF";

// KSXT state - when fresh, suppress PANDA (AOG reads heading from KSXT directly)
bool ksxtReceived = false;
unsigned long ksxtLastReceived = 0;
#define KSXT_TIMEOUT_MS 500

// Sentence buffer
char nmea[100];

// GGA fields
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char ageDGPS[10];

// VTG fields
char vtgHeading[12] = {};
char speedKnots[10] = {};

// IMU output fields - written by imuHandler(), read by BuildNmea()
char imuHeading[12];
char imuRoll[12];
char imuPitch[12];
char imuYawRate[6];

void errorHandler()
{
    // NMEA parse error - silent
}

// -----------------------------------------------------------------------
// imuHandler - fills imuHeading/Roll/Pitch from TM171
// Matches official firmware format exactly:
//   heading * 10 integer (TM171 yaw in degrees * 10)
//   roll    * 10 integer
//   pitch   * 10 integer
// -----------------------------------------------------------------------
void imuHandler()
{
    if (useTM171)
    {
        // Heading from TM171 yaw * 10 as integer - matches official firmware
        itoa((int16_t)(YawV.fValue * 10.0), imuHeading, 10);

        if (!steerConfig.IsUseY_Axis)
        {
            itoa((int16_t)(PitchV.fValue * 10.0), imuPitch, 10);
            itoa((int16_t)(RollV.fValue  * 10.0), imuRoll,  10);
        }
        else
        {
            // Y-axis swap
            itoa((int16_t)(RollV.fValue  * 10.0), imuPitch, 10);
            itoa((int16_t)(PitchV.fValue * 10.0), imuRoll,  10);
        }

        itoa(0, imuYawRate, 10);
    }
}

// -----------------------------------------------------------------------
// GGA Handler - fires every GPS epoch
// -----------------------------------------------------------------------
void GGA_Handler()
{
    // One-time GPS acquired notification
    static bool gpsAcquired = false;
    if (!gpsAcquired)
    {
        gpsAcquired = true;
        Serial.println("GNSS signal acquired.");
    }

    parser.getArg(0,  fixTime);
    parser.getArg(1,  latitude);
    parser.getArg(2,  latNS);
    parser.getArg(3,  longitude);
    parser.getArg(4,  lonEW);
    parser.getArg(5,  fixQuality);
    parser.getArg(6,  numSats);
    parser.getArg(7,  HDOP);
    parser.getArg(8,  altitude);
    parser.getArg(12, ageDGPS);

    digitalWrite(GGAReceivedLED, blink ? HIGH : LOW);
    blink         = !blink;
    GGA_Available = true;
    gpsReadyTime  = systick_millis_count;

    if (useTM171)
    {
        imuHandler();   // Fill imuHeading/Roll/Pitch from TM171
        BuildNmea();    // Send PANDA (suppressed if KSXT fresh)

        if (!ksxtReceived || (millis() - ksxtLastReceived >= KSXT_TIMEOUT_MS))
        {
            // No KSXT - TM171 fallback mode
            digitalWrite(GPSRED_LED,   HIGH);
            digitalWrite(GPSGREEN_LED, LOW);
        }
    }
    else
    {
        // No IMU - position only
        itoa(65535, imuHeading, 10);
        itoa(0,     imuRoll,    10);
        itoa(0,     imuPitch,   10);
        itoa(0,     imuYawRate, 10);
        digitalWrite(GPSRED_LED,   blink ? HIGH : LOW);
        digitalWrite(GPSGREEN_LED, LOW);
        BuildNmea();
    }
}

// -----------------------------------------------------------------------
// VTG Handler
// -----------------------------------------------------------------------
void VTG_Handler()
{
    parser.getArg(0, vtgHeading);
    parser.getArg(4, speedKnots);
}

// -----------------------------------------------------------------------
// KSXT Handler - UM982 dual-antenna sentence
// Forwards raw KSXT to AOG via UDP.
// AOG 6.8.2 reads heading directly from KSXT - no need to build PAOGI.
// Sets ksxtReceived flag to suppress PANDA while KSXT is fresh.
// -----------------------------------------------------------------------
void KSXT_Handler()
{
    // Toggle green LED on each valid KSXT
    digitalWrite(GPSGREEN_LED, !digitalRead(GPSGREEN_LED));
    digitalWrite(GPSRED_LED,   LOW);

    // Reconstruct full KSXT sentence
    char ksxtBuf[256];
    char sentence[32];
    parser.getType(sentence);

    strcpy(ksxtBuf, "$");
    strcat(ksxtBuf, sentence);

    for (int i = 0; i < parser.argCount(); i++)
    {
        strcat(ksxtBuf, ",");
        char arg[64];
        parser.getArg(i, arg);
        strcat(ksxtBuf, arg);
    }

    // Calculate checksum
    int16_t sum = 0;
    for (unsigned int i = 1; i < strlen(ksxtBuf); i++)
        sum ^= ksxtBuf[i];

    char chk[8];
    sprintf(chk, "*%02X\r\n", sum);
    strcat(ksxtBuf, chk);

    // Mark KSXT fresh - suppresses PANDA in BuildNmea()
    ksxtReceived    = true;
    ksxtLastReceived = millis();

    // Forward raw KSXT to AOG - AOG reads heading from it directly
    if (Ethernet_running)
    {
        int len = strlen(ksxtBuf);
        Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
        Eth_udpPAOGI.write(ksxtBuf, len);
        Eth_udpPAOGI.endPacket();
    }
}

// -----------------------------------------------------------------------
// BuildNmea - send PANDA sentence
// Suppressed when KSXT is fresh (AOG gets heading from KSXT directly).
// Always sends roll/pitch from TM171 regardless.
// -----------------------------------------------------------------------
void BuildNmea()
{
    // If KSXT was recently received, suppress PANDA
    // AOG reads heading from the raw KSXT we forwarded in KSXT_Handler
    if (ksxtReceived && (millis() - ksxtLastReceived < KSXT_TIMEOUT_MS))
    {
        // Still send PANDA with roll/pitch only - heading ignored by AOG when KSXT present
        // This keeps roll correction active even in KSXT mode
    }
    else
    {
        ksxtReceived = false;
    }

    strcpy(nmea, "");
    strcat(nmea, "$PANDA,");   // Always PANDA - heading comes from KSXT directly in AOG

    strcat(nmea, fixTime);    strcat(nmea, ",");
    strcat(nmea, latitude);   strcat(nmea, ",");
    strcat(nmea, latNS);      strcat(nmea, ",");
    strcat(nmea, longitude);  strcat(nmea, ",");
    strcat(nmea, lonEW);      strcat(nmea, ",");
    strcat(nmea, fixQuality); strcat(nmea, ",");
    strcat(nmea, numSats);    strcat(nmea, ",");
    strcat(nmea, HDOP);       strcat(nmea, ",");
    strcat(nmea, altitude);   strcat(nmea, ",");

    // ageDGPS - use "0" if empty to avoid field shift
    if (strlen(ageDGPS) == 0)
        strcat(nmea, "0");
    else
        strcat(nmea, ageDGPS);
    strcat(nmea, ",");

    strcat(nmea, speedKnots); strcat(nmea, ",");
    strcat(nmea, imuHeading); strcat(nmea, ",");
    strcat(nmea, imuRoll);    strcat(nmea, ",");
    strcat(nmea, imuPitch);   strcat(nmea, ",");
    strcat(nmea, imuYawRate);
    strcat(nmea, "*");

    CalculateChecksum();
    strcat(nmea, "\r\n");

    if (Ethernet_running)
    {
        int len = strlen(nmea);
        Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
        Eth_udpPAOGI.write(nmea, len);
        Eth_udpPAOGI.endPacket();
    }
}

void CalculateChecksum()
{
    int16_t sum = 0;
    for (int16_t inx = 1; inx < 200; inx++)
    {
        char tmp = nmea[inx];
        if (tmp == '*') break;
        sum ^= tmp;
    }
    byte chk = (sum >> 4);
    char hex[2]  = {asciiHex[chk], 0};
    strcat(nmea, hex);
    chk = (sum % 16);
    char hex2[2] = {asciiHex[chk], 0};
    strcat(nmea, hex2);
}
