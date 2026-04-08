// zHandlers.ino
// NMEA sentence handlers for UM982 + TM171 fused operation.
//
// Data sources:
//   GGA   (Serial7) - position, fix quality, satellites, HDOP, altitude
//   VTG   (Serial7) - speed over ground
//   KSXT  (Serial7) - UM982 dual-antenna: heading, pitch, fix status
//   TM171 (Serial5) - roll, pitch, yaw rate (always running, always used for roll)
//
// Fusion strategy:
//   ROLL -> always from TM171 (accelerometer-based, faster and more stable
//             than GPS baseline roll, immune to antenna blockage)
//   HEADING:
//     KSXT valid (fix >= 1, received within 500ms)
//        -> heading from UM982 dual-antenna (most accurate)
//        -> send $PAOGI (AgOpenGPS dual-antenna format)
//        -> AgOpenGPS IMU fusion will blend UM982 heading with TM171 yaw rate
//          to bridge momentary antenna blockages under trees
//     KSXT absent or stale
//        -> heading from TM171 (magnetometer-based fallback)
//        -> send $PANDA (single antenna + IMU format)
//     Neither available
//        -> heading = 65535 (AgOpenGPS ignores IMU, position-only mode)
//
// Why this matters under trees:
//   When one antenna loses signal, KSXT heading degrades or stops.
//   AgOpenGPS fusion uses the TM171 gyro yaw rate to coast through the gap.
//   TM171 roll is unaffected by antenna blockage entirely.
//
// KSXT field layout (0-indexed):
//   0  - UTC time        4  - Heading (deg, 0-360 true)
//   1  - Latitude        5  - Pitch   (deg)
//   2  - Longitude       6  - Roll    (deg, not used - TM171 preferred)
//   3  - Altitude (m)    7  - Speed (m/s)
//                        8  - HDOP
//                        9  - Fix status (4=RTK fixed,5=float,1=GPS,0=invalid)
//                        10 - Satellites

const char* asciiHex = "0123456789ABCDEF";

// KSXT heading state
bool ksxtHeadingValid  = false;
unsigned long ksxtLastReceived = 0;
#define KSXT_TIMEOUT_MS 500     // ms without KSXT before falling back to TM171 heading

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

// Output fields written into PANDA/PAOGI sentence
char imuHeading[12];
char imuRoll[12];
char imuPitch[12];
char imuYawRate[6];

void errorHandler()
{
    // NMEA parse error - silent
}

// -----------------------------------------------------------------------
// fillFromTM171 - write roll+pitch+yawRate from TM171 into output fields
// Called regardless of KSXT state - TM171 always provides roll
// -----------------------------------------------------------------------
void fillRollPitchFromTM171()
{
    if (!useTM171) return;

    if (!steerConfig.IsUseY_Axis)
    {
        dtostrf(PitchV.fValue, 6, 2, imuPitch);
        dtostrf(RollV.fValue,  6, 2, imuRoll);
    }
    else
    {
        dtostrf(RollV.fValue,  6, 2, imuPitch);
        dtostrf(PitchV.fValue, 6, 2, imuRoll);
    }
    itoa(0, imuYawRate, 10);
}

// -----------------------------------------------------------------------
// GGA Handler - fires every GPS epoch, triggers sentence build
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
    blink        = !blink;
    GGA_Available = true;
    gpsReadyTime  = systick_millis_count;

    bool ksxtFresh = ksxtHeadingValid && (millis() - ksxtLastReceived < KSXT_TIMEOUT_MS);

    // Roll and pitch always come from TM171 (best source regardless of GPS state)
    if (useTM171)
    {
        fillRollPitchFromTM171();
    }

    if (ksxtFresh)
    {
        // --- FUSED MODE ---
        // Heading: UM982 dual-antenna (imuHeading already set in KSXT_Handler)
        // Roll/Pitch: TM171 (set above, overwrites KSXT roll)
        // Format: $PAOGI - AgOpenGPS reads this as dual-antenna + IMU fusion
        digitalWrite(GPSGREEN_LED, HIGH);
        digitalWrite(GPSRED_LED,   LOW);
        BuildNmea();
    }
    else if (useTM171)
    {
        // --- TM171 FALLBACK MODE ---
        // KSXT lost (antenna blocked) - use TM171 heading + roll
        // AgOpenGPS IMU fusion will use yaw rate to maintain heading
        ksxtHeadingValid = false;
        dtostrf(YawV.fValue, 8, 4, imuHeading);
        // roll/pitch already set by fillRollPitchFromTM171() above
        digitalWrite(GPSRED_LED,   HIGH);
        digitalWrite(GPSGREEN_LED, LOW);
        BuildNmea();
    }
    else
    {
        // --- NO HEADING SOURCE ---
        // GPS position only - AgOpenGPS will not use IMU heading
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
// VTG Handler - speed over ground
// -----------------------------------------------------------------------
void VTG_Handler()
{
    parser.getArg(0, vtgHeading);
    parser.getArg(4, speedKnots);
}

// -----------------------------------------------------------------------
// KSXT Handler - UM982 dual-antenna heading sentence
// Runs at GPS rate (e.g. 10 Hz), independently of GGA.
// Only updates heading - roll/pitch will be overwritten by TM171 in GGA_Handler.
// -----------------------------------------------------------------------
void KSXT_Handler()
{
    char headingStr[12] = {};
    char pitchStr[12]   = {};
    char fixStr[4]      = {};

    parser.getArg(4, headingStr);   // heading degrees 0-360
    parser.getArg(5, pitchStr);     // pitch degrees
    parser.getArg(9, fixStr);       // fix status

    int fixStatus = atoi(fixStr);

    if (fixStatus >= 1 && strlen(headingStr) > 0)
    {
        float hdg = atof(headingStr);
        float pch = atof(pitchStr);

        // Store heading for GGA_Handler to use
        // Note: KSXT roll (field 6) is intentionally NOT used here -
        // TM171 roll will overwrite it in GGA_Handler for better stability
        dtostrf(hdg, 8, 4, imuHeading);
        dtostrf(pch, 6, 2, imuPitch);
        // imuRoll left for TM171 to fill

        heading          = hdg;
        ksxtHeadingValid = true;
        ksxtLastReceived = millis();

        // Green LED toggles on each valid KSXT packet
        digitalWrite(GPSGREEN_LED, !digitalRead(GPSGREEN_LED));
        digitalWrite(GPSRED_LED,   LOW);

        // Forward raw KSXT sentence to AgIO - AgOpenGPS can use it for fusion
        /*char sentence[256];
        parser.getType(sentence);
        char ksxtBuf[256];
        strcpy(ksxtBuf, "$");
        strcat(ksxtBuf, sentence);
        for (int i = 0; i < parser.argCount(); i++)
        {
            strcat(ksxtBuf, ",");
            char arg[64];
            parser.getArg(i, arg);
            strcat(ksxtBuf, arg);
        }
        int16_t sum = 0;
        for (unsigned int i = 1; i < strlen(ksxtBuf); i++) sum ^= ksxtBuf[i];
        char chk[8];
        sprintf(chk, "*%02X\r\n", sum);
        strcat(ksxtBuf, chk);

        if (!passThroughGPS && !passThroughGPS2)
            SerialAOG.print(ksxtBuf);

        if (Ethernet_running)
        {
            int len = strlen(ksxtBuf);
            Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
            Eth_udpPAOGI.write(ksxtBuf, len);
            Eth_udpPAOGI.endPacket();
        }*/
    }
    else
    {
        // KSXT received but fix invalid (e.g. only one antenna has signal)
        ksxtHeadingValid = false;
    }
}

// -----------------------------------------------------------------------
// imuHandler - called from main loop for TM171 timed trigger
// Kept for compatibility with the imuTrigger / imuTimer mechanism.
// With UM982, the actual sentence build happens in GGA_Handler.
// This just keeps the TM171 data fresh between GGA epochs.
// -----------------------------------------------------------------------
void imuHandler()
{
    // TM171 data is read continuously by TM171process() in the main loop.
    // YawV, RollV, PitchV are always up to date.
    // Nothing extra needed here - GGA_Handler calls fillRollPitchFromTM171().
}

// -----------------------------------------------------------------------
// BuildNmea - assemble and transmit PANDA or PAOGI sentence
// $PAOGI when KSXT heading is valid (dual-antenna + IMU fused mode)
// $PANDA when falling back to TM171 heading only
// -----------------------------------------------------------------------
void BuildNmea()
{
    strcpy(nmea, "");

    bool ksxtFresh = ksxtHeadingValid && (millis() - ksxtLastReceived < KSXT_TIMEOUT_MS);
    if (ksxtFresh)
        strcat(nmea, "$PAOGI,");   // dual-antenna format: AgOpenGPS applies IMU fusion
    else
        strcat(nmea, "$PANDA,");   // single antenna + IMU format

    strcat(nmea, fixTime);    strcat(nmea, ",");
    strcat(nmea, latitude);   strcat(nmea, ",");
    strcat(nmea, latNS);      strcat(nmea, ",");
    strcat(nmea, longitude);  strcat(nmea, ",");
    strcat(nmea, lonEW);      strcat(nmea, ",");
    strcat(nmea, fixQuality); strcat(nmea, ",");
    strcat(nmea, numSats);    strcat(nmea, ",");
    strcat(nmea, HDOP);       strcat(nmea, ",");
    strcat(nmea, altitude);   strcat(nmea, ",");
    strcat(nmea, ageDGPS);    strcat(nmea, ",");
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
