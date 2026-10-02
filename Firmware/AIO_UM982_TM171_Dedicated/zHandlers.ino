// zHandlers.ino
// NMEA sentence handlers - DEDICATED UM982 (dual-antenna) + TM171 build.
// This firmware assumes a fixed hardware configuration: no runtime
// detection, no fallback to other GPS/IMU combinations.
//
// Data sources:
//   GGA  (Serial7)  - position, fix quality, satellites, HDOP, altitude
//   VTG  (Serial7)  - speed over ground
//   KSXT (Serial7)  - UM982 dual-antenna: heading, pitch, fix status
//   TM171 (Serial5) - roll, pitch, yaw rate (always running, always used for roll)
//
// Fusion strategy:
//   ROLL    -> always from TM171 (accelerometer-based, faster and more
//              stable than GPS baseline roll, immune to antenna blockage)
//   HEADING:
//     KSXT valid (fix >= 1, received within KSXT_TIMEOUT_MS)
//       -> heading from UM982 dual-antenna (most accurate)
//       -> send $PAOGI (AgOpenGPS dual-antenna format)
//       -> AgOpenGPS IMU fusion blends UM982 heading with TM171 yaw rate
//          to bridge momentary antenna blockage under trees/structures
//     KSXT absent or stale (antenna blocked)
//       -> heading from TM171
//       -> send $PANDA (IMU-heading format)
//
// There is no "no heading source" case: TM171 is always present on this
// board, so GGA_Handler always has a heading to send.
//
// The KSXT sentence is parsed for heading/pitch only and is NEVER
// forwarded to AgIO as a separate UDP packet. Only one NMEA sentence is
// built and sent per GGA epoch (one isNMEAToSend event in AgIO), which
// keeps AgOpenGPS's udpWatch timer happy - no missed-sentence warnings.
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

// TM171 mounting orientation on this board.
// Set once for the dedicated PCB layout - not a runtime setting.
// 0 = pitch/roll as read from TM171 directly
// 1 = swap pitch/roll (TM171 rotated 90 deg on the board)
#define TM171_SWAP_ROLL_PITCH 0

// Sentence buffer
// Measured worst case for a full $PAOGI/$PANDA sentence is ~112 bytes;
// 160 leaves comfortable headroom.
char nmea[160];

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
// fillRollPitchFromTM171 - write roll+pitch+yawRate from TM171 into
// output fields. TM171 is always present on this board, so this always
// runs - no "if useTM171" guard needed.
// -----------------------------------------------------------------------
void fillRollPitchFromTM171()
{
#if TM171_SWAP_ROLL_PITCH
    dtostrf(RollV.fValue,  6, 2, imuPitch);
    dtostrf(PitchV.fValue, 6, 2, imuRoll);
#else
    dtostrf(PitchV.fValue, 6, 2, imuPitch);
    dtostrf(RollV.fValue,  6, 2, imuRoll);
#endif
    itoa(0, imuYawRate, 10);   // TM171 does not report angular rate directly
}

// -----------------------------------------------------------------------
// GGA Handler - fires every GPS epoch, triggers sentence build.
// Exactly one BuildNmea() call per GGA - this is what keeps AgIO/AgOpenGPS
// from ever seeing two PGN 0xD6 events in the same cycle.
// -----------------------------------------------------------------------
void GGA_Handler()
{
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

    // Roll and pitch always come from TM171 (best source regardless of GPS state)
    fillRollPitchFromTM171();

    bool ksxtFresh = ksxtHeadingValid && (millis() - ksxtLastReceived < KSXT_TIMEOUT_MS);

    if (ksxtFresh)
    {
        // --- FUSED MODE ---
        // Heading: UM982 dual-antenna (imuHeading already set in KSXT_Handler)
        // Roll/Pitch: TM171 (set above, overwrites KSXT roll/pitch)
        // Format: $PAOGI - AgOpenGPS reads this as dual-antenna + IMU fusion
        digitalWrite(GPSGREEN_LED, HIGH);
        digitalWrite(GPSRED_LED,   LOW);
    }
    else
    {
        // --- TM171 FALLBACK MODE ---
        // KSXT lost (antenna blocked) - use TM171 heading + roll/pitch
        // AgOpenGPS IMU fusion uses yaw rate to maintain heading through the gap
        ksxtHeadingValid = false;
        dtostrf(YawV.fValue, 8, 4, imuHeading);
        digitalWrite(GPSRED_LED,   HIGH);
        digitalWrite(GPSGREEN_LED, LOW);
    }

    BuildNmea();
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
// KSXT Handler - UM982 dual-antenna heading sentence.
// Runs at GPS rate (10Hz), independently of GGA.
// Only stores heading/pitch for GGA_Handler to use - never sends
// anything by itself. Roll (field 6) is intentionally not read; TM171
// roll overwrites it in GGA_Handler for better stability.
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

        dtostrf(hdg, 8, 4, imuHeading);
        dtostrf(pch, 6, 2, imuPitch);
        // imuRoll intentionally left for TM171 to fill in GGA_Handler

        heading          = hdg;
        ksxtHeadingValid = true;
        ksxtLastReceived = millis();

        // Green LED toggles on each valid KSXT packet
        digitalWrite(GPSGREEN_LED, !digitalRead(GPSGREEN_LED));
        digitalWrite(GPSRED_LED,   LOW);
    }
    else
    {
        // KSXT received but fix invalid (e.g. only one antenna has signal)
        ksxtHeadingValid = false;
    }
}

// -----------------------------------------------------------------------
// imuHandler - called from main loop for TM171 timed trigger.
// TM171 data is read continuously by TM171process() in the main loop;
// YawV, RollV, PitchV are always up to date. Sentence build happens in
// GGA_Handler, so nothing extra is needed here.
// -----------------------------------------------------------------------
void imuHandler()
{
}

// -----------------------------------------------------------------------
// BuildNmea - assemble and transmit PANDA or PAOGI sentence.
// $PAOGI when KSXT heading is valid (dual-antenna + IMU fused mode)
// $PANDA when falling back to TM171 heading only.
// Called exactly once per GGA epoch - never called from KSXT_Handler.
// -----------------------------------------------------------------------
void BuildNmea()
{
    strcpy(nmea, "");

    bool ksxtFresh = ksxtHeadingValid && (millis() - ksxtLastReceived < KSXT_TIMEOUT_MS);
    if (ksxtFresh)
        strcat(nmea, "$PAOGI,");   // dual-antenna format: AgOpenGPS applies IMU fusion
    else
        strcat(nmea, "$PANDA,");   // TM171-only heading format

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
    for (int16_t inx = 1; inx < 160; inx++)   // must match nmea[] buffer size above
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
