// zHandlers.ino
// NMEA sentence handlers - DEDICATED F9P/X20P (single antenna) + TM171 build.
// This firmware assumes a fixed hardware configuration: no runtime
// detection, no dual-antenna support, no KSXT parsing.
//
// Data sources:
//   GGA  (Serial7)  - position, fix quality, satellites, HDOP, altitude
//   VTG  (Serial7)  - speed over ground
//   TM171 (Serial5) - heading, roll, pitch (always running, only IMU source)
//
// Fusion strategy:
//   HEADING -> always from TM171 (single-antenna GPS has no heading of
//              its own; AgOpenGPS fuses this IMU heading with the GPS
//              track heading calculated from consecutive fixes)
//   ROLL    -> always from TM171 (used for antenna-height lateral
//              position correction)
//   PITCH   -> always from TM171 (informational)
//
// There is no dual-antenna fallback logic here: with a single F9P/X20P
// there is no KSXT sentence and no second heading source to switch to.
// Every GGA epoch always sends $PANDA with the current TM171 reading.
//
// Exactly one NMEA sentence is built and sent per GGA epoch (one
// isNMEAToSend event in AgIO), which is what keeps AgOpenGPS's udpWatch
// timer happy - no missed-sentence warnings.

const char* asciiHex = "0123456789ABCDEF";

// TM171 mounting orientation on this board.
// Set once for the dedicated PCB layout - not a runtime setting.
// 0 = pitch/roll as read from TM171 directly
// 1 = swap pitch/roll (TM171 rotated 90 deg on the board)
#define TM171_SWAP_ROLL_PITCH 0

// Sentence buffer
// Measured worst case for a full $PANDA sentence is ~112 bytes;
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

// Output fields written into the PANDA sentence
char imuHeading[12];
char imuRoll[12];
char imuPitch[12];
char imuYawRate[6];

void errorHandler()
{
    // NMEA parse error - silent
}

// -----------------------------------------------------------------------
// fillFromTM171 - write heading+roll+pitch+yawRate from TM171 into
// output fields. TM171 is the only IMU source on this board, so this
// always runs unconditionally.
// -----------------------------------------------------------------------
void fillFromTM171()
{
    dtostrf(YawV.fValue, 8, 4, imuHeading);

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
// Exactly one BuildNmea() call per GGA - single antenna GPS, single IMU
// source, no branching needed.
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

    // Heading, roll and pitch all come from TM171 - the only IMU on this board
    fillFromTM171();

    digitalWrite(GPSGREEN_LED, HIGH);
    digitalWrite(GPSRED_LED,   LOW);

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
// imuHandler - called from main loop for TM171 timed trigger.
// TM171 data is read continuously by TM171process() in the main loop;
// YawV, RollV, PitchV are always up to date. Sentence build happens in
// GGA_Handler, so nothing extra is needed here.
// -----------------------------------------------------------------------
void imuHandler()
{
}

// -----------------------------------------------------------------------
// BuildNmea - assemble and transmit the PANDA sentence.
// Always $PANDA on this board: single antenna GPS has no dual-antenna
// heading to report, so there is no $PAOGI case here.
// Called exactly once per GGA epoch.
// -----------------------------------------------------------------------
void BuildNmea()
{
    strcpy(nmea, "");
    strcat(nmea, "$PANDA,");

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
