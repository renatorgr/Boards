/* TM171.ino - reads the TM171 IMU (Serial5).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards.
   Based on the TM171.ino of Balázs Gunics' fork of the AgOpenGPS boards
   (github.com/gunicsba/agopengps_boards, firmware V4.1),
   which is based on the work of Paulius a.k.a. babtai (original line kept below).
   Changes 2026 by Claude (Anthropic) for renatorgr: Serial5, bad CRC packets and over-long frames counted instead of printed; the port type
   TeensySerial (so it compiles with Teensyduino 1.58 and 1.59 or newer); one byte at a
   time (TM171byte), so zImu.ino can listen for a TM171 and a BNO085 on Serial5 at the same time; two compiler
   warnings removed (same code).
   AiO_WEB 4.5, 23 Oct 2026: commands TO the TM171 (tm171Send) and the gyro calibration from the web page
   (tm171Calib*), after the SYD Dynamics TransducerM user guide (EasyProtocol, Calibration object) and
   application note AN002. Needs Teensy TX5 (pin 20) wired to the TM171 RXD.
   AiO_WEB 4.5, 23 Oct 2026: the "new install" calibration (CalibB, saved in the TM171's flash) as a second button.
   AiO_WEB 4.5, 23 Oct 2026: the TM171's settings (Setting object 21) read and written from the web page
   (tm171Setting*): magnetometer, filters and gains, outputs, boot mode, output data rate.
   AiO_WEB 4.5, 24 Oct 2026: the layout of the Setting object corrected from a real TM171 (the output data
   rate is its own field in Hz; the inhibit time and the other words are left as read); the raw bytes in the
   log and on the page; the "RX not wired" warning only after a second unanswered request.
   AiO_WEB 4.5, 24 Oct 2026: a settings request was timed out at once when its timer was set a millisecond
   after the loop's "now" (unsigned wrap): a false "no reply" line at every start-up (tmElapsed).
   AiO_WEB 4.5, 23 Oct 2026: set-up check. At start-up the TM171 is also looked for at the other baud rates
   (tm171ScanBaud); a baud rate other than 115200, or settings that differ from the AgOpenGPS ones, give a
   warning on the web page and on the program's screen, and a "Fix TM171 setup" button (tm171FixSetup).
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
//Based on the work of Paulius a.k.a. babtai RTK!

TeensySerial* SerialImu = &Serial5;   // TM171 IMU on Serial5

//constexpr int serial_buffer_size = 512;
uint8_t SerialImurxbuffer[serial_buffer_size];    //Extra serial tx buffer
uint8_t SerialImutxbuffer[serial_buffer_size];    //Extra serial tx buffer

//State machine from the almighty ChatGPT
enum TM171ParseState {
  WAIT_HEADER_1,
  WAIT_HEADER_2,
  WAIT_LENGTH,
  WAIT_PAYLOAD
};

TM171ParseState parseState = WAIT_HEADER_1;
uint8_t packetLength = 0;
uint8_t payloadIndex = 0;
bool gotPacket = false;

uint8_t ImuData[64];
uint8_t ImuDC = 0 ;

union Onion
{
    uint8_t     fBytes[sizeof( float )];
    float       fValue;
};

Onion YawV;
Onion RollV;
Onion PitchV;

Onion TemperatureV;
uint8_t qos;

bool TM171DataSeen = false;
uint32_t TM171crcErrors = 0;   // packets dropped for a bad CRC (shown by FUSION_DEBUG)
// AiO_WEB: used by the parser below, defined with the settings code further down
#define TM171_SET_LEN 20                              // Setting object content, bytes
static void tm171SettingGot(const uint8_t* p);
static void tm171SettingHex(char* out);
uint32_t tmSetAckMs = 0;                              // the TM171 acknowledged our settings write
uint32_t TM171rpyCount = 0;                           // roll-pitch-yaw packets this second

//#define TM171DEBUG

uint32_t tm171Baud = 115200;   // AiO_WEB: the baud rate Serial5 runs at (tm171ScanBaud may change it)

void TM171setup() {
  SerialImu->begin(tm171Baud); 
  SerialImu->addMemoryForWrite(SerialImutxbuffer, serial_buffer_size);
  SerialImu->addMemoryForRead(SerialImurxbuffer, serial_buffer_size);
}

uint32_t TM171goodPackets = 0;   // packets with a good CRC (zImu.ino: a TM171 is there)

// one byte from Serial5. True when it completed a packet with a good CRC.
bool TM171byte(uint8_t temp)
{
    bool good = false;
    switch (parseState) 
    {
      case WAIT_HEADER_1:
        if (temp == 0xAA)
        {
          ImuData[0] = temp;
          parseState = WAIT_HEADER_2;
        }
        break;

      case WAIT_HEADER_2:
        if (temp == 0x55)
        {
          ImuData[1] = temp;
          parseState = WAIT_LENGTH;
        }
        else
        {
          parseState = WAIT_HEADER_1; //Reset if second byte isn't 0x55
        }
        break;
      
      case WAIT_LENGTH:
        packetLength = temp;
        ImuData[2] = temp;
        payloadIndex = 3; // We've stored 3 bytes so far
        parseState = WAIT_PAYLOAD;

        //check for unreasonable lengths
        if ((size_t)packetLength + 5 > sizeof(ImuData))   // (size_t), no compiler warning
        {
          parseState = WAIT_HEADER_1;
          TM171crcErrors++;                                   // counted with the bad CRCs (web page)
#ifdef TM171DEBUG
          Serial.println("[tm171] frame too long, dropped");
#endif
        }
        break;
      
      case WAIT_PAYLOAD:
        ImuData[payloadIndex++] = temp;

        if (payloadIndex >= packetLength + 5) 
        {
          gotPacket = true;
          parseState = WAIT_HEADER_1;
        }
        break;
    }
    if (gotPacket)
    {
      gotPacket = false;
//      if (GoodCRC(ImuData,packetLength+5))
      if (GoodCRC(ImuData,ImuData[2]+5))
      {
        good = true;
        TM171goodPackets++;
        TM171lastData = 0;
        TM171DataSeen = true;
        uint8_t functionCode = ImuData[3]; // Function ID (documented at 4th byte)
        switch (functionCode)
        {
        case 35: //RPY Output
          TM171rpyCount++;                                  // AiO_WEB: the output rate, shown by the web page
          RollV.fBytes[0] = ImuData[11];
          RollV.fBytes[1] = ImuData[12];
          RollV.fBytes[2] = ImuData[13];
          RollV.fBytes[3] = ImuData[14];
      
          PitchV.fBytes[0] = ImuData[15];
          PitchV.fBytes[1] = ImuData[16];
          PitchV.fBytes[2] = ImuData[17];
          PitchV.fBytes[3] = ImuData[18];

          YawV.fBytes[0] = ImuData[19];
          YawV.fBytes[1] = ImuData[20];
          YawV.fBytes[2] = ImuData[21];
          YawV.fBytes[3] = ImuData[22];
      
#ifdef TM171DEBUG
          Serial.print("[tm171] yaw=");
          Serial.print(YawV.fValue);
          Serial.print(",pitch=");
          Serial.print(PitchV.fValue);
          Serial.print(",Roll=");
          Serial.println(RollV.fValue);
#endif
        break;
        
        case 23: // Calibration status report (AiO_WEB: the gyro calibration from the web page, below)
          tm171CalibReport(ImuData[7], ImuData[8]);     // content: type, control value
        break;

        case 21: // Setting object (AiO_WEB: answer to our request, below)
          if (packetLength == 4 + TM171_SET_LEN) tm171SettingGot(ImuData + 7);
        break;

        case 13: // Acknowledge (AiO_WEB: byte 0 = the object acknowledged)
          if (ImuData[7] == 21) tmSetAckMs = millis() | 1;
        break;

        case 22: // Status Output
          TemperatureV.fBytes[0] = ImuData[11];
          TemperatureV.fBytes[1] = ImuData[12];
          TemperatureV.fBytes[2] = ImuData[13];
          TemperatureV.fBytes[3] = ImuData[14];
          qos = ImuData[17] & 0x07;
#ifdef TM171DEBUG
          Serial.print("[tm171] temperature "); Serial.print(TemperatureV.fValue);
          Serial.print(", QoS "); Serial.println(qos);
#endif
        break;
        
        default:
#ifdef TM171DEBUG
          Serial.print("[tm171] unhandled packet type 0x");
          Serial.println(functionCode, HEX);
#endif
        break;
        }
      }
       else {
        // An occasional bad CRC is normal (a corrupted byte on the wire, or the
        // first packet after a long blocking step): the packet is dropped and the
        // next one, 25 per second, replaces it. Counted instead of printed.
        TM171crcErrors++;
#ifdef TM171DEBUG
        Serial.println("[tm171] bad CRC, packet dropped");
#endif
      }
      // Reset for next packet
/*
      ImuData[0] = 0;
      ImuData[1] = 0;
      ImuData[2] = 0;  
*/
    } //gotPacket
    return good;
}

void TM171process() {
  while (SerialImu->available()) TM171byte(SerialImu->read());
}

// ---------------------------------------------------------------- commands to the TM171 (AiO_WEB)
// EasyProtocol frame (TransducerM user guide): AA 55, length (4 + content), payload information (32 bits
// little-endian: CMD 7 bits, 3 reserved, FROM_ID 11 bits, TO_ID 11 bits), content, Modbus CRC16 of the
// length and payload (little-endian). The host is ID 2; TO_ID 0 reaches every module.
// Only works with the Teensy TX5 (pin 20) wired to the TM171 RXD: the board joins the TM171 TXD to RX5
// (jumper SJ1), the other direction may need a wire.
static uint16_t MODBUS_CRC16_v3(const unsigned char* buf, unsigned int len);   // below
static void tm171Send(uint8_t cmd, const uint8_t* content, uint8_t n)
{
    if (n > 64) return;
    uint8_t f[3 + 4 + 64 + 2];
    f[0] = 0xAA; f[1] = 0x55; f[2] = (uint8_t)(4 + n);
    const uint32_t info = (uint32_t)(cmd & 0x7F) | (2UL << 10);     // from the host (2) to everyone (0)
    f[3] = (uint8_t)info; f[4] = (uint8_t)(info >> 8); f[5] = (uint8_t)(info >> 16); f[6] = (uint8_t)(info >> 24);
    memcpy(f + 7, content, n);
    const uint16_t crc = MODBUS_CRC16_v3(f, 3 + 4 + n);              // (it skips the 2 head bytes itself)
    f[7 + n] = (uint8_t)crc; f[8 + n] = (uint8_t)(crc >> 8);
    SerialImu->write(f, 9 + n);
}

// Gyro bias calibration ("run-time static / forced calibration", Calibration object 23, type 0x05, key
// 0xAA65) from the web page. The TM171 collects data while the tractor stands still (the engine may run:
// SYD AN002), the firmware tells it to finish after TM171_CALIB_S seconds, and the TM171 reports success
// (control 0x01) or failure (0xFF). The user guide: the result is kept in RAM, so it lasts until the TM171
// loses power. Autosteer on, or the tractor moving, stops the calibration.
#define TM171_CALIB_S       20     // seconds of data (SYD: anything from milliseconds to half an hour; at least 12 s is a full one)
#define TM171_CALIB_LIMIT   50     // accepted gyro error, x 0.01 deg/s (5..500): more is discarded by the TM171
#define TM171_CALIB_CONF    0      // how surely the tractor stands still, x 0.01 % (0 = the TM171 decides)
#define TM171_ANSWER_MS     3000   // no report in this time = no answer (the TX5 wire?); the first settings request is sent again once (tmElapsed)
// "New install" calibration (CalibB in ImuAssistant, Calibration object type 0x04, key 0xAA35): the TM171
// takes about 20 s of data with the module still, writes the result to its FLASH and reports success (0x01) or
// failure (0xFF); no "started" report, no stop command. SYD: keep the power on during those 20 s (a power-off
// then may damage the TM171), and switch the TM171 off and on afterwards (on the AiO board: the whole module).
#define TM171_INSTALL_MS    60000  // no result in this time = no answer

// ---------------------------------------------------------------- settings (Setting object 21, AiO_WEB)
// 20 bytes: switches u32 (0), a word at 4 (not known, left as read), UART baud u16 at 6 (x100 bps: 1152),
// a word at 8 (CAN baud?, left as read), accelerometer gain u16 at 10 (x0.01: the knob by GyroErrFilter in
// ImuAssistant), magnetometer gain u16 at 12 (x0.01: the knob by Self-Adapt), inhibit time u16 at 14 (ms, the
// shortest gap between two packets; left as read), a word at 16 (0, left as read), output data rate u16 at
// 18 (Hz: 25 for AgOpenGPS). Checked on a real TM171 on 24 Oct 2026 (the AgOpenGPS set-up read as
// 3B A8 18 00 .. .. 80 04 .. .. 5E 01 64 00 0A 00 00 00 19 00: gains 3.50 / 1.00, inhibit 10 ms, 25 Hz
// measured = the word at 18). Before that the firmware took 4 for a reserved 1152, 14 for the rate and 16-19
// for a silent time, which the real TM171 does not have there: the page said "layout not recognised".
// The switches, bit 0 first (the user guide's C bit fields, little-endian): 0 gyro, 1 accelerometer,
// 2 magnetometer, 3 GyroErrFilter, 4 magnetometer self-adapt filter, 5 status output, 6 raw data output,
// 7 reserved 0, 8 quaternion, 9 reserved 0, 10 Euler, 11 roll-pitch-yaw, 12 gravity, 13 reserved 1,
// 14-16 reserved 2, 17 request acknowledge, 18 save to flash, 19 UART output, 20 reserved, 21 CAN output,
// 22-23 reserved, 24-25 boot mode (0 auto, 1 static, 2 dynamic), 26-31 unused.
// The guide does not number the bits, so before writing anything the firmware checks what it read: bit 13
// = 1, bits 14-16 = 2 (the real TM171 above: 0x0018A83B) and the baud word at 6 = the rate the Teensy is
// talking at. If that does not hold, the page says so (with the raw bytes) and nothing is written. The
// firmware always keeps the gyro, the accelerometer, the roll-pitch-yaw output and the UART output on, the
// baud rate as read (it talks to the TM171 at 115200), and the words it does not know as read.
#define TM171_SET_BIT(w, n)  (((w) >> (n)) & 1UL)
uint8_t  tmSet[TM171_SET_LEN];   // as read from the TM171
bool     tmSetValid  = false;    // read, and the layout check passed
bool     tmSetOdd    = false;    // read, but the layout check failed
uint32_t tmSetMs     = 0;        // when read (0 = never)
uint32_t tmSetReqMs  = 0;        // request sent, no answer yet (0 = none pending)
uint32_t tmSetWriteMs = 0;       // our write, to read the settings back after it
uint32_t tmSetNoAnswer = 0;      // requests without an answer (the TX5 wire?)
uint32_t tmBaudSwitchMs = 0;     // a write changed the TM171's baud rate: Serial5 follows after this time
uint32_t tmBaudNew = 0;
uint32_t TM171rpyRate = 0;       // roll-pitch-yaw packets per second

static uint32_t tmSetU32(const uint8_t* p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t tmSetU16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void     tmSetPut16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void     tmSetPut32(uint8_t* p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

// ---------------------------------------------------------------- set-up check (AiO_WEB)
// Nothing at 115200 at start-up: look for the TM171 at its other baud rates (zImu.ino, imuSetup). Found:
// Serial5 stays at that rate, so the TM171 works, and the page warns.
#define TM171_SCAN_MS 400
bool tm171ScanBaud()
{
    static const uint32_t rates[] = { 460800, 230400, 921600, 57600, 38400, 9600 };
    for (uint8_t k = 0; k < sizeof(rates) / sizeof(rates[0]); k++)
    {
        SerialImu->begin(rates[k]);
        while (SerialImu->available()) SerialImu->read();
        parseState = WAIT_HEADER_1; TM171goodPackets = 0; TM171DataSeen = false;
        const uint32_t t0 = millis();
        while (millis() - t0 < TM171_SCAN_MS)
        {
            TM171process();
            if (TM171goodPackets > 0) { tm171Baud = rates[k]; return true; }
            delay(1);
        }
    }
    SerialImu->begin(115200);
    tm171Baud = 115200;
    parseState = WAIT_HEADER_1; TM171goodPackets = 0; TM171DataSeen = false;
    return false;
}

// The AgOpenGPS set-up (the ImuAssistant settings from the AgOpenGPS forum): magnetometer off, GyroErrFilter
// on with the accelerometer gain 3.50, Self-Adapt on with the magnetometer gain 1.00, roll-pitch-yaw and
// status output, UART on at 115200, 25 Hz, Auto boot mode.
#define TM171_AOG_ACC      350
#define TM171_AOG_MAG      100
#define TM171_AOG_ODR      25
static const TmSetForm TM171_AOG_FORM = { false, true, true, true, true, 0, TM171_AOG_ACC, TM171_AOG_MAG, TM171_AOG_ODR, 1152 };

uint32_t tmFixMs = 0;   // "Fix TM171 setup" pressed (0 = never)

// "" when the TM171 is set up for AgOpenGPS, else what differs (for the page, the log and the program)
const char* tm171SetupWhy()
{
    static char why[120];
    why[0] = '\0';
    if (imuType != IMU_TM171) return why;
    auto add = [&](const char* s) { if (why[0]) strlcat(why, ", ", sizeof(why)); strlcat(why, s, sizeof(why)); };
    if (tm171Baud != 115200) { char b[24]; snprintf(b, sizeof(b), "%lu baud", (unsigned long)tm171Baud); add(b); }
    if (!tmSetValid) return why;
    const uint32_t w = tmSetU32(tmSet);
    if (TM171_SET_BIT(w, 2))  add("magnetometer on");
    if (!TM171_SET_BIT(w, 3)) add("GyroErrFilter off");
    if (!TM171_SET_BIT(w, 4)) add("Self-Adapt off");
    if (!TM171_SET_BIT(w, 5)) add("status output off");
    if (!TM171_SET_BIT(w, 11)) add("RPY output off");
    if (((w >> 24) & 3) != 0) add("boot mode not Auto");
    if (tmSetU16(tmSet + 6) != 1152) add("UART not 115200");
    if (tmSetU16(tmSet + 10) != TM171_AOG_ACC) add("accelerometer gain");
    if (tmSetU16(tmSet + 12) != TM171_AOG_MAG) add("magnetometer gain");
    if (tmSetU16(tmSet + 18) != TM171_AOG_ODR) add("output rate");
    return why;
}

// The "Fix TM171 setup" button: the AgOpenGPS set-up and 115200 baud, saved to the TM171's flash. Serial5
// follows the new baud rate 300 ms later (tm171SettingLoop) and the settings are read back.
bool tm171FixSetup(const char** why)
{
    if (!tm171SettingWrite(TM171_AOG_FORM, why)) return false;
    tmFixMs = millis() | 1;
    return true;
}

// No reply ever to a settings request: the TM171's RX pin is not wired to Teensy TX5 (the web page explains)
bool tm171NoWire() { return tmSetNoAnswer > 1 && tmSetMs == 0; }   // (two requests unanswered: the first is sent again once)

// A timer set with millis() | 1 can be one millisecond ahead of the loop's "now" (the request is sent in the
// same loop pass, or by the web server between two passes): a plain unsigned "now - timer" then wraps to
// four billion and the timer looks expired at once. Signed, it is -1: not expired.
static bool tmElapsed(uint32_t now, uint32_t since, uint32_t ms) { return (int32_t)(now - since) > (int32_t)ms; }

// Ask the TM171 for its Setting object (Request object 12)
void tm171SettingRead()
{
    const uint8_t c[4] = { 21, 0, 0, 0 };
    tm171Send(12, c, sizeof(c));
    tmSetReqMs = millis() | 1;
}

static void tm171SettingGot(const uint8_t* p)
{
    memcpy(tmSet, p, TM171_SET_LEN);
    const uint32_t w = tmSetU32(tmSet);
    const bool layoutOk = TM171_SET_BIT(w, 13) == 1 && ((w >> 14) & 7UL) == 2 && tmSetU16(tmSet + 6) == tm171Baud / 100;
    const bool first = tmSetMs == 0;
    tmSetMs = millis() | 1;
    tmSetReqMs = 0;
    tmSetValid = layoutOk;
    tmSetOdd   = !layoutOk;
    if (first || !layoutOk)
    {
        char raw[3 * TM171_SET_LEN + 1];
        tm171SettingHex(raw);
        StatusLog.line("[status] TM171: settings read (switches 0x%08lX, UART %u00 bps, inhibit %u ms, %u Hz; bytes %s)%s", (unsigned long)w,
                       (unsigned)tmSetU16(tmSet + 6), (unsigned)tmSetU16(tmSet + 14), (unsigned)tmSetU16(tmSet + 18), raw,
                       layoutOk ? "" : " - LAYOUT NOT RECOGNISED, writing from the page is disabled");
    }
}

// the 20 bytes as read, "3B A8 18 00 ..." (for the log and the page: the next unknown layout shows itself)
static void tm171SettingHex(char* out)
{
    for (uint8_t i = 0; i < TM171_SET_LEN; i++) snprintf(out + 3 * i, 4, "%02X ", tmSet[i]);
    out[3 * TM171_SET_LEN - 1] = '\0';
}

// The web page's form. Returns false with the reason.
bool tm171SettingWrite(const TmSetForm& f, const char** why)
{
    if (imuType != IMU_TM171) { *why = "no TM171 found"; return false; }
    if (!tmSetValid)          { *why = tmSetOdd ? "settings layout not recognised; writing is disabled" : "read the settings from the TM171 first"; return false; }
    if (f.accGain < 1 || f.accGain > 2000 || f.magGain < 1 || f.magGain > 2000) { *why = "gain out of range (0.01 - 20.00)"; return false; }
    if (f.odr > 1000 || f.boot > 2) { *why = "value out of range"; return false; }
    uint8_t s[TM171_SET_LEN];
    memcpy(s, tmSet, TM171_SET_LEN);
    uint32_t w = tmSetU32(s);
    auto setBit = [&](uint8_t n, bool v) { if (v) w |= (1UL << n); else w &= ~(1UL << n); };
    setBit(0, true); setBit(1, true);            // gyro, accelerometer: always
    setBit(2, f.mag);
    setBit(3, f.gyroErr);
    setBit(4, f.selfAdapt);
    setBit(5, f.status);
    setBit(11, true);                            // roll-pitch-yaw: the firmware reads it
    setBit(17, true);                            // acknowledge our write
    setBit(18, f.save);
    setBit(19, true);                            // UART output: never off (the Teensy would lose the TM171)
    w = (w & ~(3UL << 24)) | ((uint32_t)f.boot << 24);
    tmSetPut32(s, w);
    if (f.baud100) tmSetPut16(s + 6, f.baud100);     // 0 = keep the baud rate as read
    tmSetPut16(s + 10, f.accGain);
    tmSetPut16(s + 12, f.magGain);
    tmSetPut16(s + 18, f.odr);                       // the words at 4, 8, 14 (inhibit time) and 16 stay as read
    tm171Send(21, s, TM171_SET_LEN);
    SerialImu->flush();                              // sent before Serial5 may change its baud rate
    tmSetAckMs = 0;
    tmSetWriteMs = millis() | 1;
    if (f.baud100 && (uint32_t)f.baud100 * 100UL != tm171Baud) { tmBaudNew = (uint32_t)f.baud100 * 100UL; tmBaudSwitchMs = tmSetWriteMs; }
    StatusLog.line("[status] TM171: settings written (switches 0x%08lX, gains %u/%u, %u Hz%s)", (unsigned long)w,
                   (unsigned)f.accGain, (unsigned)f.magGain, (unsigned)f.odr, f.save ? ", saved to flash" : ", this session only");
    return true;
}

// From imuProcess() (zImu.ino), with a TM171: read the settings once at the start, read them back after
// a write, give up on a request without an answer, count the output rate
void tm171SettingLoop()
{
    static uint32_t firstMs = 0, rateMs = 0;
    static bool asked = false;
    const uint32_t now = millis();
    if (!firstMs) { firstMs = now | 1; rateMs = now; }
    if (!asked && now - firstMs > 2000) { asked = true; tm171SettingRead(); }
    if (tmBaudSwitchMs && tmElapsed(now, tmBaudSwitchMs, 300))
    {
        tmBaudSwitchMs = 0;
        tm171Baud = tmBaudNew;
        SerialImu->begin(tm171Baud);
        while (SerialImu->available()) SerialImu->read();
        parseState = WAIT_HEADER_1;
        StatusLog.line("[status] TM171: Serial5 now at %lu baud", (unsigned long)tm171Baud);
    }
    if (tmSetWriteMs && tmElapsed(now, tmSetWriteMs, 500)) { tmSetWriteMs = 0; tm171SettingRead(); }
    // Set-up warning on the program's screen (PGN 221): once a minute while something differs
    static uint32_t warnMs = 0;
    static bool warned = false;
    const char* why = tm171SetupWhy();
    if (why[0] && !tmSetWriteMs && !tmBaudSwitchMs && now - firstMs > 10000 && (!warned || now - warnMs > 60000))
    {
        warnMs = now;
        if (!warned) StatusLog.line("[status] TM171: not set up for AgOpenGPS (%s) - IMU tab, Fix TM171 setup", why);
        warned = true;
        hwMessage("TM171 not set up for AgOpenGPS: module page, IMU tab, Fix TM171 setup", 10, true);
    }
    else if (!why[0] && warned)
    {
        warned = false;
        StatusLog.println("[status] TM171: set up for AgOpenGPS");
    }
    if (tmSetReqMs && tmElapsed(now, tmSetReqMs, TM171_ANSWER_MS))
    {
        tmSetReqMs = 0;
        if (tmSetNoAnswer++ == 0 && !tmSetMs) { tm171SettingRead(); return; }   // once more before "not wired" is concluded
        if (tmSetNoAnswer == 2 && !tmSetMs)
        {
            StatusLog.println("[status] TM171: no reply - its RX pin is not wired to Teensy TX5 (pin 20). Roll, pitch and yaw work; calibration and settings from the page do not");
            hwMessage("TM171 RX not wired to Teensy TX5: calibration and setup from the page unavailable", 10, true);
        }
    }
    if (now - rateMs >= 1000) { rateMs = now; TM171rpyRate = TM171rpyCount; TM171rpyCount = 0; }
}

// For the web page: the settings as read, decoded
void tm171SettingJson(Json& j)
{
    const uint32_t now = millis();
    j.obj("set");
    j.bo("valid", tmSetValid);
    j.bo("odd", tmSetOdd);
    j.i("age", tmSetMs ? (long)((now - tmSetMs) / 1000) : -1);
    j.bo("pending", tmSetReqMs != 0 || tmSetWriteMs != 0);
    j.i("ackAge", tmSetAckMs ? (long)((now - tmSetAckMs) / 1000) : -1);
    j.u("noAnswer", tmSetNoAnswer);
    j.u("rate", TM171rpyRate);
    j.u("baudNow", tm171Baud);
    j.bo("noWire", tm171NoWire());
    j.str("setupWhy", tm171SetupWhy());
    j.i("fixAgo", tmFixMs ? (long)((now - tmFixMs) / 1000) : -1);
    if (tmSetMs)
    {
        const uint32_t w = tmSetU32(tmSet);
        j.u("sw", w);
        j.bo("gyro", TM171_SET_BIT(w, 0)); j.bo("acc", TM171_SET_BIT(w, 1)); j.bo("mag", TM171_SET_BIT(w, 2));
        j.bo("gyroErr", TM171_SET_BIT(w, 3)); j.bo("selfAdapt", TM171_SET_BIT(w, 4));
        j.bo("status", TM171_SET_BIT(w, 5)); j.bo("raw", TM171_SET_BIT(w, 6)); j.bo("quat", TM171_SET_BIT(w, 8));
        j.bo("euler", TM171_SET_BIT(w, 10)); j.bo("rpy", TM171_SET_BIT(w, 11)); j.bo("gravity", TM171_SET_BIT(w, 12));
        j.bo("uart", TM171_SET_BIT(w, 19)); j.bo("can", TM171_SET_BIT(w, 21));
        j.u("boot", (w >> 24) & 3);
        j.u("baud", (unsigned long)tmSetU16(tmSet + 6) * 100UL);
        j.u("accGain", tmSetU16(tmSet + 10)); j.u("magGain", tmSetU16(tmSet + 12));
        j.u("inhibit", tmSetU16(tmSet + 14)); j.u("odr", tmSetU16(tmSet + 18));
        j.u("w4", tmSetU16(tmSet + 4)); j.u("w8", tmSetU16(tmSet + 8)); j.u("w16", tmSetU16(tmSet + 16));
        char raw[3 * TM171_SET_LEN + 1];
        tm171SettingHex(raw);
        j.str("raw", raw);
    }
    j.close();
}

uint8_t  tmCalibState  = TMC_IDLE;
uint8_t  tmCalibKind   = TMC_KIND_GYRO;   // what the state is about: gyro (RAM) or new install (flash)
uint32_t tmCalibMs     = 0;        // when the state was entered
uint32_t tmCalibDoneMs = 0;        // last result (0 = none yet)
uint32_t tmCalibCount  = 0;

static void tm171CalibCmd(uint8_t ctrl, uint32_t p2, uint32_t p3)
{
    const uint8_t c[12] = { 0x05, ctrl, 0x65, 0xAA,
                            (uint8_t)p2, (uint8_t)(p2 >> 8), (uint8_t)(p2 >> 16), (uint8_t)(p2 >> 24),
                            (uint8_t)p3, (uint8_t)(p3 >> 8), (uint8_t)(p3 >> 16), (uint8_t)(p3 >> 24) };
    tm171Send(23, c, sizeof(c));
}

static bool tmCalibBusy() { return tmCalibState == TMC_STARTING || tmCalibState == TMC_RUNNING || tmCalibState == TMC_FINISHING; }

const char* tmCalibName(uint8_t s)      // for the web page
{
    switch (s)
    {
        case TMC_STARTING:  return "starting";
        case TMC_RUNNING:   return "running";
        case TMC_FINISHING: return "finishing";
        case TMC_OK:        return "ok";
        case TMC_FAILED:    return "failed";
        case TMC_NO_ANSWER: return "noanswer";
        case TMC_ABORTED:   return "aborted";
        default:            return "";
    }
}

// Web page buttons. False with the reason when it cannot start.
bool tm171CalibStart(uint8_t kind, const char** why)
{
    if (imuType != IMU_TM171) { *why = "no TM171 found"; return false; }
    if (tmCalibBusy())        { *why = "a calibration is already in progress"; return false; }
    tmCalibCount++;
    tmCalibKind = kind;
    tmCalibMs   = millis();
    if (kind == TMC_KIND_INSTALL)
    {
        const uint8_t c[12] = { 0x04, 0x02, 0x35, 0xAA, 0,0,0,0, 0,0,0,0 };
        tm171Send(23, c, sizeof(c));
        tmCalibState = TMC_RUNNING;                 // the TM171 sends no "started": only the result
        StatusLog.line("[status] TM171: new install calibration started (about 20 s; keep the tractor parked, do not cut power)");
    }
    else
    {
        tmCalibState = TMC_STARTING;
        tm171CalibCmd(0x03, TM171_CALIB_LIMIT, TM171_CALIB_CONF);
        StatusLog.line("[status] TM171: gyro calibration started (%u s; keep the tractor parked, the engine can stay running)", (unsigned)TM171_CALIB_S);
    }
    return true;
}

// Web page button, or autosteer on / the tractor moving: finish now (the TM171 decides what it can use).
// The new install calibration has no stop command: it goes on in the TM171 (it fails if the tractor moves).
void tm171CalibStop(const char* why)
{
    if (!tmCalibBusy()) return;
    if (tmCalibKind == TMC_KIND_INSTALL)
    {
        StatusLog.line("[status] TM171: new install calibration cannot be stopped (%s), waiting for the result", why);
        return;
    }
    tm171CalibCmd(0x04, 0, 0);
    tmCalibState  = TMC_ABORTED;
    tmCalibMs     = millis();
    tmCalibDoneMs = tmCalibMs | 1;
    StatusLog.line("[status] TM171: gyro calibration stopped (%s)", why);
}

// The TM171's report (Calibration object: type 0x15 = forced gyro calibration, 0x14 = new install)
void tm171CalibReport(uint8_t type, uint8_t ctrl)
{
    const uint32_t now = millis();
    if (type == 0x15 && tmCalibKind == TMC_KIND_GYRO)
    {
        if (ctrl == 0x00 && tmCalibState == TMC_STARTING) { tmCalibState = TMC_RUNNING; tmCalibMs = now; return; }
        if (!tmCalibBusy()) return;
        if (ctrl == 0x01)
        {
            tmCalibState = TMC_OK; tmCalibMs = now; tmCalibDoneMs = now | 1;
            StatusLog.line("[status] TM171: gyro calibration done (kept until the TM171 is powered off)");
        }
        else if (ctrl == 0xFF)
        {
            tmCalibState = TMC_FAILED; tmCalibMs = now; tmCalibDoneMs = now | 1;
            StatusLog.line("[status] TM171: gyro calibration FAILED (the tractor moved, or too much vibration); previous values kept");
        }
    }
    else if (type == 0x14 && tmCalibKind == TMC_KIND_INSTALL && tmCalibBusy())
    {
        if (ctrl == 0x01)
        {
            tmCalibState = TMC_OK; tmCalibMs = now; tmCalibDoneMs = now | 1;
            StatusLog.line("[status] TM171: new install calibration saved to flash; switch the whole module off and on");
        }
        else if (ctrl == 0xFF)
        {
            tmCalibState = TMC_FAILED; tmCalibMs = now; tmCalibDoneMs = now | 1;
            StatusLog.line("[status] TM171: new install calibration FAILED (did the tractor move?); switch the module off and on and try again");
        }
    }
}

// From imuProcess() (zImu.ino), with a TM171
void tm171CalibLoop()
{
    if (!tmCalibBusy()) return;
    const uint32_t now = millis();
    if (tmCalibKind == TMC_KIND_INSTALL)        // nothing to send: wait for the result
    {
        if (now - tmCalibMs > TM171_INSTALL_MS)
        {
            tmCalibState = TMC_NO_ANSWER; tmCalibMs = now; tmCalibDoneMs = now | 1;
            StatusLog.line("[status] TM171: no result from the new install calibration - is Teensy TX5 (pin 20) wired to the TM171 RX?");
        }
        return;
    }
    if (watchdogTimer < WATCHDOG_THRESHOLD) { tm171CalibStop("autosteer engaged"); return; }
    if (gpsSpeed > 1.0f)                    { tm171CalibStop("the tractor is moving"); return; }
    switch (tmCalibState)
    {
        case TMC_STARTING:
            if (now - tmCalibMs > TM171_ANSWER_MS)
            {
                tmCalibState = TMC_NO_ANSWER; tmCalibMs = now; tmCalibDoneMs = now | 1;
                StatusLog.line("[status] TM171: no reply to the calibration command - is Teensy TX5 (pin 20) wired to the TM171 RX?");
            }
            break;
        case TMC_RUNNING:
            if (now - tmCalibMs >= (uint32_t)TM171_CALIB_S * 1000UL)
            {
                tm171CalibCmd(0x04, 0, 0);
                tmCalibState = TMC_FINISHING; tmCalibMs = now;
            }
            break;
        case TMC_FINISHING:
            if (now - tmCalibMs > TM171_ANSWER_MS)
            {
                tmCalibState = TMC_NO_ANSWER; tmCalibMs = now; tmCalibDoneMs = now | 1;
                StatusLog.line("[status] TM171: no reply after the calibration; result unknown");
            }
            break;
        default: break;
    }
}

// Seconds of data collection left (web page), -1 when not running
int tm171CalibLeft()
{
    if (tmCalibState != TMC_RUNNING || tmCalibKind != TMC_KIND_GYRO) return -1;
    const uint32_t el = (millis() - tmCalibMs) / 1000UL;
    return el >= TM171_CALIB_S ? 0 : (int)(TM171_CALIB_S - el);
}

bool TM171detectOnPort(TeensySerial* port, uint32_t detectionMs)
{
    SerialImu = port;
    TM171setup();

    TM171DataSeen = false;
    TM171lastData = 0;
    parseState = WAIT_HEADER_1;
    packetLength = 0;
    payloadIndex = 0;
    gotPacket = false;

    while (SerialImu->available())
    {
        SerialImu->read();
    }

    delay(200);

    uint32_t start = millis();
    while (millis() - start < detectionMs)
    {
        TM171process();
        if (TM171DataSeen)
        {
            return true;
        }
        delay(1);
    }

    return false;
}

bool GoodCRC(byte Data[], byte Length)
{
	uint16_t ck = MODBUS_CRC16_v3(Data,Length-2);
	bool Result = (ck == Data[Length - 2] + (Data[Length - 1]<<8));
	return Result;
}

static uint16_t MODBUS_CRC16_v3( const unsigned char *buf, unsigned int len )
{
	static const uint16_t table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

	uint8_t tor = 0;
	uint16_t crc = 0xFFFF;
  
  //mod for not counting first to bytes
  buf++;   // buf++ instead of *buf++ (same result, no compiler warning)
  buf++;
  len = len -2;


	while( len-- )
	{
		tor = (*buf++) ^ crc;
		crc >>= 8;
		crc ^= table[tor];
	}

	return crc;
}