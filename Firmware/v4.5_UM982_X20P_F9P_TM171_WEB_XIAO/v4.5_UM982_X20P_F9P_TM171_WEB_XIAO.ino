/* v4.5_UM982_X20P_F9P_TM171_WEB_XIAO.ino - setup and loop (main file).
   Part of UM982_X20P_F9P_TM171_WEB_XIAO, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Based on the code of:
     - chriskinal: AOG_Teensy_UM982 (github.com/chriskinal/AOG_Teensy_UM982), Copyright 2024 chriskinal, GPL v3
     - Balázs Gunics: fork of the AgOpenGPS boards (github.com/gunicsba/agopengps_boards), firmware V4.1
       (TM171)
     - the AgOpenGPS project: AiO v4 Teensy firmware, on which both are based; its Keya CAN code
       (AIO_v4_Firmware_Keya_CANBUS_HPR_Dual_27.04.2025)
   Changed and extended 2026 by Claude (Anthropic) for renatorgr: UM982 / X20P / F9P detection, TM171
   heading, standard and classic output, automatic mode, web page, update over the network, XIAO link,
   autosteer safety and watchdog, receiver identification, u-blox and UM982 settings, BNO085 and
   the AiO firmware's Keya.
   Original notice (chriskinal), kept as it was:
     Copywrite 2024 chriskinal
     This program is free software: you can redistribute it and/or modify it under the terms of the GNU General
     Public License as published by the Free Software Foundation, either version 3 of the License, or (at your
     option) any later version. This program is distributed in the hope that it will be useful, but WITHOUT ANY
     WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
     GNU General Public License for more details. You should have received a copy of the GNU General Public
     License along with this program. If not, see <https://www.gnu.org/licenses/>.
   Licence: GNU General Public License v3 or later (file COPYING), WITHOUT ANY WARRANTY.
   SPDX-License-Identifier: GPL-3.0-or-later
*/
// v4.5_UM982_X20P_F9P_TM171_WEB_XIAO.ino  (version number: zVersion.h)
// What 4.5 does:
//   - one firmware for the UM982 (dual antenna) and the u-blox F9P / X20P (single antenna), told
//     apart by what the receiver sends; the IMU heading (TM171 or BNO085) aligned to the antennas or
//     to the GPS course (zHandlers.ino, zImu.ino);
//   - output: the standard set for AgOpenWeb, the classic $PAOGI / $PANDA for AgOpenGPS, or automatic
//     by the hello each program sends once a second (zHandlers.ino, zStatus.ino);
//   - a web page: status, messages, settings, the GPS receiver (its model, the u-blox and UM982
//     settings, the UM982 console), firmware update over the network or from GitHub, Portuguese /
//     English (zWeb.ino, zWebPage.h, zSettings.ino, zOTA.ino, zReceiver.ino);
//   - a bridge to a Seeed XIAO ESP32C3 / ESP32S3 on the GPS2 port (Serial2, 460800), with the protocol
//     of the ESP32 bridge of the official firmware v26: AgOpenGPS packets from UDP 8888 go to the
//     XIAO, its PGNs go to UDP 9999; it also gets the module status and messages for its WiFi page
//     (zXiao.ino);
//   - autosteer safety: WAS voltage check, autosteer off at once, why it went off on the program's
//     screen (PGN 221), hardware watchdog, why the Teensy started (zSafety.ino); the RTCM corrections
//     counted (zRtcm.h);
//   - the Keya code of the AgOpenGPS AiO v4 firmware 27.04.2025 (KeyaCANBUS.ino); PWM at 122 Hz and
//     LOW_HIGH_DEGREES 1.5, as that firmware (Autosteer.ino).
// Build for a fixed hardware configuration:
//   GPS/heading : UM982 dual antenna (GGA + VTG + HPR on Serial7, 10 Hz), or
//                 u-blox F9P / X20P single antenna (GGA + VTG) - detected automatically.
//   IMU         : TM171 (Serial5), or BNO085 on Serial5 in RVC mode or on I2C - found at start-up.
//   Output      : per GGA epoch to AgOpenWeb / AgOpenGPS (UDP 9999), see zHandlers.ino.
//                 Standard set (AgOpenWeb with the epoch assembler, PR #300):
//                   $GNGGA  position          $GNVTG  speed
//                   $GNTHS  dual-antenna heading (UM982 HPR), V when there is none
//                   $INHPR  TM171 heading + roll (IMU talker)
//                 Classic (AgOpenGPS / AgIO, AgOpenWeb): $PAOGI or $PANDA
//                 Automatic (default): standard for AgOpenWeb, classic for AgOpenGPS (by its hello)
//   Web page    : http://<module IP>/  (normally http://192.168.5.126/)
//   Steering    : Cytron MD13S (hydraulic valve) or Keya CAN motor,
//                 found by its heartbeat (at boot or later, KeyaCANBUS.ino)
//   RTK link    : XBee LR radio on Serial3
//   XIAO        : optional XIAO ESP32C3 / ESP32S3 on Serial2 (GPS2 socket, 460800), detected by its
//                 "ESP32-hello"; it can be switched off on the web page (Definições)
//
// This is not a general-purpose AiO firmware (the AiO board takes many more options): there is no
// CMPS, no second GPS (the GPS2 socket holds the XIAO). What is sent to the program follows whether
// the receiver sends HPR (UM982 dual antenna) or not (F9P / X20P); the receiver is also asked its
// model, for the web page and the u-blox / UM982 settings.
//
// Connections:
//   Serial7 RX (pin 28) <- receiver TX  (GGA, VTG, HPR)
//   Serial7 TX (pin 29) -> receiver RX  (RTCM corrections in)
//   Serial5              <- TM171 IMU TX, or BNO085 TX in RVC mode (115200)
//   Wire (pins 18 / 19)  <-> BNO085 on I2C (only if nothing on Serial5)
//   Serial3              <- XBee LR radio TX (RTCM at 115200)
//   Serial2 RX (pin 7)   <- XIAO D6 (TX; C3 GPIO21, S3 GPIO43) - Micro: GPS2 socket pin 2; Standard: P8 pin 1
//   Serial2 TX (pin 8)   -> XIAO D7 (RX; C3 GPIO20, S3 GPIO44) - Micro: GPS2 socket pin 3; Standard: P8 pin 2
// The AiO v4.5 Micro and Standard wire all these Teensy pins the same way.
//

/************************* User Settings *************************/
#define SerialAOG Serial                //AgIO USB connection
#define SerialRTK Serial3               //RTK radio (XBee LR board)
// TeensySerial: the type of Serial1..Serial8. Teensyduino 1.59 split HardwareSerial in two and moved
// addMemoryForRead / addMemoryForWrite to HardwareSerialIMXRT, so a HardwareSerial* no longer
// compiles there; this builds with 1.58 and with 1.59 or newer.
#if defined(TEENSYDUINO) && TEENSYDUINO >= 159
typedef HardwareSerialIMXRT TeensySerial;
#else
typedef HardwareSerial TeensySerial;
#endif
TeensySerial* SerialGPS = &Serial7;     //UM982 (GGA + VTG + HPR) or F9P / X20P (GGA + VTG)

const int32_t baudAOG = 115200;
const int32_t baudGPS = 460800;         // must match the receiver port baud rate (UM982 COM, u-blox UART1)
const int32_t baudRTK = 115200;         // XBee LR

// Status LEDs
#define GGAReceivedLED 13
#define Power_on_LED 5
#define Ethernet_Active_LED 6
#define GPSRED_LED 9                    // Red   (ON = single antenna / TM171 heading, blinking = TM171 lost)
#define GPSGREEN_LED 10                 // Green (ON = UM982 dual heading good)
#define AUTOSTEER_STANDBY_LED 11
#define AUTOSTEER_ACTIVE_LED 12
uint32_t gpsReadyTime = 0;
/*****************************************************************/

// Ethernet
#ifdef ARDUINO_TEENSY41
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

struct ConfigIP {
    uint8_t ipOne   = 192;
    uint8_t ipTwo   = 168;
    uint8_t ipThree = 5;
}; ConfigIP networkAddress;

byte Eth_myip[4] = {0, 0, 0, 0};
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

unsigned int portMy           = 5120;
unsigned int AOGNtripPort     = 2233;
unsigned int AOGAutoSteerPort = 8888;
unsigned int portDestination  = 9999;
char Eth_NTRIP_packetBuffer[512];

EthernetUDP Eth_udpPAOGI;
EthernetUDP Eth_udpNtrip;
EthernetUDP Eth_udpAutoSteer;

IPAddress Eth_ipDestination;
#endif

// Serial buffers
constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];
uint8_t GPStxbuffer[serial_buffer_size];
uint8_t RTKrxbuffer[serial_buffer_size];

// Speed pulse
elapsedMillis speedPulseUpdateTimer = 0;
byte velocityPWM_Pin = 36;

#include "zNMEAParser.h"
#include "zSettings.h"                  // web page settings (cfg), see zSettings.ino
#include "zStatusLog.h"                 // [status] log for USB + web page, see zStatus.ino
#include "zWebJson.h"                   // JSON writer for the web page (zWeb.ino)
#include "zXiao.h"                      // XIAO ESP32C3 bridge state (zXiao.ino)
#include "zSafety.h"                    // autosteer safety, watchdog, start reason (zSafety.ino)
#include "zRtcm.h"                      // RTCM messages counted (radio, NTRIP), zStatus.ino
#include "zReceiver.h"                  // receiver identification, u-blox settings (zReceiver.ino)
#include "zImu.h"                       // TM171 or BNO085 (zImu.ino)
#include "zVersion.h"                    // FW_NAME, FW_VERSION
#include <Wire.h>

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> Keya_Bus;

extern "C" uint32_t set_arm_clock(uint32_t frequency);

// Keya motor on the CAN bus (KeyaCANBUS.ino, from the AiO firmware 27.04.2025): found by its
// heartbeat, at start-up or later. true = steering by the Keya; false = the valve / motor driver set
// in the program (Cytron, IBT2)
bool isKeya = false;
bool keyaIntendToSteer = false;         // Autosteer.ino: steering now (the Keya gets enable / disable)
// What the Keya said last (KeyaCANBUS.ino; Autosteer.ino, web page). Declared here: Autosteer.ino
// comes before KeyaCANBUS.ino.
int16_t  keyaSteeringPosition   = 0;    // motor position (heartbeat bytes 0-1, sign as the AiO firmware)
int16_t  keyaCurrentActualSpeed = 0;    // motor speed
int16_t  keyaCurrentSetSpeed    = 0;    // the speed it was told (x 0.1)
int16_t  keyaCurrent            = 0;    // motor current (heartbeat bytes 4-5)
float    keyaSlip               = 0;    // the AiO firmware's "sensorReading": how much slower than told (kickout)
uint16_t keyaFaults             = 0;    // byte 6 << 8 | byte 7 (byte 7 bit 0 = disabled, not a fault)
uint32_t keyaHeartbeatMs        = 0;    // last heartbeat (millis; keyaHeartbeats 0 = never)
uint32_t keyaHeartbeats         = 0;

// Wheel angle from the GPS heading (zHandlers.ino), for the Keya without a WAS (Autosteer.ino), as
// the AgOpenGPS AiO v4 firmware
#define WHEELBASE_M     3.20            // metres (the AiO firmware's wheelBase)
float   wheelAngleGPS   = 0;            // degrees
bool    wheelAngleGPSOk = false;        // moving above 1 km/h with a heading that turns believably
float   headingRateDps  = 0;            // degrees / second
int8_t  workingDir      = 1;            // 1 forward, -1 reverse

elapsedMillis TM171lastData;

/* A parser with 3 handlers (GGA, VTG, HPR) */
NMEAParser<3> parser;

bool blink = false;

bool Autosteer_running = true;
bool Ethernet_running  = false;

// Pass-through for GPS config tool
bool passThroughGPS = false;

// AOG serial command detection (!AOGR1 / !AOGED)
uint8_t aogSerialCmd[4]       = {'!', 'A', 'O', 'G'};
uint8_t aogSerialCmdBuffer[6];
uint8_t aogSerialCmdCounter   = 0;


// ----- Setup -----
void setup()
{
    bootCapture();                      // why the Teensy started, before anything clears it (zSafety.ino)
    delay(500);

    pinMode(GGAReceivedLED,        OUTPUT);
    pinMode(Power_on_LED,          OUTPUT);
    pinMode(Ethernet_Active_LED,   OUTPUT);
    pinMode(GPSRED_LED,            OUTPUT);
    pinMode(GPSGREEN_LED,          OUTPUT);
    pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
    pinMode(AUTOSTEER_ACTIVE_LED,  OUTPUT);

    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA",  GGA_Handler);   // $GNGGA / $GPGGA
    parser.addHandler("G-VTG",  VTG_Handler);   // $GNVTG / $GPVTG
    parser.addHandler("G-HPR",  HPR_Handler);   // $GNHPR / $GPHPR (UM982 dual-antenna heading)

    delay(10);
    Serial.begin(baudAOG);
    delay(10);
    Serial.println("Start " FW_NAME " setup");
    settingsLoad();                     // web page settings from the EEPROM (defaults if none)
    StatusLog.line("[boot] " FW_NAME ", settings %s, output %s%s, XIAO bridge %s",
                   cfgFromEeprom ? "from EEPROM" : "default",
                   cfg.outputMode == OUT_AUTO ? "automatic (by the program's hello; classic until then)" :
                   cfg.outputMode == OUT_CLASSIC ? "classic $PAOGI/$PANDA" : "standard set",
                   settingsAutoSetNow ? " (set to automatic by this version, once; can be changed on the web page)" : "",
                   cfg.xiaoOff ? "off" : "on");
    bootReport();                       // power on / restart / watchdog / crash (zSafety.ino)
    if (cfg.wasCheckOff) StatusLog.line("[boot] WAS check OFF (bench setting): a broken WAS wire does not stop autosteer");

    Serial.println("GNSS receiver on Serial7");
    SerialGPS->begin(baudGPS);
    SerialGPS->addMemoryForRead(GPSrxbuffer,  serial_buffer_size);
    SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

    delay(10);
    SerialRTK.begin(baudRTK);
    SerialRTK.addMemoryForRead(RTKrxbuffer, serial_buffer_size);

    xiaoSetup();                        // XIAO ESP32C3 on Serial2 (zXiao.ino)

    Serial.println("SerialAOG, SerialRTK, SerialGPS, Serial2 (XIAO) initialized");

    Serial.println("\r\nStarting AutoSteer...");
    autosteerSetup();

    Serial.println("\r\nStarting Ethernet...");
    EthernetStart();
    webSetup();                         // web page on port 80 (zWeb.ino)

    Serial.println("\r\nLooking for the IMU (TM171 / BNO085)...");
    // zImu.ino: TM171 or BNO085 (RVC) on Serial5, else BNO085 on I2C. Roll always comes from
    // it; heading whenever there is no valid dual heading (always with an F9P / X20P).
    imuSetup();

    Serial.println("Starting CANBUS...");
    CAN_Setup();

    // Auto-detect Keya motor by listening for its CAN heartbeat (ID 0x07000001).
    // Listen for 2 seconds - Keya heartbeats every 20ms so plenty of time.
    // If no heartbeat seen, assume hydraulic via Cytron.
    Serial.println("Detecting steering type (Keya CAN / Hydraulic)...");
    {
        uint32_t detectStart = millis();
        while (millis() - detectStart < 2000)
        {
            // Keep emptying the IMU serial buffer during this 2 s wait. The TM171 sends
            // ~25 packets/s (a BNO085 in RVC mode 100) and the 512-byte buffer fills in under
            // a second; the bytes dropped when it overflows cut a packet in half (bad CRC at start-up).
            imuProcess();
            xiaoProcess();      // same for the XIAO on Serial2 (its boot lines and first hello)

            KeyaBus_Receive();  // a heartbeat sets isKeya (KeyaCANBUS.ino)
            if (isKeya) break;
        }
    }
    if (!isKeya)
        StatusLog.line("[boot] No Keya heartbeat: steering by the valve / motor driver set in the program (Cytron, IBT2); a Keya found later is used");

    gnssFlushAfterSetup();   // drop the stale GPS data that piled up while setup() was blocking

    wdtStart();              // hardware watchdog, 2 s (zSafety.ino; can be switched off on the web page)

    Serial.println("\r\nEnd setup, waiting for GPS...\r\n");
}

// ----- Main Loop -----
// loopStep says which part runs, for the watchdog report (zSafety.ino, stepName)
void loop()
{
    wdtFeed();                                    // hardware watchdog (zSafety.ino)

    loopStep = 1;
    KeyaBus_Receive();

    // Forward incoming bytes from AgIO/NTRIP to the receiver (RTCM corrections)
    loopStep = 2;
    if (SerialAOG.available())
    {
        uint8_t incoming_char = SerialAOG.read();

        // Handle !AOGRx / !AOGED config commands from AgIO config utility
        if (aogSerialCmdCounter < 4 && aogSerialCmd[aogSerialCmdCounter] == incoming_char)
        {
            aogSerialCmdBuffer[aogSerialCmdCounter] = incoming_char;
            aogSerialCmdCounter++;
        }
        else if (aogSerialCmdCounter == 4)
        {
            aogSerialCmdBuffer[aogSerialCmdCounter]     = incoming_char;
            aogSerialCmdBuffer[aogSerialCmdCounter + 1] = SerialAOG.read();

            if (aogSerialCmdBuffer[aogSerialCmdCounter] == 'R' &&
                aogSerialCmdBuffer[aogSerialCmdCounter + 1] == '1')
            {
                // !AOGR1 - verify NMEA GGA at fixed 460800 baud
                passThroughGPS  = true;
                bool found = false;

                Serial.print("Checking for NMEA at "); Serial.println(baudGPS);
                SerialGPS->begin(baudGPS);
                delay(300);
                wdtFeed();
                while (SerialGPS->available()) SerialGPS->read(); // flush

                static uint8_t gi = 0;
                const char* pat = "GGA,";            // $GNGGA or $GPGGA
                uint32_t t = millis();
                while (millis() - t < 800)
                {
                    wdtFeed();
                    if (SerialGPS->available())
                    {
                        char c = SerialGPS->read();
                        if (c == pat[gi]) { gi++; if (gi == 4) { gi = 0; found = true; break; } }
                        else gi = (c == pat[0]) ? 1 : 0;
                    }
                }

                if (found)
                {
                    SerialAOG.write(aogSerialCmdBuffer, 6);
                    SerialAOG.print("Found NMEA at baudrate: ");
                    SerialAOG.println(baudGPS);
                    SerialAOG.println("!AOGOK");
                }
                else
                {
                    SerialAOG.println("GNSS receiver not detected at 460800. Check for faults.");
                }
            }
            else if (aogSerialCmdBuffer[aogSerialCmdCounter] == 'E' &&
                     aogSerialCmdBuffer[aogSerialCmdCounter + 1] == 'D')
            {
                passThroughGPS  = false;
            }
            aogSerialCmdCounter = 0;
        }
        else
        {
            aogSerialCmdCounter = 0;
        }

        // Forward to GPS (RTCM corrections pass through to the receiver)
        if (!passThroughGPS)
        {
            SerialGPS->write(incoming_char);
        }
    }

    // Read NMEA from the receiver (GGA, VTG, HPR)
    loopStep = 3;
    gnssNoteRxBacklog();                          // [status] warns if bytes pile up (Teensy reading too slowly)
    if (SerialGPS->available())
    {
        if (passThroughGPS)
        {
            SerialAOG.write(SerialGPS->read());
        }
        else
        {
            uint8_t gc = SerialGPS->read();
            if (!ubxFeed(gc))                     // u-blox UBX frames are taken out (zReceiver.ino)
            {
                rawNmeaTee((char)gc);             // [raw] lines on USB when RAW_NMEA_DEBUG = 1
                parser << gc;
            }
        }
    }

    loopStep = 4;
    udpNtrip();

    // Forward RTK radio (XBee LR) corrections to the receiver. what has arrived, at once (was
    // one byte per loop), counted for the web page and the [status] messages (zStatus.ino)
    loopStep = 5;
    {
        int n = SerialRTK.available();
        if (n > 0)
        {
            uint8_t rb[128];
            if (n > (int)sizeof(rb)) n = sizeof(rb);
            for (int k = 0; k < n; k++) rb[k] = (uint8_t)SerialRTK.read();
            SerialGPS->write(rb, n);
            statusNoteRadio(rb, (uint16_t)n);
        }
    }

    // GGA timeout - turn off GPS LEDs after 10 sec with no fix
    if ((systick_millis_count - gpsReadyTime) > 10000)
    {
        digitalWrite(GPSRED_LED,   LOW);
        digitalWrite(GPSGREEN_LED, LOW);
    }

    // IMU (TM171 / BNO085) - roll at all times, heading when single antenna
    loopStep = 6;
    imuProcess();

    // XIAO ESP32C3 on Serial2: its hello, its PGNs to UDP 9999, its text lines to the log (zXiao.ino)
    loopStep = 7;
    xiaoProcess();

    // Per GGA epoch: $GNGGA, $GNVTG, $GNTHS (dual heading), $INHPR (TM171) to AgOpenWeb
    loopStep = 8;
    gnssProcess();

    // [status] messages on the serial monitor when Ethernet / AgOpenWeb / GPS / dual change (zStatus.ino)
    loopStep = 9;
    statusUpdate();

    // Web page (zWeb.ino): one request at a time, never waits long for the browser
    loopStep = 10;
    webLoop();

    loopStep = 11;
    if (Autosteer_running) autosteerLoop();
    else ReceiveUdp();

    // the receiver's model, the u-blox settings from the web page (zReceiver.ino)
    loopStep = 13;
    receiverLoop();

    loopStep = 12;
    if (Ethernet.linkStatus() == LinkOFF)
    {
        digitalWrite(Power_on_LED,        1);
        digitalWrite(Ethernet_Active_LED, 0);
    }
    if (Ethernet.linkStatus() == LinkON)
    {
        digitalWrite(Power_on_LED,        0);
        digitalWrite(Ethernet_Active_LED, 1);
    }
}
