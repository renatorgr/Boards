/* v4.5_AiO_WEB.ino - setup and loop (main file).
   Part of AiO_WEB, Teensy 4.1 firmware for the AiO v4.5 boards
   (renatorgr).
   Based on the code of:
     - the AgOpenGPS project: AiO v4 Teensy firmware (AIO_v4_Firmware_Keya_CANBUS_HPR_Dual_27.04.2025:
       dual F9P RELPOSNED, Keya CAN, WAS map), on which the others are based
     - chriskinal: AOG_Teensy_UM982 (github.com/chriskinal/AOG_Teensy_UM982), Copyright 2024 chriskinal, GPL v3
     - Balázs Gunics: fork of the AgOpenGPS boards (github.com/gunicsba/agopengps_boards), firmware V4.1 (TM171)
   Changed and extended 2026 by Claude (Anthropic) for renatorgr: the UM982_X20P_F9P_TM171_WEB_XIAO
   firmware (UM982 / X20P / F9P detection, IMU heading, standard and classic output, automatic mode,
   web page, update over the network, XIAO link, autosteer safety and watchdog, receiver
   identification and settings, BNO085, the AiO firmware's Keya) and, in this AiO_WEB line, the dual
   F9P on the GPS2 port, the rest of the AiO firmware's steering, the network stack polled from
   loop(), the receiver check (zRxCheck.ino, with the AgOpenGPS receiver settings of AOGConfigOMatic
   by lansalot, zRxConfig.h) and the simulator mode setting.
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
// v4.5_AiO_WEB.ino  (version number: zVersion.h)
// AiO_WEB 4.5: the v4.5_UM982_X20P_F9P_TM171_WEB_XIAO firmware with what the AgOpenGPS AiO v4
// firmware does and that one does not: the dual F9P (second F9P on the GPS2 port, zDualF9P.ino), the
// cut beyond the WAS map or above 25 km/h and the WAS map (Autosteer.ino).
// The GPS2 port is the second F9P's or the XIAO's, chosen on the web page (Settings).
//
//   GPS / heading: UM982 dual antenna (GGA + VTG + HPR on Serial7), or
//                  dual F9P: first F9P on Serial7 (GGA + VTG), second F9P on Serial2 (RELPOSNED), or
//                  single F9P / X20P (GGA + VTG) - told apart by what arrives.
//   IMU          : TM171 or BNO085 (Serial5, RVC mode) or BNO085 on I2C - found at start-up.
//   Output       : per GGA epoch to AgOpenWeb / AgOpenGPS (UDP 9999), see zHandlers.ino.
//                  Standard set (AgOpenWeb with the epoch assembler, PR #300):
//                    $GNGGA  position          $GNVTG  speed
//                    $GNTHS  dual-antenna heading (UM982 HPR or second F9P), V when there is none
//                    $INHPR  IMU heading + roll (IMU talker)
//                  Classic (AgOpenGPS / AgIO, AgOpenWeb): $PAOGI (dual antenna) or $PANDA (single)
//                  Automatic (default): standard for AgOpenWeb, classic for AgOpenGPS (by its hello)
//   Web page     : http://<module IP>/  (normally http://192.168.5.126/)
//   Steering     : Cytron (hydraulic valve), IBT2, Danfoss (as set in the program), or a Keya CAN
//                  motor (found by its heartbeat)
//   RTK link     : XBee radio on Serial3, NTRIP from the program (UDP 2233)
//   GPS2 port    : second F9P or XIAO ESP32C3 / ESP32S3, both at 460800, chosen on the web page
//
// Connections:
//   Serial7 RX (pin 28) <- receiver TX  (GGA, VTG, HPR)
//   Serial7 TX (pin 29) -> receiver RX  (RTCM corrections in)
//   Serial5              <- TM171 IMU TX, or BNO085 TX in RVC mode (115200)
//   Wire (pins 18 / 19)  <-> BNO085 on I2C (only if nothing on Serial5)
//   Serial3              <- XBee radio TX (RTCM at 115200)
//   Serial2 RX (pin 7)   <- second F9P TX (RELPOSNED), or XIAO D6 (TX) - Micro: GPS2 socket pin 2; Standard: P8 pin 1
//   Serial2 TX (pin 8)   -> XIAO D7 (RX) - Micro: GPS2 socket pin 3; Standard: P8 pin 2
//   CAN3 (pins 30 / 31)  <-> Keya motor (250 kbit/s)
// The AiO v4.5 Micro and Standard wire all these Teensy pins the same way.
//

/************************* User Settings *************************/
#define SerialAOG Serial                //AgIO USB connection
#define SerialRTK Serial3               //RTK radio (XBee LR board)
// the type of Serial1..Serial8. Teensyduino 1.59 split HardwareSerial in two and moved
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
#include "zNet.h"                       // the network stack runs from loop(), not from an interrupt

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
uint8_t GPSrxbuffer[2048];                 // the receiver: 2048 so a long loop (about 1 s of a UM982 at 10 Hz) loses nothing
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

// Keya motor on the CAN bus (KeyaCANBUS.ino): found by its heartbeat, at start-up or later (as the
// AiO firmware). true = steering by the Keya; false = Cytron (hydraulic valve), IBT2 or Danfoss,
// as set in the program
bool isKeya = false;
bool keyaIntendToSteer = false;         // Autosteer.ino: steering now (the Keya gets enable / disable)
// What the Keya said last (KeyaCANBUS.ino; Autosteer.ino, web page)
int16_t  keyaSteeringPosition   = 0;    // motor position (heartbeat bytes 0-1, sign as the AiO firmware)
int16_t  keyaCurrentActualSpeed = 0;    // motor speed
int16_t  keyaCurrentSetSpeed    = 0;    // the speed it was told (x 0.1)
int16_t  keyaCurrent            = 0;    // motor current (heartbeat bytes 4-5)
float    keyaSlip               = 0;    // the AiO firmware's "sensorReading": how much slower than told (kickout)
uint16_t keyaFaults             = 0;    // byte 6 << 8 | byte 7 (byte 7 bit 0 = disabled, not a fault)
uint32_t keyaHeartbeatMs        = 0;    // last heartbeat (millis; keyaHeartbeats 0 = never)
uint32_t keyaHeartbeats         = 0;

elapsedMillis TM171lastData;

// AiO_WEB: wheel angle from the GPS heading (zHandlers.ino), for the Keya without a WAS (Autosteer.ino),
// as the AgOpenGPS AiO v4 firmware
#define WHEELBASE_M     3.20            // metres (the AiO firmware's wheelBase)
float   wheelAngleGPS   = 0;            // degrees
bool    wheelAngleGPSOk = false;        // moving above 1 km/h with a heading that turns believably
float   headingRateDps  = 0;            // degrees / second
int8_t  workingDir      = 1;            // 1 forward, -1 reverse

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
    faultHookInstall();                 // a crash switches the steering outputs off at once (zSafety.ino)
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
    // The start-up messages, one per fact, all through the [status] log (USB and the web page's Messages):
    // firmware and build, why it started, settings, autosteer (ADS1115), Ethernet UDP ports, IMU, Keya,
    // watchdog, setup time. The module's IP and the page's address: "[status] Ethernet: link up" (zStatus.ino).
#ifdef TEENSYDUINO
    StatusLog.line("[boot] " FW_NAME ", built " __DATE__ " " __TIME__ " with Teensyduino %d.%02d", TEENSYDUINO / 100, TEENSYDUINO % 100);
#else
    StatusLog.line("[boot] " FW_NAME ", built " __DATE__ " " __TIME__);
#endif
    bootReport();                       // power on / restart / watchdog / crash (zSafety.ino)
    settingsLoad();                     // web page settings from the EEPROM (defaults if none)
    StatusLog.line("[boot] Settings %s: output %s%s, GPS2 port %s",
                   cfgFromEeprom ? "from the EEPROM" : "default (nothing saved yet)",
                   cfg.outputMode == OUT_AUTO ? "automatic (by the program's hello; classic until then)" :
                   cfg.outputMode == OUT_CLASSIC ? "classic $PAOGI/$PANDA" : "standard set",
                   settingsAutoSetNow ? " (set to automatic by this version, once; can be changed on the web page)" : "",
                   gps2Name());
    if (cfg.wasCheckOff) StatusLog.line("[boot] WAS check OFF (bench setting): a broken WAS wire does not stop autosteer");
    if (cfg.simMode)     StatusLog.line("[boot] Simulator mode ON (for the program's simulator only): the GPS is not sent to the program");

    SerialGPS->begin(baudGPS);          // the GNSS receiver
    SerialGPS->addMemoryForRead(GPSrxbuffer,  sizeof(GPSrxbuffer));
    SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

    delay(10);
    SerialRTK.begin(baudRTK);
    SerialRTK.addMemoryForRead(RTKrxbuffer, serial_buffer_size);

    xiaoSetup();                        // GPS2 port, Serial2: XIAO ESP32 (zXiao.ino) or second F9P (zDualF9P.ino)

    autosteerSetup();                   // ADS1115 (WAS), steer settings from the EEPROM (Autosteer.ino)
    EthernetStart();
    webSetup();                         // web page on port 80 (zWeb.ino)
    netPollFromLoop();                  // from now on the network stack runs from loop() (zNet.h)

    // zImu.ino: TM171 or BNO085 (RVC) on Serial5, else BNO085 on I2C. Roll always comes from
    // it; heading whenever there is no valid dual heading (always with an F9P / X20P).
    imuSetup();

    CAN_Setup();

    // Auto-detect Keya motor by listening for its CAN heartbeat (ID 0x07000001).
    // Listen for 2 seconds - Keya heartbeats every 20ms so plenty of time.
    // If no heartbeat seen, assume hydraulic via Cytron.
    {
        uint32_t detectStart = millis();
        while (millis() - detectStart < 2000)
        {
            // Keep emptying the IMU serial buffer during this 2 s wait. The TM171 sends
            // ~25 packets/s (a BNO085 in RVC mode 100) and the 512-byte buffer fills in under
            // a second; the bytes dropped when it overflows cut a packet in half (bad CRC at start-up).
            imuProcess();
            dualF9pProcess();   // same for the GPS2 port: the second F9P,
            xiaoProcess();      // or the XIAO (its boot lines and first hello)
            netPoll();          // the network stack (zNet.h)

            KeyaBus_Receive();  // a heartbeat sets isKeya (KeyaCANBUS.ino)
            if (isKeya) break;
        }
    }
    if (!isKeya)
        StatusLog.line("[boot] No Keya heartbeat: steering by the valve / motor driver set in the program (Cytron, IBT2, Danfoss); a Keya found later is used");

    gnssFlushAfterSetup();   // drop the stale GPS data that piled up while setup() was blocking

    wdtStart();              // hardware watchdog, 2 s (zSafety.ino; can be switched off on the web page)

    StatusLog.line("[boot] Setup done in %lu.%lu s", (unsigned long)(millis() / 1000), (unsigned long)(millis() / 100 % 10));
}

// ----- Main Loop -----
// loopStep says which part runs, for the watchdog report (zSafety.ino, stepName)
void loop()
{
    wdtFeed();                                    // hardware watchdog (zSafety.ino)
    loopTick();                                   // how long the loops take, for the report (zSafety.ino)

    // the network stack: TCP timers, the web page's connections, link (zNet.h, zEthernet.ino)
    loopStep = 14;
    netPoll();

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
                netDelay(300);
                wdtFeed();
                while (SerialGPS->available()) SerialGPS->read(); // flush

                static uint8_t gi = 0;
                const char* pat = "GGA,";            // $GNGGA or $GPGGA
                uint32_t t = millis();
                while (millis() - t < 800)
                {
                    wdtFeed();
                    netPoll();
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

        // Forward to GPS (RTCM corrections pass through to the receiver; not while the receiver check
        // searches its baud rate, zRxCheck.ino)
        if (!passThroughGPS && !rxCkOwns(RXCK_GPS))
        {
            SerialGPS->write(incoming_char);
        }
    }

    // Read NMEA from the receiver (GGA, VTG, HPR)
    loopStep = 3;
    gnssNoteRxBacklog();                          // [status] warns if bytes pile up (Teensy reading too slowly)
    if (!rxCkOwns(RXCK_GPS) && SerialGPS->available())    // (the receiver check reads it while it searches its baud rate)
    {
        if (passThroughGPS)
        {
            SerialAOG.write(SerialGPS->read());
        }
        else
        {
            uint8_t gc = SerialGPS->read();
            rxCkBytes[RXCK_GPS]++;                // the data at 460800 (zRxCheck.ino)
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
            if (!rxCkOwns(RXCK_GPS)) SerialGPS->write(rb, n);
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

    // GPS2 port (Serial2): the second F9P of a dual F9P (zDualF9P.ino), or the XIAO ESP32: its hello,
    // its PGNs to UDP 9999, its text lines to the log (zXiao.ino) - one of them, chosen on the web page
    loopStep = 7;
    dualF9pProcess();
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
    rxCheckLoop();                                // the receiver check: data, baud rate, settings (zRxCheck.ino)

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
