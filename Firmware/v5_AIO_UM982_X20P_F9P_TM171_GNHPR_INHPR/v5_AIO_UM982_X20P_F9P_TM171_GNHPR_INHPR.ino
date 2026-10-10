// v5_AIO_UM982_X20P_F9P_TM171_GNHPR_INHPR.ino
// Build for a fixed hardware configuration:
//   GPS/heading : UM982 dual antenna (GGA + VTG + HPR on Serial7, 10 Hz), or
//                 u-blox F9P / X20P single antenna (GGA + VTG) - detected automatically.
//   IMU         : TM171 (Serial5) - always present.
//   Output      : per GGA epoch, the standard sentence set to AgOpenWeb (UDP 9999),
//                 see zHandlers.ino:
//                   $GNGGA  position          $GNVTG  speed
//                   $GNTHS  dual-antenna heading (UM982 HPR), V when there is none
//                   $INHPR  TM171 heading + roll (IMU talker)
//                 Needs AgOpenWeb with the standard-set epoch assembler (PR #300).
//   Steering    : Cytron MD13S (hydraulic valve) or Keya CAN motor,
//                 auto-detected at boot
//   RTK link    : XBee LR radio on Serial3
//
// This is not a general-purpose AIO firmware: there is no BNO085, no
// CMPS, no second GPS port. The receiver type is only told apart by
// whether it sends HPR (UM982 dual antenna) or not (F9P / X20P).
//
// Connections:
//   Serial7 RX (pin 28) <- receiver TX  (GGA, VTG, HPR)
//   Serial7 TX (pin 29) -> receiver RX  (RTCM corrections in)
//   Serial5              <- TM171 IMU TX (115200, always running)
//   Serial3              <- XBee LR radio TX (RTCM at 115200)
//

/************************* User Settings *************************/
#define SerialAOG Serial                //AgIO USB connection
#define SerialRTK Serial3               //RTK radio (XBee LR board)
HardwareSerial* SerialGPS = &Serial7;   //UM982 (GGA + VTG + HPR) or F9P / X20P (GGA + VTG)

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
#include <Wire.h>

#include <FlexCAN_T4.h>
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_256> Keya_Bus;

extern "C" uint32_t set_arm_clock(uint32_t frequency);

// Required by Autosteer.ino and KeyaCANBUS.ino
int8_t KeyaCurrentSensorReading = 0;  // Keya motor current, updated by KeyaCANBUS.ino

// Auto-detected in setup() by listening for Keya CAN heartbeat.
// true  = Keya motor detected -> CAN bus steering
// false = no Keya detected    -> Hydraulic valve via Cytron PWM
bool isKeya = false;

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
    Serial.println("Start v5 AIO UM982/X20P/F9P TM171 GNHPR INHPR setup");

    Serial.println("GNSS receiver on Serial7");
    SerialGPS->begin(baudGPS);
    SerialGPS->addMemoryForRead(GPSrxbuffer,  serial_buffer_size);
    SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

    delay(10);
    SerialRTK.begin(baudRTK);
    SerialRTK.addMemoryForRead(RTKrxbuffer, serial_buffer_size);

    Serial.println("SerialAOG, SerialRTK, SerialGPS initialized");

    Serial.println("\r\nStarting AutoSteer...");
    autosteerSetup();

    Serial.println("\r\nStarting Ethernet...");
    EthernetStart();

    Serial.println("\r\nStarting TM171 IMU on Serial5...");
    // TM171 is always connected on this board. Roll always comes from it;
    // heading comes from it whenever there is no valid dual heading (always with an F9P / X20P).
    if (TM171detectOnPort(&Serial5, 1500))
        Serial.println("TM171 OK on Serial5");
    else
        Serial.println("TM171 NOT detected on Serial5 (check wiring/baud)");

    Serial.println("Starting CANBUS...");
    CAN_Setup();

    // Auto-detect Keya motor by listening for its CAN heartbeat (ID 0x07000001).
    // Listen for 2 seconds - Keya heartbeats every 20ms so plenty of time.
    // If no heartbeat seen, assume hydraulic via Cytron.
    Serial.println("Detecting steering type (Keya CAN / Hydraulic)...");
    {
        CAN_message_t msg;
        uint32_t detectStart = millis();
        while (millis() - detectStart < 2000)
        {
            // Keep emptying the TM171 serial buffer during this 2 s wait. It sends
            // ~25 packets/s and its 512-byte buffer fills in under a second; the bytes
            // dropped when it overflows cut a packet in half (bad CRC at start-up).
            TM171process();

            if (Keya_Bus.read(msg))
            {
                if (msg.id == 0x07000001)
                {
                    isKeya = true;
                    break;
                }
            }
        }
    }
    if (isKeya)
        Serial.println("Keya motor detected -> CAN bus steering");
    else
        Serial.println("Hydraulic steering via Cytron PWM");

    gnssFlushAfterSetup();   // drop the stale GPS data that piled up while setup() was blocking

    Serial.println("\r\nEnd setup, waiting for GPS...\r\n");
}

// ----- Main Loop -----
void loop()
{
    KeyaBus_Receive();

    // Forward incoming bytes from AgIO/NTRIP to the receiver (RTCM corrections)
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
                while (SerialGPS->available()) SerialGPS->read(); // flush

                static uint8_t gi = 0;
                const char* pat = "GGA,";            // $GNGGA or $GPGGA
                uint32_t t = millis();
                while (millis() - t < 800)
                {
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
    if (SerialGPS->available())
    {
        if (passThroughGPS)
        {
            SerialAOG.write(SerialGPS->read());
        }
        else
        {
            uint8_t gc = SerialGPS->read();
            rawNmeaTee((char)gc);                 // [raw] lines on USB when RAW_NMEA_DEBUG = 1
            parser << gc;
        }
    }

    udpNtrip();

    // Forward RTK radio (XBee LR) corrections to the receiver
    if (SerialRTK.available())
    {
        SerialGPS->write(SerialRTK.read());
    }

    // GGA timeout - turn off GPS LEDs after 10 sec with no fix
    if ((systick_millis_count - gpsReadyTime) > 10000)
    {
        digitalWrite(GPSRED_LED,   LOW);
        digitalWrite(GPSGREEN_LED, LOW);
    }

    // TM171 always runs - roll at all times, heading when single antenna
    TM171process();

    // Per GGA epoch: $GNGGA, $GNVTG, $GNTHS (dual heading), $INHPR (TM171) to AgOpenWeb
    gnssProcess();

    // [status] messages on the serial monitor when Ethernet / AgOpenWeb / GPS / dual change (zStatus.ino)
    statusUpdate();

    if (Autosteer_running) autosteerLoop();
    else ReceiveUdp();

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
