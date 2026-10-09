// v4_UM982_TM171.ino
// DEDICATED build for a fixed hardware configuration:
//   GPS/heading : UM982 (GGA + KSXT on Serial7). VTG and HPR are NOT used.
//   IMU         : TM171 (Serial5) - always present.
//   Output      : one sentence per GGA epoch to AgOpenWeb (UDP 9999), see zHandlers.ino:
//                 dual antenna (KSXT heading float/fixed): the $KSXT line, forwarded as is
//                 single antenna (KSXT absent/invalid): $PANDA = GGA + TM171 heading/roll
//   Steering    : Cytron MD13S (hydraulic valve) or Keya CAN motor,
//                 auto-detected at boot
//   RTK link    : XBee LR radio on Serial3
//
// This is not a general-purpose AIO firmware: there is no BNO085, no
// CMPS, no single-F9P mode, no runtime GPS-type detection. Every
// conditional that existed only to support those other configurations
// has been removed so the code always does exactly what this specific
// board does.
//
// Connections:
//   Serial7 RX (pin 28) <- UM982 TX  (GGA, KSXT)
//   Serial7 TX (pin 29) -> UM982 RX  (RTCM corrections in)
//   Serial5              <- TM171 IMU TX (115200, always running)
//   Serial3              <- XBee LR radio TX (RTCM at 115200)
//

/************************* User Settings *************************/
#define SerialAOG Serial                //AgIO USB connection
#define SerialRTK Serial3               //RTK radio (XBee LR board)
HardwareSerial* SerialGPS = &Serial7;   //UM982 - GGA + KSXT on one port

const int32_t baudAOG = 115200;
const int32_t baudGPS = 460800;         // must match UM982 COM port baud rate
const int32_t baudRTK = 115200;         // XBee LR

#define ImuWire Wire                    //SCL=19:A5 SDA=18:A4 - ADS1115 (WAS)
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

// Status LEDs
#define GGAReceivedLED 13
#define Power_on_LED 5
#define Ethernet_Active_LED 6
#define GPSRED_LED 9                    // Red   (ON = single antenna / TM171 heading, blinking = TM171 lost)
#define GPSGREEN_LED 10                 // Green (ON = UM982 dual heading (KSXT) good)
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

/* A parser with 2 handlers (GGA, KSXT) */
NMEAParser<2> parser;

bool isTriggered = false;
bool blink       = false;

bool Autosteer_running = true;
bool Ethernet_running  = false;
bool GGA_Available     = false;

// Pass-through for GPS config tool
bool passThroughGPS  = false;
bool passThroughGPS2 = false;

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
    parser.addHandler("G-GGA",  GGA_Handler);
    parser.addHandler("KSXT-",  KSXT_Handler);  // UM982 dual-antenna heading ($KSXT, no talker id)

    delay(10);
    Serial.begin(baudAOG);
    delay(10);
    Serial.println("Start v4 UM982 & TM171 $PANDA $KSXT setup");

    Serial.println("UM982 on Serial7");
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
    // heading comes from it whenever the UM982 has no valid dual heading.
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
        Serial.println("No Keya detected -> Hydraulic / Cytron PWM steering");

    gnssFlushAfterSetup();   // drop the stale GPS data that piled up while setup() was blocking

    Serial.println("\r\nEnd setup, waiting for GPS...\r\n");
}

// ----- Main Loop -----
void loop()
{
    KeyaBus_Receive();

    // Forward incoming bytes from AgIO/NTRIP to UM982 (RTCM corrections)
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
                passThroughGPS2 = false;
                bool found = false;

                Serial.print("Checking for NMEA at "); Serial.println(baudGPS);
                SerialGPS->begin(baudGPS);
                delay(300);
                while (SerialGPS->available()) SerialGPS->read(); // flush

                static uint8_t gi = 0;
                const char* pat = "$GNGGA";
                uint32_t t = millis();
                while (millis() - t < 800)
                {
                    if (SerialGPS->available())
                    {
                        char c = SerialGPS->read();
                        if (c == pat[gi]) { gi++; if (gi == 6) { gi = 0; found = true; break; } }
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
                    SerialAOG.println("UM982 not detected at 460800. Check for faults.");
                }
            }
            else if (aogSerialCmdBuffer[aogSerialCmdCounter] == 'E' &&
                     aogSerialCmdBuffer[aogSerialCmdCounter + 1] == 'D')
            {
                passThroughGPS  = false;
                passThroughGPS2 = false;
            }
            aogSerialCmdCounter = 0;
        }
        else
        {
            aogSerialCmdCounter = 0;
        }

        // Forward to GPS (RTCM corrections pass through to UM982)
        if (!passThroughGPS)
        {
            SerialGPS->write(incoming_char);
        }
    }

    // Read NMEA from UM982 (GGA, KSXT)
    if (SerialGPS->available())
    {
        if (passThroughGPS)
        {
            SerialAOG.write(SerialGPS->read());
        }
        else
        {
            uint8_t gc = SerialGPS->read();
            rawNmeaTee((char)gc);                 // keeps the line: KSXT is forwarded verbatim
            parser << gc;
        }
    }

    udpNtrip();

    // Forward RTK radio (XBee LR) corrections to UM982
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

    // Send ONE sentence per GGA epoch: $KSXT (dual) or $PANDA (single antenna)
    gnssProcess();

    // [estado] messages on the serial monitor when Ethernet / AgOpenWeb / GPS / dual change (zStatus.ino)
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

// Checksum for UBX-style AgIO internal packets (used in Autosteer.ino PGN packets)
// NOT related to GPS - do NOT remove!
bool calcChecksum()
{
    // Not used for GPS anymore (no RELPOSNED), kept as stub to avoid link errors if any lingering reference exists
    return false;
}
