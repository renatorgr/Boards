// AIO_F9P_X20P_TM171_Dedicated.ino
// DEDICATED build for a fixed hardware configuration:
//   GPS      : F9P or X20P, single antenna (GGA + VTG on Serial7)
//   IMU      : TM171 (Serial5) - always present, sole source of
//              heading, roll and pitch
//   Steering : Cytron MD13S (hydraulic valve) or Keya CAN motor,
//              auto-detected at boot
//   RTK link : XBee LR radio on Serial3
//
// This is not a general-purpose AIO firmware: there is no dual-antenna
// support, no KSXT parsing, no UM982-specific handling, no BNO085, no
// CMPS, no runtime GPS-type detection. Every conditional that existed
// only to support those other configurations has been removed so the
// code always does exactly what this specific board does.
//
// Connections:
//   Serial7 RX (pin 28) <- F9P/X20P TX  (GGA, VTG)
//   Serial7 TX (pin 29) -> F9P/X20P RX  (RTCM corrections in)
//   Serial5              <- TM171 IMU TX (115200, always running)
//   Serial3              <- XBee LR radio TX (RTCM at 115200)
//

/************************* User Settings *************************/
#define SerialAOG Serial                //AgIO USB connection
#define SerialRTK Serial3               //RTK radio (XBee LR board)
HardwareSerial* SerialGPS = &Serial7;   //F9P/X20P - GGA, VTG on one port

const int32_t baudAOG = 115200;
const int32_t baudGPS = 460800;         // must match F9P/X20P UART port baud rate
                                         // (typical F9P default is 115200 unless reconfigured)
const int32_t baudRTK = 115200;         // XBee LR

#define ImuWire Wire                    //SCL=19:A5 SDA=18:A4 - ADS1115 (WAS)
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

// Status LEDs
#define GGAReceivedLED 13
#define Power_on_LED 5
#define Ethernet_Active_LED 6
#define GPSRED_LED 9                    // Red   (unused fallback indicator - always off)
#define GPSGREEN_LED 10                 // Green (ON = GGA + TM171 heading available)
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

// Heading always comes from TM171 (filled in GGA_Handler)
double heading = 0;

// Kept to avoid linker errors from stub functions
float roll  = 0;
float pitch = 0;
float yaw   = 0;
double correctionHeading = 0;

/* A parser with 5 handlers */
NMEAParser<5> parser;

bool isTriggered = false;
bool blink       = false;

bool Autosteer_running = true;
bool Ethernet_running  = false;
bool GGA_Available     = false;

// Pass-through for GPS config tool
bool passThroughGPS  = false;
bool passThroughGPS2 = false;  // kept to avoid breaking BuildNmea refs

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
    parser.addHandler("G-VTG",  VTG_Handler);
    // No KSXT handler on this board - single antenna, no dual heading sentence

    delay(10);
    Serial.begin(baudAOG);
    delay(10);
    Serial.println("Start AIO F9P/X20P+TM171 dedicated setup");

    Serial.println("F9P/X20P on Serial7");
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
    // TM171 is always connected on this board - start it unconditionally.
    // It is the sole source of heading, roll and pitch.
    TM171detectOnPort(&Serial5, 1500);
    Serial.println("TM171 started on Serial5");

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

    Serial.println("\r\nEnd setup, waiting for GPS...\r\n");
}

// ----- Main Loop -----
void loop()
{
    KeyaBus_Receive();

    // Forward incoming bytes from AgIO/NTRIP to F9P/X20P (RTCM corrections)
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
                // !AOGR1 - verify NMEA GGA at fixed baud
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
                    SerialAOG.println("F9P/X20P not detected at configured baud. Check for faults.");
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

        // Forward to GPS (RTCM corrections pass through to F9P/X20P)
        if (!passThroughGPS)
        {
            SerialGPS->write(incoming_char);
        }
    }

    // Read NMEA from F9P/X20P (GGA, VTG)
    if (SerialGPS->available())
    {
        if (passThroughGPS)
        {
            SerialAOG.write(SerialGPS->read());
        }
        else
        {
            parser << SerialGPS->read();
        }
    }

    udpNtrip();

    // Forward RTK radio (XBee LR) corrections to F9P/X20P
    if (SerialRTK.available())
    {
        SerialGPS->write(SerialRTK.read());
    }

    // GGA timeout - turn off GPS LED after 10 sec with no fix
    if ((systick_millis_count - gpsReadyTime) > 10000)
    {
        digitalWrite(GPSRED_LED,   LOW);
        digitalWrite(GPSGREEN_LED, LOW);
    }

    // TM171 always runs - sole source of heading, roll and pitch
    TM171process();
    // Note: sentence building happens in GGA_Handler, not here

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
