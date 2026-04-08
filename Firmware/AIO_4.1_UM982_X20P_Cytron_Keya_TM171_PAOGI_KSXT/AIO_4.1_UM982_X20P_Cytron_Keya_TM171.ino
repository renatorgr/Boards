// UM982 dual-antenna code for AgOpenGPS
// One UM982 module with two antennas outputs GGA + VTG + KSXT on Serial7.
// KSXT carries heading, pitch, roll - no separate IMU needed when KSXT is valid.
// TM171 on Serial5 is the fallback IMU source if KSXT is unavailable.
// RTCM corrections from XBee LR radio (Serial3, 115200) forwarded to UM982 TX.
//
// Connections:
//   Serial7 RX (pin 28) <- UM982/X20P TX  (GGA, VTG, KSXT)
//   Serial7 TX (pin 29) -> UM982/X20P RX  (RTCM corrections in)
//   Serial3             <- XBee LR radio TX (RTCM at 115200)
//   Serial5             <- TM171 IMU TX (fallback, 115200)
//

/************************* User Settings *************************/
#define SerialAOG Serial                //AgIO USB connection
#define SerialRTK Serial3               //RTK radio (XBee LR board)
HardwareSerial* SerialGPS = &Serial7;   //UM982/X20P - GGA, VTG, KSXT all on one port

const int32_t baudAOG = 115200;
const int32_t baudGPS = 460800;         // set to match your UM982/X20P baud rate
const int32_t baudRTK = 115200;         // XBee LR

#define ImuWire Wire                    //SCL=19:A5 SDA=18:A4
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

const bool invertRoll = true;

// Status LEDs
#define GGAReceivedLED 13
#define Power_on_LED 5
#define Ethernet_Active_LED 6
#define GPSRED_LED 9                    // Red  (Flashing = no IMU/dual, ON = fix + IMU)
#define GPSGREEN_LED 10                 // Green (ON = KSXT/dual heading good)
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

// UM982 KSXT is the primary heading source.
// TM171 on Serial5 always runs: roll/pitch continuously, heading as fallback.
bool useDual  = false;   // never true; heading comes from KSXT or TM171
bool useTM171 = false;   // set true unconditionally in setup

// Required by Autosteer.ino and KeyaCANBUS.ino
bool useBNO08x = false;  // no BNO08x in this build
bool useCMPS   = false;  // no CMPS in this build
int8_t KeyaCurrentSensorReading = 0;  // Keya motor current, updated by KeyaCANBUS.ino

// Auto-detected in setup() by listening for Keya CAN heartbeat.
// true  = Keya motor detected -> CAN bus steering
// false = no Keya detected    -> Hydraulic valve via Cytron PWM
bool isKeya = false;

elapsedMillis TM171lastData;

// Heading filled by KSXT_Handler; roll/pitch filled by TM171
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
    parser.addHandler("KSXT-",  KSXT_Handler);  // UM982 dual-antenna heading+roll+pitch

    delay(10);
    Serial.begin(baudAOG);
    delay(10);
    Serial.println("Start AIO 4.5 Micro setup");

    // UM982 on Serial7 - fixed, no auto-detection needed
    Serial.println("\r\nGNSS on Serial7");
    SerialGPS = &Serial7;
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
    // TM171 is always connected on Serial5 - start it directly.
    // It runs continuously alongside UM982 KSXT for roll/pitch at all times.
    TM171detectOnPort(&Serial5, 1500);  // initialises SerialImu and begins Serial5
    useTM171 = true;
    Serial.println("TM171 started on Serial5");
    Serial.println("TM171 provides roll/pitch always; heading when KSXT unavailable");

    Serial.println("\r\nStarting CANBUS...");
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

    Serial.println("\r\nEnd setup, waiting for GNSS signal...\r\n");
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
                    SerialAOG.println("UM982/X20P not detected at 460800. Check for faults.");
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

        // Forward to GPS (RTCM corrections pass through to UM982/X20P)
        if (!passThroughGPS)
        {
            SerialGPS->write(incoming_char);
        }
    }

    // Read NMEA from UM982/X20P (GGA, VTG, KSXT)
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

    // Forward RTK radio (XBee LR) corrections to UM982/X20P
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

    // TM171 always runs - provides roll/pitch at all times, heading as fallback
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
